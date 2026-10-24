//! A single-task async runtime that owns the [`World`] between await points.
//!
//! Bevy's asset loading is multi-frame, which normally pushes everything that depends on it into
//! observers, and every ordering constraint becomes "whichever callback happened to fire last".
//! Here the same work is a straight-line `async fn`:
//!
//! ```ignore
//! async fn load(w: AsyncWorld) {
//!     w.spawn(WorldAssetRoot(ground), GroundRoot).await;   // resolves when the instance is ready
//!     w.spawn(WorldAssetRoot(robot), Infantry::default()).await;
//! }
//! ```
//!
//! # How it works
//!
//! The task never holds a [`World`] reference across an await point. A suspended task instead
//! leaves behind a *job* — a boxed `FnOnce(&mut World)` — which [`drive_async_world`], an exclusive
//! system, runs on its behalf before polling again. Jobs are served in a loop within one frame, so
//! consecutive [`AsyncWorld::with_world`] calls execute back to back.
//!
//! Waiting is event-driven, not polled. [`AsyncWorld::spawn`] attaches an entity observer that
//! fires a [`Signal`]; firing it wakes the task's [`Waker`], which is the only thing that makes
//! [`drive_async_world`] poll again. Between a spawn and its `WorldInstanceReady`, the task costs
//! nothing.

use bevy::asset::WaitForAssetError;
use bevy::ecs::event::EntityEvent;
use bevy::ecs::system::RunSystemOnce;
use bevy::prelude::*;
use bevy::world_serialization::{WorldAssetRoot, WorldInstanceReady};
use futures::channel::oneshot;
use std::collections::VecDeque;
use std::future::{Future, poll_fn};
use std::pin::Pin;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};
use std::task::{Context, Poll, Wake, Waker};

/// Work a suspended task wants performed on the world before it is polled again.
type WorldJob = Box<dyn FnOnce(&mut World) + Send>;

/// Setting this flag is the only thing waking the task does; the driver checks it before polling.
struct TaskWaker {
    woken: AtomicBool,
}

impl Wake for TaskWaker {
    fn wake(self: Arc<Self>) {
        self.wake_by_ref();
    }

    fn wake_by_ref(self: &Arc<Self>) {
        self.woken.store(true, Ordering::Release);
    }
}

/// A notification that completes after a fixed number of arrivals, awaited by the task.
///
/// Arriving before anyone awaits is fine — the wait then completes immediately — so there is no
/// race between an observer running and the task reaching its await point.
pub struct Signal {
    state: Mutex<SignalState>,
}

enum SignalState {
    Pending {
        remaining: usize,
        waker: Option<Waker>,
    },
    Fired,
}

impl Signal {
    /// A signal that completes after `count` arrivals. `0` is already complete.
    pub fn new(count: usize) -> Arc<Self> {
        Arc::new(Self {
            state: Mutex::new(match count {
                0 => SignalState::Fired,
                remaining => SignalState::Pending {
                    remaining,
                    waker: None,
                },
            }),
        })
    }

    /// Records one arrival, waking the task on the last one. Extra arrivals are ignored.
    pub fn arrive(&self) {
        let mut state = self.state.lock().unwrap();
        let last = match &mut *state {
            SignalState::Fired => None,
            SignalState::Pending { remaining, waker } => {
                *remaining = remaining.saturating_sub(1);
                (*remaining == 0).then(|| waker.take())
            }
        };
        let Some(waker) = last else {
            return;
        };
        *state = SignalState::Fired;
        drop(state);
        if let Some(waker) = waker {
            waker.wake();
        }
    }

    /// Resolves once every arrival has been recorded.
    pub async fn wait(self: Arc<Self>) {
        poll_fn(move |cx| {
            let mut state = self.state.lock().unwrap();
            match &mut *state {
                SignalState::Fired => Poll::Ready(()),
                SignalState::Pending { waker, .. } => {
                    *waker = Some(cx.waker().clone());
                    Poll::Pending
                }
            }
        })
        .await
    }
}

/// The rendezvous between a suspended task and [`drive_async_world`].
///
/// Each job owns its typed reply channel. Cloned handles can queue independent operations;
/// the driver still executes all world mutations sequentially.
struct TaskChannel {
    jobs: Mutex<VecDeque<WorldJob>>,
    waker: Arc<TaskWaker>,
}

/// Handle passed to an async task, granting deferred access to the world.
#[derive(Clone)]
pub struct AsyncWorld {
    channel: Arc<TaskChannel>,
}

impl AsyncWorld {
    /// Waits for the asset and its dependencies, then spawns its complete world instance.
    ///
    /// Taking the [`WorldAssetRoot`] separately from the rest of the bundle is what makes the wait
    /// safe: an entity that never loads a world would never become ready, and this signature makes
    /// that unrepresentable.
    pub async fn spawn(
        &self,
        root: WorldAssetRoot,
        extra: impl Bundle,
    ) -> Result<Entity, WaitForAssetError> {
        let assets = self
            .with_world(|world| world.resource::<AssetServer>().clone())
            .await;
        assets.wait_for_asset(&root.0).await?;
        Ok(self
            .spawn_observing::<WorldInstanceReady>((root, extra))
            .await)
    }

    /// Spawns `bundle` and resolves when `E` is first triggered on the new entity.
    ///
    /// The observer is attached in the same world job as the spawn, so the event cannot be missed.
    pub async fn spawn_observing<E: EntityEvent>(&self, bundle: impl Bundle) -> Entity {
        let ready = Signal::new(1);
        let arrival = ready.clone();

        let entity = self
            .with_world(move |world| {
                let entity = world.spawn_empty().id();
                observe_once::<E>(world, entity, arrival);
                world.entity_mut(entity).insert(bundle);
                entity
            })
            .await;

        ready.wait().await;
        entity
    }

    /// Resolves once `E` has been triggered on every entity `select` returns.
    ///
    /// Selection and observation share one world job, so no entity can fire its event in between.
    /// Selecting nothing resolves immediately.
    pub async fn observe_all<E, S>(&self, select: S)
    where
        E: EntityEvent,
        S: FnOnce(&mut World) -> Vec<Entity> + Send + 'static,
    {
        let done = self
            .with_world(move |world| {
                let entities = select(world);
                let signal = Signal::new(entities.len());
                for entity in entities {
                    observe_once::<E>(world, entity, signal.clone());
                }
                signal
            })
            .await;

        done.wait().await;
    }

    /// Runs a one-shot system with `input` and resolves to its output.
    ///
    /// Lets setup that was written as an observer stay a normal Bevy system — queries, `Commands`
    /// and all — while being called at an explicit point in the sequence instead of whenever an
    /// event happens to fire. Deferred commands are applied before this resolves.
    pub async fn run<I, O, M, S>(&self, system: S, input: I) -> O
    where
        S: IntoSystem<In<I>, O, M> + Send + 'static,
        I: Send + 'static,
        O: Send + 'static,
        M: 'static,
    {
        self.with_world(move |world| {
            world
                .run_system_once_with(system, input)
                .expect("async world one-shot system failed")
        })
        .await
    }

    /// Runs `job` on the world and resolves to its return value.
    ///
    /// Resolves within the same frame: the driver serves the job and immediately polls again.
    pub async fn with_world<T, F>(&self, job: F) -> T
    where
        F: FnOnce(&mut World) -> T + Send + 'static,
        T: Send + 'static,
    {
        let (reply, result) = oneshot::channel();
        self.channel
            .jobs
            .lock()
            .unwrap()
            .push_back(Box::new(move |world| {
                let _ = reply.send(job(world));
            }));
        self.channel.waker.wake_by_ref();
        result
            .await
            .expect("async world driver dropped a queued job")
    }
}

/// Records a single arrival on `signal` the first time `E` fires on `entity`.
///
/// Events like asset reloads can fire more than once; the latch keeps a repeat from consuming
/// another entity's arrival.
fn observe_once<E: EntityEvent>(world: &mut World, entity: Entity, signal: Arc<Signal>) {
    let arrived = AtomicBool::new(false);
    world
        .entity_mut(entity)
        .observe(move |event: On<E>, mut commands: Commands| {
            if !arrived.swap(true, Ordering::Relaxed) {
                signal.arrive();
                commands.entity(event.observer()).despawn();
            }
        });
}

/// A running async task, stored as a resource for [`drive_async_world`] to poll.
#[derive(Resource)]
pub struct AsyncWorldTask {
    // `Mutex` only to satisfy `Resource`'s `Sync` bound; the driver always has exclusive access.
    future: Mutex<Pin<Box<dyn Future<Output = ()> + Send>>>,
    channel: Arc<TaskChannel>,
}

impl AsyncWorldTask {
    /// Builds a task from a function that receives the world handle.
    pub fn new<F, Fut>(task: F) -> Self
    where
        F: FnOnce(AsyncWorld) -> Fut,
        Fut: Future<Output = ()> + Send + 'static,
    {
        let channel = Arc::new(TaskChannel {
            jobs: Mutex::new(VecDeque::new()),
            waker: Arc::new(TaskWaker {
                woken: AtomicBool::new(true),
            }),
        });
        let future = Mutex::new(Box::pin(task(AsyncWorld {
            channel: channel.clone(),
        })) as Pin<Box<dyn Future<Output = ()> + Send>>);

        Self { future, channel }
    }
}

/// Polls the task, serving its world jobs, until it blocks on a [`Signal`] or finishes.
///
/// Removes [`AsyncWorldTask`] once the task completes, which also stops this system running.
pub fn drive_async_world(world: &mut World) {
    let Some(mut task) = world.remove_resource::<AsyncWorldTask>() else {
        return;
    };

    let waker = Waker::from(task.channel.waker.clone());
    let mut cx = Context::from_waker(&waker);
    let future = task
        .future
        .get_mut()
        .expect("async world task future poisoned");

    while task.channel.waker.woken.swap(false, Ordering::AcqRel) {
        if future.as_mut().poll(&mut cx).is_ready() {
            return;
        }

        // Serving a job re-wakes the task, so the loop continues without waiting for a frame.
        let jobs = std::mem::take(&mut *task.channel.jobs.lock().unwrap());
        if jobs.is_empty() {
            break;
        }
        for job in jobs {
            job(world);
        }
    }

    world.insert_resource(task);
}

#[cfg(test)]
mod tests {
    use super::*;

    #[derive(Resource, Default, PartialEq, Debug)]
    struct Steps(Vec<&'static str>);

    #[test]
    fn concurrent_world_jobs_keep_their_typed_results() {
        let mut world = World::new();
        world.init_resource::<Steps>();
        world.insert_resource(AsyncWorldTask::new(|w: AsyncWorld| async move {
            let other = w.clone();
            let (number, text) = futures::join!(
                w.with_world(|world| {
                    world.resource_mut::<Steps>().0.push("number");
                    42_u32
                }),
                other.with_world(|world| {
                    world.resource_mut::<Steps>().0.push("text");
                    String::from("ready")
                }),
            );
            assert_eq!(number, 42);
            assert_eq!(text, "ready");
        }));
        drive_async_world(&mut world);
        assert_eq!(world.resource::<Steps>().0, ["number", "text"]);
        assert!(!world.contains_resource::<AsyncWorldTask>());
    }

    #[test]
    fn spawn_observes_events_fired_during_bundle_insertion() {
        #[derive(Component)]
        struct Spawned;
        #[derive(EntityEvent)]
        struct Ready {
            entity: Entity,
        }

        let mut world = World::new();
        world
            .register_component_hooks::<Spawned>()
            .on_add(|mut world, context| {
                world.trigger(Ready {
                    entity: context.entity,
                });
            });
        world.init_resource::<Steps>();
        world.insert_resource(AsyncWorldTask::new(|w: AsyncWorld| async move {
            w.spawn_observing::<Ready>(Spawned).await;
            w.with_world(|world| world.resource_mut::<Steps>().0.push("spawned"))
                .await;
        }));
        drive_async_world(&mut world);
        assert_eq!(world.resource::<Steps>().0, ["spawned"]);
        assert!(!world.contains_resource::<AsyncWorldTask>());
        assert_eq!(world.query::<&Observer>().iter(&world).count(), 0);
    }

    #[test]
    fn repeated_events_do_not_complete_another_entitys_wait() {
        #[derive(EntityEvent)]
        struct Ready {
            entity: Entity,
        }

        let mut world = World::new();
        world.init_resource::<Steps>();
        let first = world.spawn_empty().id();
        let second = world.spawn_empty().id();
        world.insert_resource(AsyncWorldTask::new(move |w: AsyncWorld| async move {
            w.observe_all::<Ready, _>(move |_| vec![first, second])
                .await;
            w.with_world(|world| world.resource_mut::<Steps>().0.push("ready"))
                .await;
        }));
        drive_async_world(&mut world);
        world.trigger(Ready { entity: first });
        world.trigger(Ready { entity: first });
        drive_async_world(&mut world);
        assert!(world.resource::<Steps>().0.is_empty());
        world.trigger(Ready { entity: second });
        drive_async_world(&mut world);
        assert_eq!(world.resource::<Steps>().0, ["ready"]);
        assert_eq!(world.query::<&Observer>().iter(&world).count(), 0);
    }

    #[test]
    fn consecutive_world_jobs_run_in_one_frame() {
        let mut world = World::new();
        world.init_resource::<Steps>();
        world.insert_resource(AsyncWorldTask::new(|w: AsyncWorld| async move {
            for step in ["a", "b", "c"] {
                w.with_world(move |world| world.resource_mut::<Steps>().0.push(step))
                    .await;
            }
        }));

        drive_async_world(&mut world);

        assert_eq!(world.resource::<Steps>().0, vec!["a", "b", "c"]);
        assert!(!world.contains_resource::<AsyncWorldTask>());
    }

    #[test]
    fn a_task_awaiting_a_signal_resumes_when_it_fires() {
        #[derive(Resource, Deref)]
        struct Trigger(Arc<Signal>);

        let signal = Signal::new(1);
        let mut world = World::new();
        world.init_resource::<Steps>();
        world.insert_resource(Trigger(signal.clone()));
        world.insert_resource(AsyncWorldTask::new(|w: AsyncWorld| async move {
            let signal = w
                .with_world(|world| world.resource::<Trigger>().0.clone())
                .await;
            w.with_world(|world| world.resource_mut::<Steps>().0.push("before"))
                .await;
            signal.wait().await;
            w.with_world(|world| world.resource_mut::<Steps>().0.push("after"))
                .await;
        }));

        drive_async_world(&mut world);
        assert_eq!(world.resource::<Steps>().0, vec!["before"]);

        // Not woken: polling again must not advance the task.
        drive_async_world(&mut world);
        assert_eq!(world.resource::<Steps>().0, vec!["before"]);

        signal.arrive();
        drive_async_world(&mut world);
        assert_eq!(world.resource::<Steps>().0, vec!["before", "after"]);
        assert!(!world.contains_resource::<AsyncWorldTask>());
    }

    #[test]
    fn a_counting_signal_waits_for_every_arrival() {
        #[derive(Resource, Deref)]
        struct Trigger(Arc<Signal>);

        let signal = Signal::new(3);
        let mut world = World::new();
        world.init_resource::<Steps>();
        world.insert_resource(Trigger(signal.clone()));
        world.insert_resource(AsyncWorldTask::new(|w: AsyncWorld| async move {
            let signal = w
                .with_world(|world| world.resource::<Trigger>().0.clone())
                .await;
            signal.wait().await;
            w.with_world(|world| world.resource_mut::<Steps>().0.push("all arrived"))
                .await;
        }));

        drive_async_world(&mut world);
        for _ in 0..2 {
            signal.arrive();
            drive_async_world(&mut world);
            assert!(world.resource::<Steps>().0.is_empty());
        }

        signal.arrive();
        drive_async_world(&mut world);
        assert_eq!(world.resource::<Steps>().0, vec!["all arrived"]);

        // Late arrivals must not panic or resurrect the signal.
        signal.arrive();
    }

    #[test]
    fn an_empty_signal_is_already_complete() {
        let mut world = World::new();
        world.init_resource::<Steps>();
        world.insert_resource(AsyncWorldTask::new(|w: AsyncWorld| async move {
            Signal::new(0).wait().await;
            w.with_world(|world| world.resource_mut::<Steps>().0.push("skipped"))
                .await;
        }));

        drive_async_world(&mut world);

        assert_eq!(world.resource::<Steps>().0, vec!["skipped"]);
        assert!(!world.contains_resource::<AsyncWorldTask>());
    }

    #[test]
    fn a_signal_fired_before_the_wait_does_not_deadlock() {
        let signal = Signal::new(1);
        signal.arrive();

        let mut world = World::new();
        world.init_resource::<Steps>();
        world.insert_resource(AsyncWorldTask::new(move |w: AsyncWorld| async move {
            signal.wait().await;
            w.with_world(|world| world.resource_mut::<Steps>().0.push("done"))
                .await;
        }));

        drive_async_world(&mut world);

        assert_eq!(world.resource::<Steps>().0, vec!["done"]);
        assert!(!world.contains_resource::<AsyncWorldTask>());
    }
}
