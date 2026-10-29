use crate::robomaster::power_rune::common::RuneHitOutcome;
use crate::robomaster::power_rune::rotation::PowerRuneRotation;
use crate::robomaster::power_rune::rune::PowerRuneMechanism;
use avian3d::prelude::{CcdFilter, CollisionEnd, CollisionEventsEnabled, SweptCcd};
use bevy::prelude::{
    ChildOf, Commands, Component, Entity, EntityEvent, On, Query, ResMut, Resource, Update, With,
};
use std::collections::HashSet;

#[derive(Component)]
// Avian 0.8 only sweeps against static and kinematic bodies by default. Projectiles also
// need continuous collision detection against dynamic vehicles and standalone targets.
#[require(CollisionEventsEnabled, SweptCcd = SweptCcd::new().with_filter(CcdFilter::ALL))]
pub struct Projectile;

#[derive(Resource, Default)]
struct ConsumedRuneProjectiles(HashSet<Entity>);

#[derive(Component, Debug, Copy, Clone)]
pub struct RuneIndex {
    pub target: usize,
    pub rune: Entity,
}

#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub struct HitResult {
    pub outcome: RuneHitOutcome,
}

impl HitResult {
    pub const fn accurate(self) -> bool {
        self.outcome.is_accurate()
    }
}

#[derive(EntityEvent)]
pub struct RuneActivated {
    #[event_target]
    pub rune: Entity,
}

#[derive(EntityEvent)]
pub struct RuneHit {
    #[event_target]
    pub rune: Entity,
    pub projectile: Entity,
    pub result: HitResult,
}

fn handle_rune_collision(
    event: On<CollisionEnd>,
    mut commands: Commands,
    mut consumed_projectiles: ResMut<ConsumedRuneProjectiles>,
    mut runes: Query<(&mut PowerRuneMechanism, &mut PowerRuneRotation)>,
    targets: Query<&RuneIndex>,
    projectiles: Query<Entity, With<Projectile>>,
    child_of: Query<&ChildOf>,
) {
    let projectile_body1 = event.body1.and_then(|body| projectiles.get(body).ok());
    let projectile_body2 = event.body2.and_then(|body| projectiles.get(body).ok());

    let (projectile_entity, target_collider) = match (projectile_body1, projectile_body2) {
        (Some(projectile), _) => (projectile, event.collider2),
        (_, Some(projectile)) => (projectile, event.collider1),
        _ => return,
    };

    let Some(target) = find_rune_target(target_collider, &targets, &child_of) else {
        return;
    };

    let Ok((mut mechanism, mut rotation)) = runes.get_mut(target.rune) else {
        return;
    };

    if !consumed_projectiles.0.insert(projectile_entity) {
        return;
    }

    commands
        .entity(projectile_entity)
        .remove::<CollisionEventsEnabled>();

    let mut rng = rand::rng();
    let outcome = mechanism.state_mut().hit(target.target, &mut rng);
    rotation.sync_activation(
        mechanism.state().mode(),
        mechanism.state().is_activating(),
        &mut rng,
    );

    commands.trigger(RuneHit {
        rune: target.rune,
        projectile: projectile_entity,
        result: HitResult { outcome },
    });

    if outcome.activates_rune() {
        commands.trigger(RuneActivated { rune: target.rune });
    }
}

fn cleanup_consumed_rune_projectiles(
    mut consumed_projectiles: ResMut<ConsumedRuneProjectiles>,
    projectiles: Query<(), With<Projectile>>,
) {
    consumed_projectiles
        .0
        .retain(|entity| projectiles.contains(*entity));
}

fn find_rune_target(
    entity: Entity,
    targets: &Query<&RuneIndex>,
    child_of: &Query<&ChildOf>,
) -> Option<RuneIndex> {
    if let Ok(target) = targets.get(entity) {
        return Some(*target);
    }

    child_of
        .iter_ancestors(entity)
        .find_map(|ancestor| targets.get(ancestor).ok().copied())
}

#[derive(Default)]
pub(super) struct PowerRuneCollisionPlugin;

impl bevy::app::Plugin for PowerRuneCollisionPlugin {
    fn build(&self, app: &mut bevy::app::App) {
        app.init_resource::<ConsumedRuneProjectiles>()
            .add_systems(Update, cleanup_consumed_rune_projectiles)
            .add_observer(handle_rune_collision);
    }
}

#[cfg(test)]
mod tests {
    use super::Projectile;
    use avian3d::prelude::*;
    use bevy::{prelude::*, time::TimeUpdateStrategy};
    use std::time::Duration;

    #[test]
    fn fast_projectiles_do_not_tunnel_through_thin_targets() {
        for body in [RigidBody::Static, RigidBody::Kinematic, RigidBody::Dynamic] {
            let mut app = App::new();
            app.add_plugins((
                MinimalPlugins,
                TransformPlugin,
                AssetPlugin::default(),
                bevy::mesh::MeshPlugin,
                PhysicsPlugins::new(Update),
            ))
            .insert_resource(Gravity(Vec3::ZERO))
            .insert_resource(TimeUpdateStrategy::ManualDuration(Duration::from_secs_f64(
                1.0 / 60.0,
            )));
            app.finish();
            app.cleanup();

            app.world_mut().spawn((
                body,
                Collider::cuboid(0.02, 2.0, 2.0),
                Transform::IDENTITY,
                Mass(1000.0),
            ));
            let projectile = app
                .world_mut()
                .spawn((
                    Projectile,
                    RigidBody::Dynamic,
                    Collider::sphere(0.0085),
                    Mass(0.0032),
                    Transform::from_xyz(-1.0, 0.0, 0.0),
                    LinearVelocity(Vec3::X * 120.0),
                ))
                .id();

            // Each step travels two meters without CCD, across the entire thin target.
            for _ in 0..4 {
                app.update();
            }
            let position = app.world().get::<Position>(projectile).unwrap();
            assert!(
                position.x < 0.05,
                "projectile passed through {body:?}: {position:?}"
            );
        }
    }
}
