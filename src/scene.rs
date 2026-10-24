//! Scene load order.
//!
//! glTF worlds become ready on whatever frame their asset happens to finish loading, so spawning
//! everything at once lets `setup_vehicle` turn a robot into a dynamic rigid body before the
//! environment has any colliders — and the robot falls through the floor. Loading is therefore one
//! linear task: each `spawn` resolves when that world's instance is ready, and the robots are
//! spawned last.
//!
//! Assets are requested together so file IO overlaps. World mutation remains a linear async task.
//! `LoadedMap` is only constructed after every required collision mesh has a collider; its method
//! is the only startup path that spawns robots.

use avian3d::prelude::*;
use bevy::prelude::*;
use bevy::world_serialization::{InstanceId, WorldAssetRoot, WorldInstance};
use std::collections::HashMap;

use crate::components::{
    ActiveSlapper, Controlled, GameLayer, GroundRoot, Infantry, PreciousCollision, SlapperInfantry,
};
use crate::robomaster::power_rune::construct::setup_power_rune;
use crate::robomaster::prelude::{
    HERO_ROBOT_CONFIG, INFANTRY_THREE_CONFIG, PowerRuneRoot, Team, TechCoreRoot,
};
use crate::robomaster::tech_core::construct::setup_tech_core;
use crate::setup::{
    ScanOutpost, setup_collision, setup_dart_launch, setup_outposts, setup_vehicle,
};
use crate::util::async_world::{AsyncWorld, AsyncWorldTask, drive_async_world};

type SceneResult<T> = Result<T, Box<dyn std::error::Error + Send + Sync>>;

struct RobotAssets {
    infantry: Handle<WorldAsset>,
    hero: Handle<WorldAsset>,
}

struct SceneAssets {
    ground: Handle<WorldAsset>,
    calibration: Handle<WorldAsset>,
    outpost: Handle<WorldAsset>,
    tech_core: Handle<WorldAsset>,
    power_rune: Handle<WorldAsset>,
    robots: RobotAssets,
}

impl SceneAssets {
    fn load(assets: &AssetServer) -> Self {
        let scene = |path: &'static str| assets.load(GltfAssetLabel::Scene(0).from_asset(path));
        Self {
            ground: scene("GROUND.glb"),
            calibration: scene("CALIB.glb"),
            outpost: scene("OUTPOST.glb"),
            tech_core: scene("TECH_CORE.glb"),
            power_rune: scene("POWER.glb"),
            robots: RobotAssets {
                infantry: scene("vehicle.glb"),
                hero: scene("HERO.glb"),
            },
        }
    }
}

/// A capability produced only after map instances and collision meshes are complete.
struct LoadedMap {
    robots: RobotAssets,
}

/// Avian's hierarchy-ready event also fires on construction failure. Check the external
/// constructor's output once here, before granting access to robot spawning.
async fn build_collision(w: &AsyncWorld, root: Entity, map: PreciousCollision) -> SceneResult<()> {
    let roots = w.run(setup_collision, (root, map)).await?;
    let pending = roots.clone();
    w.observe_all::<ColliderConstructorHierarchyReady, _>(move |world| {
        pending
            .into_iter()
            .filter(|entity| world.get::<ColliderConstructorHierarchy>(*entity).is_some())
            .collect()
    })
    .await;
    w.with_world(move |world| {
        for root in roots {
            let mut descendants = Vec::new();
            collect_descendants(world, root, &mut descendants);
            let meshes: Vec<_> = descendants
                .into_iter()
                .filter(|entity| world.get::<Mesh3d>(*entity).is_some())
                .collect();
            if meshes.is_empty() {
                return Err(format!("collision node {root:?} has no meshes"));
            }
            for mesh in meshes {
                if world.get::<Collider>(mesh).is_none() {
                    return Err(format!(
                        "collision mesh {mesh:?} under {root:?} failed to build"
                    ));
                }
            }
        }
        Ok::<_, String>(())
    })
    .await?;
    Ok(())
}

/// The instance id a loaded world spawned under, needed to enumerate its entities by name.
async fn instance_of(w: &AsyncWorld, root: Entity) -> InstanceId {
    w.with_world(move |world| {
        **world
            .get::<WorldInstance>(root)
            .expect("world instance is ready, so its id must exist")
    })
    .await
}

fn collect_descendants(world: &World, entity: Entity, out: &mut Vec<Entity>) {
    let Some(children) = world.get::<Children>(entity) else {
        return;
    };
    for child in children.iter() {
        out.push(child);
        collect_descendants(world, child, out);
    }
}

pub struct ScenePlugin;

impl Plugin for ScenePlugin {
    fn build(&self, app: &mut App) {
        app.insert_resource(AsyncWorldTask::new(load_scene))
            .add_systems(
                PreUpdate,
                drive_async_world.run_if(resource_exists::<AsyncWorldTask>),
            );
    }
}

async fn load_scene(w: AsyncWorld) {
    if let Err(error) = initialize_scene(&w).await {
        error!("scene loading failed: {error}");
    }
}

async fn initialize_scene(w: &AsyncWorld) -> SceneResult<()> {
    let assets = w
        .with_world(|world| SceneAssets::load(world.resource::<AssetServer>()))
        .await;
    let map = load_map(w, assets).await?;
    info!("map loaded; spawning robots");
    map.spawn_robots(w).await?;
    info!("scene loaded");
    Ok(())
}

async fn load_map(w: &AsyncWorld, assets: SceneAssets) -> SceneResult<LoadedMap> {
    let layers = GameLayer::environment_collision_layers();
    let trimesh = || {
        ColliderConstructorHierarchy::new(ColliderConstructor::TrimeshFromMeshWithConfig(
            TrimeshFlags::all(),
        ))
        .with_default_layers(layers)
    };
    let static_trimesh = || {
        (
            trimesh(),
            layers,
            Visibility::Visible,
            Some(RigidBody::Static),
        )
    };

    // Environment first. Every robot below lands on what these build.
    let ground = w
        .spawn(
            WorldAssetRoot(assets.ground),
            (GroundRoot, Friction::new(0.5)),
        )
        .await?;
    w.run(setup_dart_launch, ground).await;
    build_collision(
        w,
        ground,
        PreciousCollision(HashMap::from([(
            "GROUND_DENSE".to_string(),
            static_trimesh(),
        )])),
    )
    .await?;

    w.spawn(
        WorldAssetRoot(assets.calibration.clone()),
        Transform::IDENTITY.with_translation(Vec3::new(1.0, 2.5, 1.0)),
    )
    .await?;

    w.spawn(
        WorldAssetRoot(assets.calibration.clone()),
        Transform::IDENTITY.with_translation(Vec3::new(2.0, 0.5, 2.0)),
    )
    .await?;

    let outpost = w
        .spawn(
            WorldAssetRoot(assets.outpost),
            (RigidBody::Static, ScanOutpost),
        )
        .await?;
    w.run(setup_outposts, outpost).await;

    let tech_core = w
        .spawn(WorldAssetRoot(assets.tech_core), TechCoreRoot)
        .await?;
    w.run(
        setup_tech_core,
        (tech_core, instance_of(w, tech_core).await),
    )
    .await;
    build_collision(
        w,
        tech_core,
        PreciousCollision(HashMap::from([("GROUND".to_string(), static_trimesh())])),
    )
    .await?;

    let power_rune = w
        .spawn(
            WorldAssetRoot(assets.power_rune),
            (
                RigidBody::Static,
                CollisionMargin(0.001),
                Restitution::ZERO,
                PowerRuneRoot,
            ),
        )
        .await?;
    w.run(
        setup_power_rune,
        (power_rune, instance_of(w, power_rune).await),
    )
    .await;
    build_collision(
        w,
        power_rune,
        PreciousCollision(power_rune_collision(layers)),
    )
    .await?;
    Ok(LoadedMap {
        robots: assets.robots,
    })
}

impl LoadedMap {
    async fn spawn_robots(self, w: &AsyncWorld) -> SceneResult<()> {
        // Robots last, so they can never become dynamic over an empty world.
        let player = w
            .spawn(
                WorldAssetRoot(self.robots.infantry.clone()),
                (
                    Transform::from_xyz(0.0, 1.0, 0.0),
                    Infantry::new(Team::Red, INFANTRY_THREE_CONFIG),
                    Controlled,
                ),
            )
            .await?;
        w.run(setup_vehicle, player).await;

        let slapper = w
            .spawn(
                WorldAssetRoot(self.robots.infantry.clone()),
                (
                    Transform::from_xyz(1.0, 1.0, 1.0),
                    Infantry::new(Team::Blue, INFANTRY_THREE_CONFIG),
                    SlapperInfantry,
                ),
            )
            .await?;
        w.run(setup_vehicle, slapper).await;

        let hero = w
            .spawn(
                WorldAssetRoot(self.robots.hero),
                (
                    Transform::from_xyz(2.0, 1.0, 1.0),
                    Infantry::new(Team::Blue, HERO_ROBOT_CONFIG),
                    SlapperInfantry,
                    ActiveSlapper,
                ),
            )
            .await?;
        w.run(setup_vehicle, hero).await;

        Ok(())
    }
}

/// Every rune target gets a voxelized collider so hits register on the spinning arms.
fn power_rune_collision(
    layers: CollisionLayers,
) -> HashMap<
    String,
    (
        ColliderConstructorHierarchy,
        CollisionLayers,
        Visibility,
        Option<RigidBody>,
    ),
> {
    let voxel = |size| {
        ColliderConstructorHierarchy::new(ColliderConstructor::VoxelizedTrimeshFromMesh {
            voxel_size: size,
            fill_mode: FillMode::FloodFill {
                detect_cavities: true,
            },
        })
        .with_default_layers(layers)
    };

    let mut collision = HashMap::from([(
        "BASE".to_string(),
        (
            ColliderConstructorHierarchy::new(ColliderConstructor::TrimeshFromMeshWithConfig(
                TrimeshFlags::all(),
            ))
            .with_default_layers(layers),
            layers,
            Visibility::Visible,
            Some(RigidBody::Static),
        ),
    )]);
    for face in 1..=2 {
        for target in 1..=5 {
            for state in ["ACTIVATED", "ACTIVE", "COMPLETED", "DISABLED"] {
                collision.insert(
                    format!("FACE_{face}_TARGET_{target}_{state}"),
                    (voxel(0.015), layers, Visibility::Visible, None),
                );
            }
        }
    }
    collision
}

#[cfg(test)]
mod tests {
    use super::*;

    #[derive(Resource)]
    struct LoadOutcome(Result<(), String>);

    fn collision_fixture() -> (World, [(Entity, Entity); 2]) {
        let mut world = World::new();
        let root = world.spawn_empty().id();
        let nodes = ["floor", "wall"].map(|name| {
            let node = world.spawn((Name::new(name), ChildOf(root))).id();
            let mesh = world.spawn((Mesh3d::default(), ChildOf(node))).id();
            (node, mesh)
        });
        let map = PreciousCollision(HashMap::from(["floor", "wall"].map(|name| {
            (
                name.to_owned(),
                (
                    ColliderConstructorHierarchy::new(
                        ColliderConstructor::TrimeshFromMeshWithConfig(TrimeshFlags::all()),
                    ),
                    GameLayer::environment_collision_layers(),
                    Visibility::Visible,
                    Some(RigidBody::Static),
                ),
            )
        })));
        world.insert_resource(AsyncWorldTask::new(move |w: AsyncWorld| async move {
            let outcome = build_collision(&w, root, map)
                .await
                .map_err(|error| error.to_string());
            w.with_world(move |world| {
                if outcome.is_ok() {
                    world.spawn(Controlled);
                }
                world.insert_resource(LoadOutcome(outcome));
            })
            .await;
        }));
        (world, nodes)
    }

    fn complete(world: &mut World, node: Entity, mesh: Entity) {
        world.entity_mut(mesh).insert(Collider::sphere(1.0));
        world
            .entity_mut(node)
            .remove::<ColliderConstructorHierarchy>();
        world.trigger(ColliderConstructorHierarchyReady { entity: node });
    }

    #[test]
    fn player_waits_for_every_collision_hierarchy() {
        let (mut world, [(floor, floor_mesh), (wall, wall_mesh)]) = collision_fixture();
        drive_async_world(&mut world);
        assert_eq!(world.query::<&Controlled>().iter(&world).count(), 0);
        complete(&mut world, wall, wall_mesh);
        drive_async_world(&mut world);
        assert_eq!(world.query::<&Controlled>().iter(&world).count(), 0);
        complete(&mut world, floor, floor_mesh);
        drive_async_world(&mut world);
        assert!(world.resource::<LoadOutcome>().0.is_ok());
        assert_eq!(world.query::<&Controlled>().iter(&world).count(), 1);
    }

    #[test]
    fn failed_collision_construction_does_not_spawn_player() {
        let (mut world, [(floor, _), (wall, wall_mesh)]) = collision_fixture();
        drive_async_world(&mut world);
        complete(&mut world, wall, wall_mesh);
        world
            .entity_mut(floor)
            .remove::<ColliderConstructorHierarchy>();
        world.trigger(ColliderConstructorHierarchyReady { entity: floor });
        drive_async_world(&mut world);
        assert!(world.resource::<LoadOutcome>().0.is_err());
        assert_eq!(world.query::<&Controlled>().iter(&world).count(), 0);
        assert!(!world.contains_resource::<AsyncWorldTask>());
    }

    #[test]
    fn missing_collision_node_does_not_spawn_player() {
        let (mut world, [_, (wall, _)]) = collision_fixture();
        world.entity_mut(wall).remove::<Name>();
        drive_async_world(&mut world);
        assert!(
            world
                .resource::<LoadOutcome>()
                .0
                .as_ref()
                .unwrap_err()
                .contains("wall")
        );
        assert_eq!(world.query::<&Controlled>().iter(&world).count(), 0);
        assert!(!world.contains_resource::<AsyncWorldTask>());
    }
}
