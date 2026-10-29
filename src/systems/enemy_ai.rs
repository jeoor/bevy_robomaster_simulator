use std::collections::HashMap;

use avian3d::prelude::*;
use bevy::prelude::*;

use super::{ControllerState, input::update_chassis_rotation};
use crate::components::{
    ActiveSlapper, Controlled, GameLayer, Infantry, InfantryChassis, SlapperInfantry,
};
use crate::config::SimulationConfig;
use crate::robomaster::vehicle::movement::VehicleDynamic;

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
enum Motion {
    #[default]
    Off,
    Patrol,
    Turn,
    VariableSpeed,
    VariableSpin,
}

#[derive(Resource, Default)]
pub struct EnemyAi {
    motion: Motion,
    agents: HashMap<Entity, Agent>,
}

impl EnemyAi {
    pub fn label(&self) -> &'static str {
        match self.motion {
            Motion::Off => "OFF",
            Motion::Patrol => "patrol",
            Motion::Turn => "90deg turns",
            Motion::VariableSpeed => "variable speed",
            Motion::VariableSpin => "variable spin",
        }
    }
}

pub fn cycle_enemy_ai(keyboard: Res<ButtonInput<KeyCode>>, mut ai: ResMut<EnemyAi>) {
    if keyboard.just_pressed(KeyCode::F7) {
        ai.motion = match ai.motion {
            Motion::Off => Motion::Patrol,
            Motion::Patrol => Motion::Turn,
            Motion::Turn => Motion::VariableSpeed,
            Motion::VariableSpeed => Motion::VariableSpin,
            Motion::VariableSpin => Motion::Off,
        };
        // Start at the current location on each change; never teleport a robot.
        ai.agents.clear();
        info!("Enemy AI: {}", ai.label());
    }
}

fn remote_override(controller: &ControllerState) -> bool {
    let input = controller.remote;
    input.movement != Vec2::ZERO
        || input.chassis_yaw != 0.0
        || input.chassis_roll != 0.0
        || input.chassis_pitch != 0.0
}

pub fn remote_chassis_controls_enabled(ai: Res<EnemyAi>, controller: Res<ControllerState>) -> bool {
    ai.motion == Motion::Off || remote_override(&controller)
}

struct Agent {
    anchor: Vec3,
    right: Vec3,
    forward: Vec3,
    waypoint: usize,
    elapsed: f32,
    avoid_until: f32,
    phase: f32,
}

impl Agent {
    fn new(pose: &GlobalTransform, phase: f32) -> Self {
        Self {
            anchor: pose.translation().with_y(0.0),
            right: pose.right().with_y(0.0).normalize_or_zero(),
            forward: pose.forward().with_y(0.0).normalize_or_zero(),
            waypoint: 0,
            elapsed: 0.0,
            avoid_until: 0.0,
            phase,
        }
    }

    fn waypoints(motion: Motion) -> &'static [Vec2] {
        const TURNS: [Vec2; 4] = [
            Vec2::new(1.5, 0.0),
            Vec2::new(1.5, 1.5),
            Vec2::new(-1.5, 1.5),
            Vec2::new(-1.5, 0.0),
        ];
        const PATROL: [Vec2; 2] = [Vec2::new(1.5, 0.0), Vec2::new(-1.5, 0.0)];
        if motion == Motion::Turn {
            &TURNS
        } else {
            &PATROL
        }
    }

    fn velocity(&self, motion: Motion, position: Vec3, dynamic: &VehicleDynamic) -> Vec3 {
        let offset = Self::waypoints(motion)[self.waypoint];
        let remaining =
            self.anchor + self.right * offset.x + self.forward * offset.y - position.with_y(0.0);
        let distance = remaining.length();
        let modulation = if motion == Motion::VariableSpeed {
            0.15 + 0.85 * (0.5 + 0.5 * (2.3 * self.elapsed + self.phase).sin())
        } else {
            1.0
        };
        let speed = (0.5 * dynamic.max_speed * modulation)
            .min((2.0 * dynamic.linear_acceleration * distance).sqrt());
        remaining.normalize_or_zero() * speed
    }
}

pub fn move_enemy_ai(
    time: Res<Time>,
    config: Res<SimulationConfig>,
    controller: Res<ControllerState>,
    mut ai: ResMut<EnemyAi>,
    player: Query<&Infantry, With<Controlled>>,
    mut enemies: Query<
        (
            Entity,
            &Infantry,
            &GlobalTransform,
            &VehicleDynamic,
            Forces,
            &Mass,
            Has<ActiveSlapper>,
        ),
        (With<SlapperInfantry>, Without<Controlled>),
    >,
    mut chassis: Query<(Entity, &mut Transform, &mut InfantryChassis), Without<Infantry>>,
    parents: Query<&ChildOf>,
    children: Query<&Children>,
    spatial: SpatialQuery,
) {
    if ai.motion == Motion::Off {
        return;
    }
    let Ok(player) = player.single() else {
        return;
    };
    let motion = ai.motion;
    let dt = time.delta_secs().min(0.1);
    ai.agents.retain(|entity, _| enemies.contains(*entity));
    for (entity, enemy, pose, dynamic, mut forces, mass, active) in &mut enemies {
        if enemy.team == player.team || (active && remote_override(&controller)) {
            ai.agents.remove(&entity);
            continue;
        }
        let phase = if enemy.config == crate::robomaster::prelude::HERO_ROBOT_CONFIG {
            std::f32::consts::PI
        } else {
            0.0
        };
        let agent = ai
            .agents
            .entry(entity)
            .or_insert_with(|| Agent::new(pose, phase));
        agent.elapsed += dt;
        let position = pose.translation();
        let mut desired = agent.velocity(motion, position, dynamic);
        let filter = SpatialQueryFilter::from_mask([
            GameLayer::Environment,
            GameLayer::VehicleSelf,
            GameLayer::VehicleOther,
        ])
        .with_excluded_entities(std::iter::once(entity).chain(children.iter_descendants(entity)));
        let velocity = forces.linear_velocity().with_y(0.0);
        let blocked = Dir3::new(desired).is_ok_and(|direction| {
            let side = direction.cross(Vec3::Y) * 0.3;
            [-side, Vec3::ZERO, side].into_iter().any(|offset| {
                spatial
                    .cast_ray(
                        position + Vec3::Y * 0.35 + offset,
                        direction,
                        0.5 + velocity.length_squared()
                            / (2.0 * dynamic.linear_acceleration.max(0.01)),
                        true,
                        &filter,
                    )
                    .is_some()
            })
        });
        let offset = Agent::waypoints(motion)[agent.waypoint];
        let destination = agent.anchor + agent.right * offset.x + agent.forward * offset.y;
        if position.with_y(0.0).distance(destination) < 0.35
            || (blocked && agent.elapsed >= agent.avoid_until)
        {
            agent.waypoint = (agent.waypoint + 1) % Agent::waypoints(motion).len();
            agent.avoid_until = agent.elapsed + 0.5;
            desired = agent.velocity(motion, position, dynamic);
        }
        // Track a bounded desired velocity through impulses; retain gravity and contacts.
        if blocked {
            desired = Vec3::ZERO;
        }
        let delta = (desired - velocity).clamp_length_max(dynamic.linear_acceleration * dt);
        forces.apply_linear_impulse(delta * mass.0);
    }
    for (entity, mut transform, mut chassis) in &mut chassis {
        if let Some(agent) = parents
            .iter_ancestors(entity)
            .find_map(|root| ai.agents.get(&root))
        {
            let yaw = if motion == Motion::VariableSpin {
                0.55 + 0.4 * (2.3 * agent.elapsed + agent.phase).sin()
            } else {
                0.0
            };
            update_chassis_rotation(
                &mut transform,
                &mut chassis,
                yaw,
                0.0,
                0.0,
                config.vehicle.rotation_speed,
                config.vehicle.yaw_acceleration,
                config.vehicle.tilt_rotation_speed,
                dt,
            );
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::robomaster::prelude::{INFANTRY_THREE_CONFIG, Team};
    use bevy::time::TimeUpdateStrategy;
    use std::time::Duration;

    fn fixture() -> (App, Entity, Entity, Entity) {
        let mut app = App::new();
        app.add_plugins((
            MinimalPlugins,
            bevy::asset::AssetPlugin::default(),
            TransformPlugin,
            PhysicsPlugins::default(),
        ));
        app.init_asset::<Mesh>();
        app.insert_resource(Gravity(Vec3::ZERO));
        app.insert_resource(TimeUpdateStrategy::ManualDuration(Duration::from_secs_f32(
            1.0 / 60.0,
        )));
        app.insert_resource(
            toml::from_str::<SimulationConfig>(include_str!("../../config.toml")).unwrap(),
        );
        app.init_resource::<ControllerState>()
            .init_resource::<EnemyAi>();
        app.add_systems(Update, move_enemy_ai);
        let player = app
            .world_mut()
            .spawn((
                Controlled,
                Infantry::new(Team::Red, INFANTRY_THREE_CONFIG),
                Transform::from_xyz(0.0, 0.0, 8.0),
                RigidBody::Dynamic,
                Mass(15.0),
            ))
            .id();
        let enemy = app
            .world_mut()
            .spawn((
                SlapperInfantry,
                ActiveSlapper,
                Infantry::new(Team::Blue, INFANTRY_THREE_CONFIG),
                Transform::IDENTITY,
                RigidBody::Dynamic,
                Mass(15.0),
                VehicleDynamic::new(6.0, 6.0, 2.5),
                Collider::cuboid(0.6, 0.4, 0.6),
                GameLayer::vehicle_body_collision_layers(false),
            ))
            .id();
        let chassis = app
            .world_mut()
            .spawn((
                Transform::IDENTITY,
                InfantryChassis::default(),
                ChildOf(enemy),
            ))
            .id();
        app.finish();
        app.cleanup();
        (app, player, enemy, chassis)
    }

    #[test]
    fn ai_moves_enemy_with_physics_and_yields_to_manual_control() {
        let (mut app, player, enemy, _) = fixture();
        for _ in 0..60 {
            app.update();
        }
        assert_eq!(app.world().get::<Position>(enemy).unwrap().x, 0.0);
        app.world_mut().resource_mut::<EnemyAi>().motion = Motion::Patrol;
        let mut min_x = f32::MAX;
        let mut max_x = f32::MIN;
        for _ in 0..600 {
            app.update();
            let p = app.world().get::<Position>(enemy).unwrap().0;
            min_x = min_x.min(p.x);
            max_x = max_x.max(p.x);
            assert!(p.x.abs() < 2.0, "patrol escaped its local route");
            assert!(app.world().get::<LinearVelocity>(enemy).unwrap().length() <= 3.01);
        }
        assert!(
            min_x < -1.0 && max_x > 1.0,
            "enemy failed to move and reverse"
        );
        assert_eq!(app.world().get::<Position>(player).unwrap().z, 8.0);
        app.world_mut()
            .resource_mut::<ControllerState>()
            .remote
            .movement = Vec2::Y;
        app.update();
        assert!(
            !app.world()
                .resource::<EnemyAi>()
                .agents
                .contains_key(&enemy)
        );
    }

    #[test]
    fn ai_exercises_turns_speed_changes_and_chassis_spin() {
        for motion in [Motion::Turn, Motion::VariableSpeed, Motion::VariableSpin] {
            let (mut app, _, enemy, chassis) = fixture();
            app.world_mut().resource_mut::<EnemyAi>().motion = motion;
            let mut min_speed = 10.0_f32;
            let mut max_speed = 0.0_f32;
            let mut max_z = 0.0_f32;
            let mut min_spin = 10.0_f32;
            let mut max_spin = 0.0_f32;
            for i in 0..480 {
                app.update();
                let speed = app.world().get::<LinearVelocity>(enemy).unwrap().length();
                assert!(speed <= 3.01);
                if i > 60 {
                    min_speed = min_speed.min(speed);
                    max_speed = max_speed.max(speed);
                    let spin = app
                        .world()
                        .get::<InfantryChassis>(chassis)
                        .unwrap()
                        .yaw_velocity;
                    min_spin = min_spin.min(spin);
                    max_spin = max_spin.max(spin);
                }
                max_z = max_z.max(app.world().get::<Position>(enemy).unwrap().z.abs());
            }
            if motion == Motion::Turn {
                assert!(max_z > 1.0, "route never turned");
            }
            if motion == Motion::VariableSpeed {
                assert!(max_speed - min_speed > 1.0);
            }
            if motion == Motion::VariableSpin {
                assert!(max_spin - min_spin > 3.0);
            }
        }
    }

    #[test]
    fn ai_brakes_for_obstacles_and_does_not_move_friendly_robots() {
        let (mut app, _, enemy, _) = fixture();
        let ally = app
            .world_mut()
            .spawn((
                SlapperInfantry,
                Infantry::new(Team::Red, INFANTRY_THREE_CONFIG),
                Transform::from_xyz(0.0, 0.0, -8.0),
                RigidBody::Dynamic,
                Mass(15.0),
                VehicleDynamic::default(),
            ))
            .id();
        app.world_mut().spawn((
            RigidBody::Static,
            Collider::cuboid(0.2, 2.0, 3.0),
            Transform::from_xyz(1.0, 0.35, 0.0),
            GameLayer::environment_collision_layers(),
        ));
        app.world_mut().resource_mut::<EnemyAi>().motion = Motion::Patrol;
        let mut minimum = 0.0_f32;
        for _ in 0..480 {
            app.update();
            let x = app.world().get::<Position>(enemy).unwrap().x;
            assert!(x < 0.6, "robot failed to brake before the wall");
            minimum = minimum.min(x);
        }
        assert!(minimum < -1.0, "obstacle response never moved away");
        assert_eq!(
            app.world().get::<Position>(ally).unwrap().0,
            Vec3::new(0.0, 0.0, -8.0)
        );
    }
}
