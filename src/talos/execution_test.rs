use super::*;
use crate::components::{GameLayer, Infantry, ProjectileSetting};
use crate::robomaster::prelude::{
    Armor, ArmorLabel, ArmorPlugins, ArmorSpec, INFANTRY_THREE_CONFIG, Projectile, SmallArmorLabel,
    Team,
};
use crate::statistic::ProjectileStatistics;
use crate::systems::{gimbal_pid_controls, tick_projectile_cooldown};
use avian3d::prelude::*;
use bevy::asset::AssetPlugin;
use bevy::input::gamepad::GamepadRumbleRequest;
use bevy::time::TimeUpdateStrategy;
use serde_json::{Value, json};
use std::io::{BufRead, BufReader, Write};
use std::process::{Command, Stdio};
use std::time::Duration;

#[derive(Resource, Default)]
struct Contacts(Vec<(Entity, ArmorLabel)>);

fn observe_contact(
    event: On<CollisionStart>,
    projectiles: Query<(), With<Projectile>>,
    armors: Query<&Armor>,
    mut contacts: ResMut<Contacts>,
) {
    for (body, collider) in [
        (event.body1, event.collider2),
        (event.body2, event.collider1),
    ] {
        if let Some(body) = body.filter(|body| projectiles.contains(*body)) {
            if let Ok(armor) = armors.get(collider) {
                if !contacts.0.iter().any(|(shot, _)| *shot == body) {
                    contacts.0.push((body, armor.label));
                }
            }
        }
    }
}

fn bevy_position(flu: Vec3) -> Vec3 {
    M_ALIGN_MAT3.transpose() * flu
}

fn scene(t: f32, mode: i32) -> (Vec3, f32, Vec3, Vec3) {
    let mut center = Vec3::new(6.0, 0.0, 0.2);
    let mut origin = Vec3::new(0.0, 0.6, 0.0);
    let mut velocity = Vec3::ZERO;
    let mut yaw = 0.0;
    match mode {
        1 => center.y = -4.0 + 3.0 * t,
        2 => {
            center.y = -3.0 + 3.0 * t.min(2.0);
            center.x += 3.0 * (t - 2.0).max(0.0);
        }
        3 => {
            center.y = 0.8 * t;
            origin.x = -1.2 * (1.0 - (3.0 * t).cos());
            velocity.x = -3.6 * (3.0 * t).sin();
        }
        4 => yaw = 3.36 * t,
        5 => center.x = 12.0,
        6 => yaw = 3.36 * t + 0.8 * (3.0 * t).sin(),
        7 => yaw = 12.0 * t,
        8 => yaw = -12.0 * t,
        _ => {}
    }
    (center, yaw, origin, velocity)
}

fn plate_pose(t: f32, mode: i32, id: usize) -> Transform {
    let (center, yaw, _, _) = scene(t, mode);
    let yaw = yaw + id as f32 * std::f32::consts::FRAC_PI_2;
    let (sin, cos) = yaw.sin_cos();
    let (tilt_sin, tilt_cos) = (std::f32::consts::PI / 12.0).sin_cos();
    let width = bevy_position(Vec3::new(-sin, cos, 0.0));
    let height = bevy_position(Vec3::new(tilt_sin * cos, tilt_sin * sin, tilt_cos));
    Transform::from_translation(bevy_position(center - Vec3::new(0.2 * cos, 0.2 * sin, 0.0)))
        .with_rotation(Quat::from_mat3(&Mat3::from_cols(
            width,
            height,
            width.cross(height),
        )))
}

fn marker(name: &str) -> Transform {
    let data = std::fs::read(concat!(env!("CARGO_MANIFEST_DIR"), "/assets/vehicle.glb")).unwrap();
    let length = u32::from_le_bytes(data[12..16].try_into().unwrap()) as usize;
    let asset: Value = serde_json::from_slice(&data[20..20 + length]).unwrap();
    let node = asset["nodes"]
        .as_array()
        .unwrap()
        .iter()
        .find(|n| n["name"] == name)
        .unwrap();
    let position: [f32; 3] = serde_json::from_value(node["translation"].clone()).unwrap();
    let rotation: [f32; 4] = serde_json::from_value(node["rotation"].clone()).unwrap();
    Transform::from_translation(Vec3::from_array(position))
        .with_rotation(Quat::from_array(rotation).normalize())
}

#[test]
#[ignore = "runs the C++ solver worker and native physics; run ./sim.sh test"]
fn fire_control_executes_solver_commands_in_native_physics() {
    let selected_case = std::env::var("SP_SIM_EXEC_CASE").ok().map(|value| {
        let (fps, mode) = value
            .split_once(':')
            .expect("SP_SIM_EXEC_CASE must be FPS:MODE");
        let case = (fps.parse::<i32>().unwrap(), mode.parse::<i32>().unwrap());
        assert!([18, 30, 60].contains(&case.0) && (0..=8).contains(&case.1));
        case
    });
    let worker =
        std::env::var("SP_SIM_WORKER").expect("SP_SIM_WORKER must point to sp_sim_motion_tests");
    let mut child = Command::new(worker)
        .arg("--worker")
        .current_dir(std::env::var("SP_SIM_ROOT").unwrap())
        .stdin(Stdio::piped())
        .stdout(Stdio::piped())
        .spawn()
        .unwrap();
    let mut input = child.stdin.take().unwrap();
    let mut output = BufReader::new(child.stdout.take().unwrap());
    let config: SimulationConfig = toml::from_str(include_str!("../../config.toml")).unwrap();
    let muzzle_marker = marker("SHOT_DIRECTION");
    let camera_marker = marker("CAM_DIRECTION");
    for fps in [18, 30, 60] {
        for mode in [0, 1, 2, 3, 4, 5, 6, 7, 8] {
            if selected_case.is_some_and(|selected| selected != (fps, mode)) {
                continue;
            }
            let dt = 1.0 / fps as f32;
            let mut app = App::new();
            app.add_plugins((
                MinimalPlugins,
                AssetPlugin::default(),
                TransformPlugin,
                PhysicsPlugins::default(),
            ));
            app.init_asset::<Mesh>().init_asset::<StandardMaterial>();
            app.add_plugins(ArmorPlugins);
            app.insert_resource(config.clone());
            app.insert_resource(Gravity(Vec3::NEG_Y * 9.81));
            app.insert_resource(SubstepCount(config.physics.substep_count));
            app.insert_resource(Time::<Fixed>::from_hz(config.physics.fixed_hz));
            app.insert_resource(TimeUpdateStrategy::ManualDuration(Duration::from_secs_f32(
                dt,
            )));
            app.insert_resource(ProjectileCooldown(Timer::from_seconds(
                config.projectile.cooldown,
                TimerMode::Once,
            )));
            app.insert_resource(ProjectileSetting(default(), default()));
            app.init_resource::<ProjectileStatistics>()
                .init_resource::<Contacts>();
            app.add_message::<GamepadRumbleRequest>();
            app.add_systems(Update, (gimbal_pid_controls, tick_projectile_cooldown));
            app.add_observer(observe_contact);
            let chassis = app
                .world_mut()
                .spawn((
                    Controlled,
                    Infantry::new(Team::Red, INFANTRY_THREE_CONFIG),
                    Transform::from_xyz(0.0, 0.6, 0.0),
                    LinearVelocity::ZERO,
                    AngularVelocity::ZERO,
                ))
                .id();
            if mode == 0 || mode == 5 {
                // A nonzero ancestor exposes launches that mix a local chassis
                // translation with a world-space muzzle rotation.
                let parent = app
                    .world_mut()
                    .spawn(Transform::from_xyz(2.0, 0.2, 1.0))
                    .id();
                app.world_mut().entity_mut(chassis).insert(ChildOf(parent));
            }
            let level = Quat::from_rotation_x(-std::f32::consts::FRAC_PI_2)
                * muzzle_marker.rotation.inverse();
            let gimbal = app
                .world_mut()
                .spawn((
                    Controlled,
                    InfantryGimbal::default(),
                    Transform::from_rotation(level),
                    ChildOf(chassis),
                ))
                .id();
            let muzzle = app
                .world_mut()
                .spawn((
                    Controlled,
                    InfantryLaunchOffset,
                    muzzle_marker,
                    ChildOf(gimbal),
                ))
                .id();
            let mut plates = Vec::new();
            for id in 0..4 {
                let pose = plate_pose(0.0, mode, id);
                plates.push(
                    app.world_mut()
                        .spawn((
                            RigidBody::Kinematic,
                            pose,
                            Collider::cuboid(0.135, 0.056, 0.005),
                            GameLayer::vehicle_armor_collision_layers(false),
                            Armor {
                                name: "target".into(),
                                team: Team::Blue,
                                spec: ArmorSpec::Small(SmallArmorLabel::Three),
                                label: ArmorLabel::Three,
                            },
                        ))
                        .id(),
                );
            }
            let mut other_pose = plate_pose(0.0, 0, 0);
            other_pose.translation = bevy_position(Vec3::new(7.0, -5.0, 0.2));
            let other = app
                .world_mut()
                .spawn((
                    RigidBody::Kinematic,
                    other_pose,
                    Collider::cuboid(0.135, 0.056, 0.005),
                    GameLayer::vehicle_armor_collision_layers(false),
                    Armor {
                        name: "distractor".into(),
                        team: Team::Blue,
                        spec: ArmorSpec::Small(SmallArmorLabel::Four),
                        label: ArmorLabel::Four,
                    },
                ))
                .id();
            app.finish();
            app.cleanup();
            app.update();
            let mut state = CommandState::default();
            let mut pending = std::collections::VecDeque::new();
            let mut wrong_identity = 0;
            for frame in 0..fps * 7 {
                let t = frame as f32 * dt;
                let now_ns = 1_000_000_000 + (t as f64 * 1e9) as u64;
                // Match receive_subscription: consume packets before this frame's PID.
                while pending.front().is_some_and(|(due, _)| *due <= now_ns) {
                    let (_, cmd) = pending.pop_front().unwrap();
                    state.update(cmd);
                }
                if let Some(cmd) = state.command {
                    if cmd.distance_m < 0.0 || !valid_command(&cmd, now_ns) || t > 5.1 {
                        app.world_mut()
                            .entity_mut(gimbal)
                            .remove::<GimbalAimTracker>();
                    } else {
                        let target = state.target();
                        if let Some(mut tracker) =
                            app.world_mut().get_mut::<GimbalAimTracker>(gimbal)
                        {
                            tracker.retarget(target);
                        } else {
                            app.world_mut()
                                .entity_mut(gimbal)
                                .insert(GimbalAimTracker::new(target));
                        }
                    }
                }
                let (_, _, origin, velocity) = scene(t, mode);
                app.world_mut().entity_mut(chassis).insert((
                    Transform::from_translation(origin),
                    LinearVelocity(velocity),
                ));
                for (id, entity) in plates.iter().enumerate() {
                    let before = plate_pose((t - dt).max(0.0), mode, id);
                    let current = plate_pose(t, mode, id);
                    let delta = current.rotation * before.rotation.inverse();
                    let (axis, angle) = delta.to_axis_angle();
                    app.world_mut().entity_mut(*entity).insert((
                        before,
                        Position(before.translation),
                        Rotation(before.rotation),
                        LinearVelocity((current.translation - before.translation) / dt),
                        AngularVelocity(axis * angle / dt),
                    ));
                }
                other_pose.translation =
                    bevy_position(Vec3::new(7.0, -5.0 + (t - dt).max(0.0), 0.2));
                app.world_mut().entity_mut(other).insert((
                    other_pose,
                    Position(other_pose.translation),
                    Rotation(other_pose.rotation),
                    LinearVelocity(bevy_position(Vec3::Y)),
                ));
                app.update();
                if t <= 5.0 {
                    let world = app.world();
                    let gimbal_world = world.get::<GlobalTransform>(gimbal).unwrap();
                    let muzzle_world = world.get::<GlobalTransform>(muzzle).unwrap();
                    let camera_rotation = muzzle_world.rotation()
                        * Quat::from_rotation_x(std::f32::consts::FRAC_PI_2);
                    let camera_position = gimbal_world.translation()
                        + gimbal_world.rotation() * camera_marker.translation;
                    let mut detections = Vec::new();
                    for entity in plates.iter().copied().chain(std::iter::once(other)) {
                        let p = world.get::<Position>(entity).unwrap().0;
                        let q = world.get::<Rotation>(entity).unwrap().0;
                        let normal = q * Vec3::Z;
                        if normal.dot(p - camera_position) <= 0.0 {
                            continue;
                        }
                        let mut points = Vec::new();
                        for corner in [
                            Vec3::new(0.0675, 0.028, 0.0),
                            Vec3::new(-0.0675, 0.028, 0.0),
                            Vec3::new(-0.0675, -0.028, 0.0),
                            Vec3::new(0.0675, -0.028, 0.0),
                        ] {
                            let c = camera_rotation.inverse() * (p + q * corner - camera_position);
                            if c.z >= 0.0 {
                                continue;
                            }
                            points
                                .push([720.0 + 1303.68 * c.x / -c.z, 540.0 - 1303.68 * c.y / -c.z]);
                        }
                        if points.len() == 4
                            && points
                                .iter()
                                .all(|p| p[0] > 0.0 && p[0] < 1440.0 && p[1] > 0.0 && p[1] < 1080.0)
                        {
                            let name = if entity == other { 3 } else { 2 };
                            detections.push(json!({"name": name, "points": points}));
                        }
                    }
                    let q = to_ros_quat(camera_rotation);
                    let pose = |p: Vec3, q: Quat| json!({"position": to_ros_translation(p).to_array(), "quaternion": [q.w, q.x, q.y, q.z]});
                    let request = json!({"time": t, "reset": frame == 0, "armors": detections,
                        "velocity": to_ros_translation(velocity).to_array(),
                        "poses": [pose(Vec3::ZERO, q), pose(gimbal_world.translation(), Quat::IDENTITY),
                            pose(muzzle_marker.translation, Quat::IDENTITY), pose(camera_marker.translation, Quat::IDENTITY)]});
                    writeln!(input, "{request}").unwrap();
                    input.flush().unwrap();
                    let mut line = String::new();
                    output.read_line(&mut line).unwrap();
                    let result: Value =
                        serde_json::from_str(&line).expect("C++ solver returned invalid JSON");
                    let label = result["label"].as_i64().unwrap();
                    wrong_identity += usize::from(label >= 0 && label != 2);
                    let cmd = GimbalCmd {
                        timestamp_ns: result["timestamp_ns"].as_u64().unwrap(),
                        yaw_deg: result["yaw_deg"].as_f64().unwrap() as f32,
                        pitch_deg: result["pitch_deg"].as_f64().unwrap() as f32,
                        distance_m: result["distance_m"].as_f64().unwrap() as f32,
                        fire_advice: result["fire"].as_u64().unwrap() as u8,
                        _pad: serde_json::from_value(result["pad"].clone()).unwrap(),
                    };
                    pending.push_back((cmd.timestamp_ns + 10_000_000, cmd));
                }
                if let Some(cmd) = state.command {
                    if cmd.distance_m < 0.0 || !valid_command(&cmd, now_ns) || t > 5.1 {
                        app.world_mut()
                            .entity_mut(gimbal)
                            .remove::<GimbalAimTracker>();
                    } else {
                        let rotation = app
                            .world()
                            .get::<GlobalTransform>(muzzle)
                            .unwrap()
                            .rotation();
                        let aligned =
                            fire_aligned(&cmd, state.target(), rotation, now_ns, state.on_target);
                        state.on_target = aligned && cmd.fire_advice != 0;
                        if aligned
                            && state.fire_ready(now_ns)
                            && app.world().resource::<ProjectileCooldown>().is_finished()
                        {
                            app.world_mut().run_system_once(projectile_launch).unwrap();
                            state.last_fire_ns = now_ns;
                        }
                    }
                }
            }
            let stats = app.world().resource::<ProjectileStatistics>();
            let contacts = &app.world().resource::<Contacts>().0;
            let hits = contacts
                .iter()
                .filter(|(_, label)| *label == ArmorLabel::Three)
                .count();
            println!(
                "native execution fps={fps} mode={mode}: shots={} armor_hits={} native_count={} wrong_track={wrong_identity}",
                stats.launch_count, hits, stats.accurate_count
            );
            assert_eq!(wrong_identity, 0, "tracker changed identity");
            assert!(
                stats.launch_count >= 3,
                "fire gate starved: fps={fps} mode={mode}"
            );
            assert!(
                hits as f32 / stats.launch_count as f32
                    >= if mode == 0 || mode == 5 { 0.9 } else { 0.5 },
                "native armor hit regression: fps={fps} mode={mode}, {hits}/{}",
                stats.launch_count
            );
            assert!(
                stats.accurate_count as usize <= hits,
                "hit counter admitted a non-armor contact"
            );
        }
    }
    drop(input);
    assert!(child.wait().unwrap().success());
}
