use crate::capture::driver::{CaptureConfig, CapturedFrameKind};
use crate::components::{
    Controlled, InfantryChassis, InfantryGimbal, InfantryLaunchOffset, ProjectileCooldown,
    SubscribeAutoAim,
};
use crate::config::SimulationConfig;
use crate::systems::{
    GameplaySystems, GimbalAimTarget, GimbalAimTracker, gimbal_controls, gimbal_pid_controls,
    projectile_launch,
};
use crate::talos::capture::{
    TalosCaptureContext, TalosCapturePlugin, TalosFrameStamp, advance_talos_frame_stamp,
    publish_talos_runtime_state_system,
};
use bevy::ecs::system::RunSystemOnce;
use bevy::prelude::*;
use bevy::render::render_resource::TextureFormat;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};
use talos_ipc::*;

#[derive(Resource)]
pub struct ShmSubscriberRes(pub Arc<Mutex<ShmSubscriber>>);

#[cfg(test)]
#[path = "execution_test.rs"]
mod execution_test;

#[derive(Resource, Deref, DerefMut)]
pub struct TalosEnabled(pub AtomicBool);

pub struct TalosPluginConfig {
    pub width: u32,
    pub height: u32,
    pub fov_y: f32,
    pub texture_format: TextureFormat,
}

impl Default for TalosPluginConfig {
    fn default() -> Self {
        let config = SimulationConfig::default();
        Self {
            width: config.capture.color.width,
            height: config.capture.color.height,
            fov_y: config.camera.fov.to_radians(),
            texture_format: TextureFormat::Rgba8UnormSrgb,
        }
    }
}

#[derive(Default)]
pub struct TalosPlugin {
    pub config: TalosPluginConfig,
}

impl Plugin for TalosPlugin {
    fn build(&self, app: &mut App) {
        let publisher = match ShmPublisher::create() {
            Ok(p) => {
                info!("talos shm created");
                p
            }
            Err(e) => {
                error!("cannot create talos shm: {}", e);
                return;
            }
        };

        let publisher = Arc::new(Mutex::new(publisher));

        let capture_config = CaptureConfig {
            width: self.config.width,
            height: self.config.height,
            texture_format: self.config.texture_format,
            frame_kind: CapturedFrameKind::Rgb8,
        };

        let capture_context = TalosCaptureContext {
            publisher: publisher.clone(),
            fov_y: self.config.fov_y,
        };

        app.init_resource::<TalosFrameStamp>()
            .init_resource::<CommandState>();

        app.add_plugins(TalosCapturePlugin {
            config: capture_config,
            context: capture_context,
        });
        app.add_plugins(super::overlay::AimOverlayPlugin);

        match ShmSubscriber::connect() {
            Ok(subscriber) => {
                info!("connected to talos-cpp");
                app.insert_resource(ShmSubscriberRes(Arc::new(Mutex::new(subscriber))));
            }
            Err(_) => {
                info!("could not connect to talos-cpp");
            }
        }

        app.insert_resource(TalosEnabled(AtomicBool::new(true)));
        app.add_systems(Last, (advance_talos_frame_stamp, heartbeat_system));
        app.add_systems(
            Update,
            receive_subscription
                .in_set(GameplaySystems::Input)
                .after(gimbal_controls)
                .before(gimbal_pid_controls),
        );
        app.add_systems(
            Last,
            publish_talos_runtime_state_system.after(advance_talos_frame_stamp),
        );
        app.add_systems(
            Last,
            crate::talos::ground_truth::publish_ground_truth_system
                .after(publish_talos_runtime_state_system),
        );
        app.add_systems(
            Last,
            process_subscription
                .run_if(|enabled: Res<SubscribeAutoAim>| enabled.load(Ordering::Acquire)),
        );
    }
}

#[derive(Resource, Default)]
struct CommandState {
    command: Option<GimbalCmd>,
    rate: Vec2,
    stable: bool,
    on_target: bool,
    last_fire_ns: u64,
    fresh: bool,
}

fn command_target(cmd: &GimbalCmd) -> GimbalAimTarget {
    GimbalAimTarget::from_solver_degrees(cmd.yaw_deg, -cmd.pitch_deg)
}

fn angle_delta(a: f32, b: f32) -> f32 {
    (a - b + std::f32::consts::PI).rem_euclid(std::f32::consts::TAU) - std::f32::consts::PI
}

impl CommandState {
    fn update(&mut self, cmd: GimbalCmd) {
        self.stable = false;
        if let Some(previous) = self.command {
            let dt = cmd.timestamp_ns.saturating_sub(previous.timestamp_ns) as f32 * 1e-9;
            if dt > 0.0 && dt <= 0.1 && cmd._pad[8] == 0xa1 {
                let (target, before) = (command_target(&cmd), command_target(&previous));
                let delta = Vec2::new(
                    angle_delta(target.yaw, before.yaw),
                    angle_delta(target.pitch, before.pitch),
                );
                let window = Vec2::new(
                    f32::from_ne_bytes(cmd._pad[0..4].try_into().unwrap()),
                    f32::from_ne_bytes(cmd._pad[4..8].try_into().unwrap()),
                );
                // A switch may move the setpoint, but must not be extrapolated as velocity.
                self.stable = (delta - self.rate * dt).abs().cmple(window * 4.0).all();
                self.rate = delta / dt;
            } else {
                self.rate = Vec2::ZERO;
            }
        }
        self.command = Some(cmd);
        if !self.stable {
            self.on_target = false;
        }
    }

    fn target(&self) -> GimbalAimTarget {
        // The solver already predicts the intercept. Preserve its angles, as the
        // original receiver did; differencing commands amplifies detector noise.
        command_target(self.command.as_ref().unwrap())
    }

    fn fire_ready(&self, now_ns: u64) -> bool {
        let cmd = self.command.as_ref().unwrap();
        self.stable
            && cmd.fire_advice == 2
            && cmd._pad[8] == 0xa1
            && cmd._pad[10] > 0
            && now_ns.saturating_sub(self.last_fire_ns) >= u64::from(cmd._pad[10]) * 10_000_000
    }
}

fn receive_subscription(
    context: Option<Res<ShmSubscriberRes>>,
    enabled: Res<SubscribeAutoAim>,
    mut commands: Commands,
    mut state: ResMut<CommandState>,
    gimbal: Single<
        (Entity, Option<&mut GimbalAimTracker>),
        (
            With<Controlled>,
            With<InfantryGimbal>,
            Without<InfantryChassis>,
            Without<InfantryLaunchOffset>,
        ),
    >,
) {
    state.fresh = false;
    if !enabled.load(Ordering::Acquire) {
        *state = CommandState {
            last_fire_ns: state.last_fire_ns,
            ..default()
        };
        return;
    }
    let Some(ctx) = context else {
        return;
    };
    let (gimbal_entity, tracker) = gimbal.into_inner();

    let now_ns = std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .map(|d| d.as_nanos() as u64)
        .unwrap_or(0);
    let received = recv_gimbal_cmd(&ctx);
    let fresh = received.is_some();
    state.fresh = fresh;
    if let Some(cmd) = received {
        if !valid_command(&cmd, now_ns) {
            state.command = None;
            commands.entity(gimbal_entity).remove::<GimbalAimTracker>();
            return;
        }
        state.update(cmd);
    }
    let Some(cmd) = state.command else {
        return;
    };
    if !valid_command(&cmd, now_ns) {
        state.command = None;
        commands.entity(gimbal_entity).remove::<GimbalAimTracker>();
        return;
    }
    if !fresh && tracker.is_none() {
        state.command = None;
        return;
    }
    if !fresh && cmd._pad[8] != 0xa1 {
        return;
    }
    // No solution from the solver: drop the target so the PID loop stops driving.
    if cmd.distance_m == -1.0 {
        state.command = None;
        commands.entity(gimbal_entity).remove::<GimbalAimTracker>();
        return;
    }
    // Consume the solver's setpoint before this frame's PID.
    let control_target = state.target();
    match tracker {
        Some(mut tracker) => tracker.retarget(control_target),
        None => {
            commands
                .entity(gimbal_entity)
                .insert(GimbalAimTracker::new(control_target));
        }
    }
}

fn process_subscription(
    mut commands: Commands,
    mut state: ResMut<CommandState>,
    cooldown: Res<ProjectileCooldown>,
    muzzle: Single<&GlobalTransform, (With<Controlled>, With<InfantryLaunchOffset>)>,
) {
    let fresh = std::mem::take(&mut state.fresh);
    let Some(cmd) = state.command else {
        return;
    };
    let now_ns = std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .map(|d| d.as_nanos() as u64)
        .unwrap_or(0);
    if !valid_command(&cmd, now_ns) || cmd.distance_m < 0.0 {
        state.on_target = false;
        return;
    }
    // Global muzzle pose includes this frame's PID and parent transforms in Last.
    let aligned = fire_aligned(
        &cmd,
        state.target(),
        muzzle.rotation(),
        now_ns,
        state.on_target,
    );
    state.on_target = aligned && cmd.fire_advice != 0;
    if ((fresh && cmd.fire_advice == 1) || state.fire_ready(now_ns))
        && cooldown.is_finished()
        && aligned
    {
        state.last_fire_ns = now_ns;
        commands.queue(|w: &mut World| {
            w.run_system_once(projectile_launch).unwrap();
        });
    }
}

fn fire_aligned(
    cmd: &GimbalCmd,
    target: GimbalAimTarget,
    muzzle: Quat,
    now_ns: u64,
    latched: bool,
) -> bool {
    // Optional v2 extension in the reserved bytes. Legacy clients retain their
    // one-shot advice semantics; vision can request an execution-time geometry check.
    if cmd._pad[8] != 0xa1 {
        return true;
    }
    let yaw = f32::from_ne_bytes(cmd._pad[0..4].try_into().unwrap());
    let pitch = f32::from_ne_bytes(cmd._pad[4..8].try_into().unwrap());
    let margin = f32::from(cmd._pad[9]) / 100.0;
    yaw.is_finite()
        && pitch.is_finite()
        && yaw > 0.0
        && pitch > 0.0
        && yaw < std::f32::consts::FRAC_PI_2
        && pitch < std::f32::consts::FRAC_PI_2
        && margin > 0.0
        && margin <= 1.0
        && now_ns.saturating_sub(cmd.timestamp_ns) <= 100_000_000
        && target.in_fire_window(
            muzzle,
            Vec2::new(yaw, pitch) * if latched { 1.0 } else { margin },
        )
}

fn valid_command(cmd: &GimbalCmd, now_ns: u64) -> bool {
    cmd.timestamp_ns > 0
        && now_ns.saturating_sub(cmd.timestamp_ns) <= 500_000_000
        && cmd.timestamp_ns.saturating_sub(now_ns) <= 50_000_000
        && cmd.yaw_deg.is_finite()
        && cmd.pitch_deg.is_finite()
        && cmd.distance_m.is_finite()
        && (cmd.fire_advice <= 1
            || (cmd.fire_advice == 2 && cmd._pad[8] == 0xa1 && cmd._pad[10] > 0))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn stale_future_and_nonfinite_commands_are_rejected() {
        let mut cmd = GimbalCmd {
            timestamp_ns: 1_000_000_000,
            ..default()
        };
        assert!(valid_command(&cmd, 1_010_000_000));
        assert!(!valid_command(&cmd, 1_600_000_000));
        assert!(!valid_command(&cmd, 900_000_000));
        cmd.yaw_deg = f32::NAN;
        assert!(!valid_command(&cmd, 1_010_000_000));
    }

    #[test]
    fn fire_extension_rechecks_geometry_at_execution_time() {
        let mut cmd = GimbalCmd {
            timestamp_ns: 1_000_000_000,
            ..default()
        };
        cmd._pad[0..4].copy_from_slice(&0.01_f32.to_ne_bytes());
        cmd._pad[4..8].copy_from_slice(&0.005_f32.to_ne_bytes());
        cmd._pad[8] = 0xa1;
        cmd._pad[9] = 80;
        let target = GimbalAimTarget::from_solver_degrees(0.0, 0.0);
        let aligned = Quat::from_rotation_x(-std::f32::consts::FRAC_PI_2);
        assert!(fire_aligned(&cmd, target, aligned, 1_010_000_000, false));
        assert!(!fire_aligned(
            &cmd,
            target,
            Quat::IDENTITY,
            1_010_000_000,
            false
        ));
        assert!(!fire_aligned(&cmd, target, aligned, 1_110_000_000, true));
        let edge = Quat::from_euler(EulerRot::YXZ, 0.009, -std::f32::consts::FRAC_PI_2, 0.0);
        assert!(!fire_aligned(&cmd, target, edge, 1_010_000_000, false));
        assert!(fire_aligned(&cmd, target, edge, 1_010_000_000, true));
        cmd._pad[4..8].copy_from_slice(&f32::NAN.to_ne_bytes());
        assert!(!fire_aligned(&cmd, target, aligned, 1_010_000_000, false));
        cmd._pad = [0; 11];
        assert!(fire_aligned(
            &cmd,
            target,
            Quat::IDENTITY,
            1_010_000_000,
            false
        ));
    }

    #[test]
    fn solver_angles_are_preserved_and_fire_permission_resets_on_switches() {
        let mut state = CommandState::default();
        let mut cmd = GimbalCmd {
            timestamp_ns: 1_000_000_000,
            fire_advice: 2,
            ..default()
        };
        cmd._pad[0..4].copy_from_slice(&0.01_f32.to_ne_bytes());
        cmd._pad[4..8].copy_from_slice(&0.005_f32.to_ne_bytes());
        cmd._pad[8] = 0xa1;
        cmd._pad[9] = 80;
        cmd._pad[10] = 50;
        assert!(valid_command(&cmd, cmd.timestamp_ns));
        state.update(cmd);
        assert!(!state.fire_ready(cmd.timestamp_ns));
        cmd.timestamp_ns += 50_000_000;
        cmd.yaw_deg = (0.25_f32 * 0.05).to_degrees();
        state.update(cmd);
        assert!(state.stable);
        assert_eq!(state.target(), command_target(&cmd));
        state.last_fire_ns = cmd.timestamp_ns;
        assert!(!state.fire_ready(cmd.timestamp_ns + 499_000_000));
        assert!(state.fire_ready(cmd.timestamp_ns + 500_000_000));
        cmd.timestamp_ns += 50_000_000;
        cmd.yaw_deg += 20.0;
        state.update(cmd);
        assert!(!state.stable);
        assert!((state.target().yaw - cmd.yaw_deg.to_radians()).abs() < 1e-5);
    }
}

fn heartbeat_system(context: Option<Res<TalosCaptureContext>>) {
    if let Some(ctx) = context {
        if let Ok(mut publisher) = ctx.publisher.lock() {
            publisher.update_heartbeat();
        }
    }
}

pub fn publish_pose(
    context: &TalosCaptureContext,
    index: PoseIndex,
    position: [f32; 3],
    quaternion: [f32; 4],
    frame_seq: u64,
    timestamp_ns: u64,
) {
    if let Ok(mut publisher) = context.publisher.lock() {
        publisher.publish_pose(index, position, quaternion, frame_seq, timestamp_ns);
    }
}

pub fn recv_gimbal_cmd(subscriber: &ShmSubscriberRes) -> Option<GimbalCmd> {
    subscriber.0.lock().ok()?.recv_gimbal_cmd()
}

pub const M_ALIGN_MAT3: Mat3 = Mat3::from_cols(
    Vec3::new(0.0, -1.0, 0.0), // M[0,0], M[1,0], M[2,0]
    Vec3::new(0.0, 0.0, 1.0),  // M[0,1], M[1,1], M[2,1]
    Vec3::new(-1.0, 0.0, 0.0), // M[0,2], M[1,2], M[2,2]
);

#[inline]
pub fn to_ros(bevy_transform: Transform) -> Transform {
    let new_rotation = to_ros_quat(bevy_transform.rotation);
    let new_translation = to_ros_translation(bevy_transform.translation);
    Transform::from_translation(new_translation).with_rotation(new_rotation)
}

pub fn to_ros_translation(vec3: Vec3) -> Vec3 {
    let align_rot_mat = M_ALIGN_MAT3;
    let new_translation = align_rot_mat * vec3;
    new_translation
}

pub fn to_ros_quat(quat: Quat) -> Quat {
    let align_rot_mat = M_ALIGN_MAT3;
    let align_quat = Quat::from_mat3(&align_rot_mat);
    let new_rotation = align_quat * quat * align_quat.inverse();
    new_rotation
}
