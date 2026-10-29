use bevy::camera::RenderTarget;
use bevy::prelude::*;
use bevy::window::PrimaryWindow;
use std::time::{SystemTime, UNIX_EPOCH};
use talos_ipc::{AimOverlay, OVERLAY_MAX_ARMORS};

use super::plugin::ShmSubscriberRes;
use crate::capture::setup_preview_window;
use crate::components::{CameraMode, FollowingType};
use crate::systems::GameplaySystems;

const EDGES: usize = OVERLAY_MAX_ARMORS * 4;

#[derive(Resource)]
struct OverlayDisplay {
    visible: bool,
    frame: AimOverlay,
}

impl Default for OverlayDisplay {
    fn default() -> Self {
        Self {
            visible: true,
            frame: AimOverlay::default(),
        }
    }
}

#[derive(Component)]
struct OverlayPart(usize);

pub struct AimOverlayPlugin;

impl Plugin for AimOverlayPlugin {
    fn build(&self, app: &mut App) {
        app.init_resource::<OverlayDisplay>()
            .add_systems(Startup, setup_overlay.after(setup_preview_window))
            .add_systems(Update, update_overlay.after(GameplaySystems::Camera));
    }
}

fn setup_overlay(mut commands: Commands, cameras: Query<(Entity, &Camera, &RenderTarget)>) {
    // Explicitly target a window camera: the capture texture must remain free of UI marks.
    let Some((camera, _, _)) = cameras
        .iter()
        .filter(|(_, c, target)| c.is_active && matches!(target, RenderTarget::Window(_)))
        .max_by_key(|(_, c, _)| c.order)
    else {
        return;
    };
    commands.spawn((
        Node {
            position_type: PositionType::Absolute,
            left: Val::Px(0.0),
            top: Val::Px(0.0),
            width: Val::Percent(100.0),
            height: Val::Percent(100.0),
            overflow: Overflow::clip(),
            ..default()
        },
        UiTargetCamera(camera),
        GlobalZIndex(10),
    )).with_children(|root| {
        for index in 0..EDGES + 2 {
            root.spawn((
                OverlayPart(index),
                Node {
                    position_type: PositionType::Absolute,
                    display: Display::None,
                    ..default()
                },
                UiTransform::default(),
                BackgroundColor(Color::srgb(0.0, 1.0, 0.0)),
            ));
        }
    });
}

fn screen_point(uv: [f32; 2], size: Vec2) -> Option<Vec2> {
    let p = Vec2::from_array(uv);
    (p.is_finite() && p.abs().max_element() <= 4.0).then_some(p * size)
}

fn line(node: &mut Node, transform: &mut UiTransform, a: Vec2, b: Vec2) {
    let delta = b - a;
    let length = delta.length();
    let center = (a + b) * 0.5;
    node.display = Display::Flex;
    node.left = Val::Px(center.x - length * 0.5);
    node.top = Val::Px(center.y - 1.0);
    node.width = Val::Px(length);
    node.height = Val::Px(2.0);
    transform.rotation = Rot2::radians(delta.y.atan2(delta.x));
}

fn update_overlay(
    keyboard: Res<ButtonInput<KeyCode>>,
    subscriber: Option<Res<ShmSubscriberRes>>,
    mut overlay: ResMut<OverlayDisplay>,
    mode: Res<CameraMode>,
    window: Single<&Window, With<PrimaryWindow>>,
    mut parts: Query<(
        &OverlayPart,
        &mut Node,
        &mut UiTransform,
        &mut BackgroundColor,
    )>,
) {
    if keyboard.just_pressed(KeyCode::F6) {
        overlay.visible = !overlay.visible;
    }
    if let Some(packet) = subscriber.and_then(|s| s.0.lock().ok().and_then(|mut s| s.recv_overlay())) {
        overlay.frame = packet;
    }
    let now = SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .map(|d| d.as_nanos() as u64)
        .unwrap_or(0);
    let frame = &overlay.frame;
    let fresh = frame.timestamp_ns != 0
        && now.saturating_sub(frame.timestamp_ns) <= 500_000_000
        && frame.timestamp_ns.saturating_sub(now) <= 50_000_000;
    let show = overlay.visible && fresh && mode.0 == FollowingType::Robot;
    let size = Vec2::new(window.width(), window.height());
    for (part, mut node, mut transform, mut background) in &mut parts {
        node.display = Display::None;
        if !show {
            continue;
        }
        if part.0 < EDGES {
            let armor = part.0 / 4;
            let edge = part.0 % 4;
            if armor >= (frame.armor_count as usize).min(OVERLAY_MAX_ARMORS) {
                continue;
            }
            let points = frame.armors[armor];
            if let (Some(a), Some(b)) = (
                screen_point(points[edge], size),
                screen_point(points[(edge + 1) % 4], size),
            ) {
                line(&mut node, &mut transform, a, b);
            }
        } else if part.0 < EDGES + 2 && frame.aim_valid == 1 {
            if let Some(p) = screen_point(frame.aim, size) {
                let delta = if part.0 == EDGES { Vec2::X } else { Vec2::Y } * 9.0;
                line(&mut node, &mut transform, p - delta, p + delta);
                background.0 = Color::srgb(1.0, 1.0, 0.0);
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use bevy::ecs::system::RunSystemOnce;

    #[test]
    fn markers_target_the_window_and_f6_only_changes_visibility() {
        let mut world = World::new();
        world.init_resource::<ButtonInput<KeyCode>>();
        world.init_resource::<OverlayDisplay>();
        world.insert_resource(CameraMode(FollowingType::Robot));
        world.spawn((Window::default(), PrimaryWindow));
        let window_camera = world
            .spawn((Camera::default(), RenderTarget::default()))
            .id();
        let capture_camera = world.spawn((
            Camera { order: 100, ..default() },
            RenderTarget::Image(Handle::<Image>::default().into()),
        )).id();
        world.run_system_once(setup_overlay).unwrap();
        let target = world.query::<&UiTargetCamera>().single(&world).unwrap().0;
        assert_eq!(target, window_camera);
        assert_ne!(target, capture_camera, "Markers must not enter the detector's capture image");
        let stamp = SystemTime::now()
            .duration_since(UNIX_EPOCH)
            .unwrap()
            .as_nanos() as u64;
        {
            let mut display = world.resource_mut::<OverlayDisplay>();
            display.frame.timestamp_ns = stamp;
            display.frame.armor_count = 1;
            display.frame.armors[0] = [[0.25, 0.25], [0.5, 0.25], [0.5, 0.5], [0.25, 0.5]];
            display.frame.aim_valid = 1;
            display.frame.aim = [0.5, 0.5];
        }
        let visible_parts = |world: &mut World| {
            world
                .query_filtered::<&Node, With<OverlayPart>>()
                .iter(world)
                .filter(|n| n.display != Display::None)
                .count()
        };
        world.run_system_once(update_overlay).unwrap();
        assert_eq!(visible_parts(&mut world), 6); // Four edges and two cross lines.
        let first = world
            .query::<(&OverlayPart, &Node)>()
            .iter(&world)
            .find(|(part, _)| part.0 == 0)
            .unwrap()
            .1;
        assert_eq!(first.left, Val::Px(320.0));
        assert_eq!(first.width, Val::Px(320.0));
        world.resource_mut::<ButtonInput<KeyCode>>().press(KeyCode::F6);
        world.run_system_once(update_overlay).unwrap();
        assert_eq!(visible_parts(&mut world), 0);
        assert_eq!(world.resource::<OverlayDisplay>().frame.timestamp_ns, stamp);
        assert_eq!(world.resource::<OverlayDisplay>().frame.armor_count, 1);
        world.resource_mut::<ButtonInput<KeyCode>>().reset_all();
        world.resource_mut::<ButtonInput<KeyCode>>().press(KeyCode::F6);
        world.run_system_once(update_overlay).unwrap();
        assert_eq!(visible_parts(&mut world), 6);
        world.resource_mut::<ButtonInput<KeyCode>>().reset_all();
        world.resource_mut::<OverlayDisplay>().frame.timestamp_ns = stamp - 600_000_000;
        world.run_system_once(update_overlay).unwrap();
        assert_eq!(visible_parts(&mut world), 0, "Expired detections must disappear");
    }
}
