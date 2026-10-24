use bevy::prelude::*;
use bevy::render::view::screenshot::{Capturing, Screenshot, save_to_disk};
use bevy::window::{CursorIcon, SystemCursorIcon, Window};

use crate::components::{SlapperInfantry, SubscribeAutoAim};
use crate::robomaster::prelude::{Armor, ArmorStickerSelection};
use crate::statistic::ProjectileStatistics;
use crate::systems::ControllerState;

#[derive(Component)]
pub struct HelpText;

fn create_help_text(
    auto_aim: bool,
    stats: &ProjectileStatistics,
    controller: &ControllerState,
) -> Text {
    format!(
        "auto-aim={} total={} accurate={} pct={:.2}\ncontroller={} mode={} gyro={} remote-gyro={}\n{}",
        if auto_aim { "ON " } else { "OFF" },
        stats.launch_count,
        stats.accurate_count,
        stats.accurate_pct(),
        controller.help_source(),
        controller.help_mode(),
        if controller.controlled_chassis_spin() {
            "ON"
        } else {
            "OFF"
        },
        if controller.remote_chassis_spin() {
            "ON"
        } else {
            "OFF"
        },
        controller.help_controls()
    )
    .into()
}

pub fn spawn_text(commands: &mut Commands) {
    commands.spawn((
        HelpText,
        Text::new(""),
        Node {
            position_type: PositionType::Absolute,
            bottom: Val::Px(12.0),
            left: Val::Px(12.0),
            ..default()
        },
    ));
}

pub fn update_help_text(
    mut text: Query<&mut Text, With<HelpText>>,
    auto_aim: Res<SubscribeAutoAim>,
    stats: Res<ProjectileStatistics>,
    controller: Res<ControllerState>,
) {
    let next = create_help_text(
        auto_aim.load(std::sync::atomic::Ordering::Acquire),
        &stats,
        &controller,
    );
    for mut text in &mut text {
        text.set_if_neq(next.clone());
    }
}

pub fn change_appearance(
    keyboard: Res<ButtonInput<KeyCode>>,
    selections: Query<&mut ArmorStickerSelection, With<SlapperInfantry>>,
    owned: Query<&mut Armor, With<SlapperInfantry>>,
) {
    if keyboard.pressed(KeyCode::ShiftLeft) && keyboard.just_pressed(KeyCode::KeyC) {
        let mut n_type = None;
        for mut selection in selections {
            let new_typ = selection.advance_debug_sequence();
            n_type = Some(new_typ);
        }
        if let Some(n_type) = n_type {
            for mut own in owned {
                own.label = n_type;
            }
        }
    }
}

pub fn screenshot_on_f2(mut commands: Commands, mut counter: Local<u32>) {
    let path = format!("./screenshot-{}.png", *counter);
    *counter += 1;
    commands
        .spawn(Screenshot::primary_window())
        .observe(save_to_disk(path));
}

pub fn screenshot_saving(
    mut commands: Commands,
    screenshot_saving: Query<Entity, With<Capturing>>,
    window: Single<Entity, With<Window>>,
) {
    match screenshot_saving.iter().count() {
        0 => {
            commands.entity(*window).remove::<CursorIcon>();
        }
        x if x > 0 => {
            commands
                .entity(*window)
                .insert(CursorIcon::from(SystemCursorIcon::Progress));
        }
        _ => {}
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use bevy::ecs::system::RunSystemOnce;
    use std::sync::atomic::AtomicBool;

    #[test]
    fn help_text_only_invalidates_layout_when_its_content_changes() {
        let mut world = World::new();
        world.insert_resource(SubscribeAutoAim(AtomicBool::new(false)));
        world.init_resource::<ProjectileStatistics>();
        world.init_resource::<ControllerState>();
        let help = world.spawn((HelpText, Text::new(""))).id();
        let unrelated = world.spawn(Text::new("other UI")).id();

        world.run_system_once(update_help_text).unwrap();
        world.clear_trackers();
        world.run_system_once(update_help_text).unwrap();
        assert!(!world.entity(help).get_ref::<Text>().unwrap().is_changed());
        assert_eq!(world.get::<Text>(unrelated).unwrap().0, "other UI");

        world
            .resource_mut::<ProjectileStatistics>()
            .increase_launch();
        world.run_system_once(update_help_text).unwrap();
        assert!(world.entity(help).get_ref::<Text>().unwrap().is_changed());
        assert!(world.get::<Text>(help).unwrap().0.contains("total=1"));
    }
}
