mod camera;
mod chassis_observation;
mod controller;
mod debug;
mod enemy_ai;
mod gimbal_pid;
mod input;
mod projectile;
mod uav;
pub use camera::*;
pub use chassis_observation::*;
pub use controller::*;
pub use debug::*;
pub use enemy_ai::*;
pub use gimbal_pid::*;
pub use input::*;
pub use projectile::*;
pub use uav::*;

use bevy::prelude::*;

#[derive(SystemSet, Clone, PartialEq, Eq, Hash, Debug)]
pub enum GameplaySystems {
    Input,
    GameLogic,
    Camera,
    Cleanup,
}
