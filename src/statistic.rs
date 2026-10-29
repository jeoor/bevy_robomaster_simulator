use bevy::prelude::*;
use std::collections::VecDeque;

#[derive(Resource, Default, Reflect)]
#[reflect(Resource)]
pub struct ProjectileStatistics {
    pub launch_count: u32,
    pub accurate_count: u32,
    #[reflect(ignore)]
    shots: VecDeque<(Entity, bool)>,
}

impl ProjectileStatistics {
    pub fn record_launch(&mut self, projectile: Entity) {
        self.launch_count += 1;
        self.shots.push_back((projectile, false));
        if self.shots.len() > 512 {
            self.shots.pop_front();
        }
    }

    pub fn record_hit(&mut self, projectile: Entity) -> bool {
        let Some((_, hit)) = self.shots.iter_mut().find(|(id, _)| *id == projectile) else {
            return false;
        };
        // Commands are deferred; multiple collider events can arrive before a component removal.
        if *hit {
            return false;
        }
        *hit = true;
        self.accurate_count += 1;
        true
    }

    pub fn accurate_pct(&self) -> f32 {
        if self.launch_count == 0 {
            return 0.0;
        }
        (self.accurate_count as f32) / (self.launch_count as f32)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn each_launched_projectile_counts_at_most_one_hit() {
        let mut world = World::new();
        let projectile = world.spawn_empty().id();
        let mut stats = ProjectileStatistics::default();
        assert!(!stats.record_hit(projectile));
        stats.record_launch(projectile);
        assert!(stats.record_hit(projectile));
        assert!(!stats.record_hit(projectile));
        assert_eq!((stats.launch_count, stats.accurate_count), (1, 1));
        assert_eq!(stats.accurate_pct(), 1.0);
    }
}
