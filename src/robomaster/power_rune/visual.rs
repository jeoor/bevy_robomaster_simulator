use crate::all_arg_constructor;
use crate::robomaster::power_rune::common::{RUNE_TARGET_COUNT, RuneMode};
use crate::robomaster::power_rune::state::MechanismState;
use crate::robomaster::visibility::{Activation, Control, Controller, StatefulAppearance};
use bevy::prelude::Component;

all_arg_constructor!(
    pub struct RuneVisual {
        target: Controller,
        legging_segments: [Controller; 3],
        padding_segments: Controller,
        progress_segments: Controller,
    }
);

impl RuneVisual {
    pub fn apply(
        &mut self,
        mode: RuneMode,
        activation: Activation,
        appearance: &mut StatefulAppearance,
    ) {
        match mode {
            RuneMode::Small => {
                self.target.set(activation, appearance);
                for swap in &mut self.legging_segments {
                    swap.set(activation, appearance);
                }
            }
            RuneMode::Large => {
                self.target.set(
                    match activation {
                        Activation::Activated => Activation::Deactivated,
                        _ => activation,
                    },
                    appearance,
                );
                match activation {
                    Activation::Activated => self.legging_segments[0].set(activation, appearance),
                    _ => {
                        for legging in &mut self.legging_segments {
                            legging.set(activation, appearance);
                        }
                    }
                }
            }
        }

        self.padding_segments.set(activation, appearance);
        self.progress_segments.set(activation, appearance);
    }
}

#[derive(Component)]
pub struct PowerRuneVisuals {
    root: Controller,
    targets: [RuneVisual; RUNE_TARGET_COUNT],
    applied: Option<VisualState>,
}

/// Only the rendered state belongs here. Countdown timers do not affect appearance.
#[derive(Clone, Copy, PartialEq, Eq)]
struct VisualState {
    mode: RuneMode,
    root: Activation,
    targets: [Activation; RUNE_TARGET_COUNT],
}

impl PowerRuneVisuals {
    pub fn new(root: Controller, targets: [RuneVisual; RUNE_TARGET_COUNT]) -> Self {
        Self {
            root,
            targets,
            applied: None,
        }
    }

    pub fn apply(
        &mut self,
        mode: RuneMode,
        state: &MechanismState,
        appearance: &mut StatefulAppearance,
    ) {
        let next = VisualState {
            mode,
            root: state.root_activation(),
            targets: state.target_states(),
        };
        if self.applied == Some(next) {
            return;
        }
        self.root.set(next.root, appearance);
        for (target, activation) in self.targets.iter_mut().zip(next.targets) {
            target.apply(mode, activation, appearance);
        }
        self.applied = Some(next);
    }
}
