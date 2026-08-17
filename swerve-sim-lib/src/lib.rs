#![feature(random)]
use rapier3d::geometry::{Group, InteractionGroups, InteractionTestMode::Or};

pub mod field;
pub mod physics_world;
pub mod robot;
pub mod swerve_module;
pub mod util;

// TODO: This doesnt allow robots to drive ontop of each other
pub const SWERVE_INTERACTION_GROUPS: InteractionGroups = InteractionGroups {
    memberships: Group::GROUP_2,
    filter: Group::GROUP_1,
    test_mode: Or,
};
pub const BUMPER_INTERACTION_GROUPS: InteractionGroups = InteractionGroups {
    memberships: Group::union(Group::GROUP_2, Group::GROUP_3),
    filter: Group::union(Group::GROUP_1, Group::GROUP_3),
    test_mode: Or,
};

pub const FIELD_INTERACTION_GROUPS: InteractionGroups =
    InteractionGroups::new(Group::GROUP_1, Group::all(), Or);

pub const SLEEP_THRESHOLD: f32 = 0.1;

#[macro_export]
macro_rules! inch_to_m {
    ($x:expr) => {
        value!(rescale!(quantity!($x, inch, f32), m, f32), m, f32)
    };
}
