#![feature(random)]
use rapier3d::geometry::{Group, InteractionGroups, InteractionTestMode::Or};

pub mod physics_world;
pub mod swerve_module;
pub mod util;

pub const ROBOT_INTERACTION_GROUPS: InteractionGroups = InteractionGroups {
    memberships: Group::GROUP_2,
    filter: Group::GROUP_1,
    test_mode: Or,
};

pub const FIELD_INTERACTION_GROUPS: InteractionGroups = InteractionGroups::new(
    Group::GROUP_1,
    Group::union(Group::GROUP_1, Group::GROUP_2),
    Or,
);
