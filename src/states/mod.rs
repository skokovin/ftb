use std::collections::HashSet;
use bevy::prelude::*;
use bevy_ecs::prelude::*;
use ordered_float::OrderedFloat;
use crate::algo::analyze_stp;
use crate::algo::cnc::LRACLR;
use crate::states::pipe_control::PipeSpecification;

pub mod state_machine;
pub mod pipe_control;
pub mod scene_control;
pub mod machine_control;
