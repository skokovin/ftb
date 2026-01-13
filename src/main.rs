#![cfg_attr(not(debug_assertions), windows_subsystem = "windows")]
use bevy::prelude::*;
use bevy::scene::ScenePlugin;
use bevy_egui::{EguiGlobalSettings, EguiPlugin, EguiPrimaryContextPass};
use crate::render::line::LineMaterial;
use crate::states::machine_control::MachineControlPlugin;
use crate::states::pipe_control::{PipeSpecification, PipeViewPlugin};
use crate::states::scene_control::{AppMode, AppScenePlugin};
use crate::states::state_machine::{MachineRegisters, MachineRegistersPlugin, RobotState};
use crate::ui::axes::AxesPlugin;
use crate::ui::bottompanel::BottomUiPanelPlugin;
use crate::ui::camera::cad_camera_controller;
use crate::ui::leftpanel::LeftUiPanelPlugin;
use crate::ui::toppanel::TopUiPanelPlugin;
use crate::ui::UiOrder;

use bevy::winit::WinitWindows; // Доступ к низкоуровневым окнам winit
use bevy::window::PrimaryWindow;
use winit::window::Icon;

mod states;
mod algo;
mod ui;
mod render;

fn main() {
    let egui_settings = EguiGlobalSettings {
        auto_create_primary_context: true,
        enable_focused_non_window_context_updates: false,
        input_system_settings: Default::default(),
        enable_absorb_bevy_input_system: false,
        enable_cursor_icon_updates: false,
        enable_ime: false,
    };

    App::new()
        .add_plugins(DefaultPlugins)
        .insert_resource(egui_settings)
        .init_resource::<MachineRegisters>()
        .init_resource::<PipeSpecification>()
        .init_state::<AppMode>()
        .init_state::<RobotState>()
        .configure_sets(EguiPrimaryContextPass, (UiOrder::Top, UiOrder::Left, UiOrder::Bottom).chain())
        .add_plugins((
            MaterialPlugin::<LineMaterial>::default(),
            EguiPlugin::default(),
            LeftUiPanelPlugin,
            BottomUiPanelPlugin,
            TopUiPanelPlugin,
            AppScenePlugin,
            PipeViewPlugin,
            MachineRegistersPlugin,
            MachineControlPlugin,
        ))
        .run();

}
