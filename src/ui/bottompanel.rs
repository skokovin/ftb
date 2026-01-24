use bevy::app::{App, Plugin};
use bevy::prelude::*; // Res, ResMut, State и т.д.
use bevy_egui::{egui, EguiContexts, EguiPrimaryContextPass};
use crate::states::scene_control::AppMode;
use crate::ui::UiOrder;
use crate::states::state_machine::{MachineRegisters, RobotState};


pub struct BottomUiPanelPlugin;

impl Plugin for BottomUiPanelPlugin {
    fn build(&self, app: &mut App) {
        app.add_systems(EguiPrimaryContextPass, ui_system.in_set(UiOrder::Bottom));
    }
}

fn ui_system(
    mut contexts: EguiContexts,
    mut status: ResMut<MachineRegisters>, 
    mut next_state: ResMut<NextState<AppMode>>, 
    current_state: Res<State<AppMode>>, 
    curr_robot_state: Res<State<RobotState>>,
    mut next_robot_state: ResMut<NextState<RobotState>>,
) {
    let Ok(ctx) = contexts.ctx_mut() else { return; };

  
    egui_material_icons::initialize(ctx);

    egui::TopBottomPanel::bottom("bottom_panel").show(ctx, |ui| {
        ui.horizontal(|ui| {

            // 1. КНОПКА PLAY / PAUSE
            let is_simulating = *current_state.get() == AppMode::Simulating;

            let icon = if is_simulating {
                egui_material_icons::icons::ICON_PAUSE
            } else {
                egui_material_icons::icons::ICON_PLAY_ARROW
            };

          
            if ui.button(egui_material_icons::icons::ICON_FAST_REWIND).clicked() {
                status.t = 0.0; // Сброс в начало
            }
            if ui.button(icon).clicked() {
                if is_simulating {
                    next_state.set(AppMode::Pause); // Пауза
                } else {
                    next_state.set(AppMode::Simulating); // Старт
                }
            }
            if ui.button(egui_material_icons::icons::ICON_FAST_FORWARD).clicked() {
                status.t = 0.0; // Сброс в начало
            }
            
            ui.style_mut().spacing.slider_width = ui.available_width() - 50.0;

            let slider = ui.add(egui::Slider::new(&mut status.t, 0.0..=1.0));

            if slider.dragged() {
                next_state.set(AppMode::Pause);
            }
        });
    });
}
