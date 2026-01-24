use std::f64::consts::PI;
use bevy::app::{App, Plugin};
use bevy_ecs::change_detection::ResMut;
use bevy_ecs::prelude::IntoScheduleConfigs;
use bevy_egui::{egui, EguiContexts, EguiPrimaryContextPass};
use bevy_egui::egui::Color32;
use crate::algo::cnc::{add_lra_row, delete_lra_row, LRACLR};

use crate::ui::UiOrder;
use bevy::prelude::*;

use crate::states::pipe_control::PipeSpecification;
use crate::states::scene_control::AppMode;
use crate::states::state_machine::{MachineRegisters, RobotState};

pub struct LeftUiPanelPlugin;

impl Plugin for LeftUiPanelPlugin {
    fn build(&self, app: &mut App) {
        app.add_systems(EguiPrimaryContextPass, ui_system.in_set(UiOrder::Left));
    }
}

fn ui_system(
    mut contexts: EguiContexts,
    mut pipe_spec: ResMut<PipeSpecification>,
    mut machine_registers:  ResMut<MachineRegisters>,
    mut next_state: ResMut<NextState<AppMode>>, 
    current_state: Res<State<AppMode>>, 
    curr_robot_state: Res<State<RobotState>>,
    mut next_robot_state: ResMut<NextState<RobotState>>,
){
    let Ok(ctx) = contexts.ctx_mut() else { return; };
    egui_material_icons::initialize(ctx);

    egui::SidePanel::left("left_panel").resizable(true).show(ctx, |ui| {

        let clamp=machine_registers.clamp;
        let col_width = 50.0;
        let col_heigth = 8.0;
        let mut new_diameter = f64::MAX;
        let mut pipe_diameter=pipe_spec.diameter.to_string();
        let total_length = ((pipe_spec.len) as i32).to_string();

        egui::ScrollArea::vertical().show(ui, |ui| {

            ui.horizontal(|ui| {
                let color = egui::Color32::from_rgb(255, 255, 255);
                let L = format!("Total len {} mm. ", total_length);
                ui.label(egui::RichText::new(L).color(color));
                ui.label(egui::RichText::new("Pipe D= ").color(color));
                let radius_labl = ui.add(egui::TextEdit::singleline(&mut pipe_diameter).text_color(color));
            });

            ui.horizontal(|ui| {
                let color = egui::Color32::from_rgb(255, 255, 255);
                ui.add_sized([col_width, col_heigth],
                             egui::Label::new(egui::RichText::new("L").color(color)),
                );
                ui.separator();
                ui.add_sized([col_width, col_heigth],
                             egui::Label::new(egui::RichText::new("R").color(color)),
                );
                ui.separator();
                ui.add_sized([col_width, col_heigth],
                             egui::Label::new(egui::RichText::new("A").color(color)),
                );
                ui.separator();
                ui.add_sized([col_width, col_heigth],
                             egui::Label::new(egui::RichText::new("Bend R").color(color)),
                );
                ui.separator();
            });
            ui.separator();

            let mut is_changed = false;
            let mut counter: i32 = 0;
            let mut deleted_index: i32 = 0;
            let mut add_index: i32 = -1;
            let last_index = pipe_spec.segments.len() as i32 - 1;

            for lra_item in &mut pipe_spec.segments {
                let color_white: Color32 = egui::Color32::from_rgb(255, 255, 255);
                let color_l: Color32 = {
                    if (lra_item.id1 == machine_registers.current_id as i32) {
                        egui::Color32::from_rgb(255, 0, 0)
                    } else {
                        egui::Color32::from_rgb(255, 255, 255)
                    }
                };
                let color_r: Color32 = {
                    if (lra_item.id2 ==  machine_registers.current_id as i32) {
                        egui::Color32::from_rgb(255, 0, 0)
                    } else {
                        egui::Color32::from_rgb(255, 255, 255)
                    }
                };
                let color_a: Color32 = {
                    if (lra_item.id2 ==  machine_registers.current_id as i32) {
                        egui::Color32::from_rgb(255, 0, 0)
                    } else {
                        egui::Color32::from_rgb(255, 255, 255)
                    }
                };


                ui.horizontal(|ui| {


                    let l_labl = ui.scope(|ui| {
                        let style = ui.visuals_mut();
                        style.widgets.inactive.bg_stroke = egui::Stroke::new(1.0, color_l);
                        ui.add_sized([col_width, ui.available_height()], egui::DragValue::new(&mut lra_item.l).speed(1.0))
                    }).inner;
                    if (l_labl.changed()) { is_changed = true;}
                    ui.separator();

                    let r_labl = ui.scope(|ui| {
                        let style = ui.visuals_mut();
                        style.widgets.inactive.bg_stroke = egui::Stroke::new(1.0, color_r);
                        ui.add_sized([col_width, ui.available_height()], egui::DragValue::new(&mut lra_item.r).speed(1.0))
                    }).inner;
                    if (r_labl.changed() ) { is_changed = true; }
                    ui.separator();

                    let a_labl = ui.scope(|ui| {
                        let style = ui.visuals_mut();
                        style.widgets.inactive.bg_stroke = egui::Stroke::new(1.0, color_a);
                        ui.add_sized([col_width, ui.available_height()], egui::DragValue::new(&mut lra_item.a).speed(1.0))
                    }).inner;

                    if (a_labl.changed() ) { is_changed = true; }

                    ui.separator();


                    let clr_labl = ui.scope(|ui| {
                        let style = ui.visuals_mut();
                        style.widgets.inactive.bg_stroke = egui::Stroke::new(1.0, color_a);
                        ui.add_sized([col_width, ui.available_height()], egui::DragValue::new(&mut lra_item.clr).speed(1.0))
                    }).inner;


                    if (clr_labl.changed()) {
                        is_changed = true;
                    }

                    ui.separator();

                    let is_add_button_disabled = if (counter != last_index) { true } else { false };
                    let add_button = ui.add_enabled(is_add_button_disabled, egui::Button::new("+"));
                    if (add_button.clicked()) {
                        add_index = counter;
                        is_changed = true;
                    }

                    let is_delete_button_disabled = if (counter != 0 && last_index >= 2 && counter != last_index)
                    { true } else { false };
                    let delete_button = ui.add_enabled(is_delete_button_disabled, egui::Button::new("x"));
                    if (delete_button.clicked()) {
                        deleted_index = counter;
                        is_changed = true;
                    }
                });

                counter = counter + 1;
            }

            if(deleted_index!=0){
                let new_lra=delete_lra_row(deleted_index,&pipe_spec.segments);
                pipe_spec.segments=new_lra;
            }

            if(add_index!= -1){
                let new_lra=add_lra_row(add_index,&pipe_spec.segments);
                pipe_spec.segments=new_lra;
            }

            if(is_changed){
                next_state.set( AppMode::Restarting);
            }
        });
    });

}


