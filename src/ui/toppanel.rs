use std::fs;
use bevy::app::{App, Plugin};
use bevy::prelude::{default, NextState, State};
use bevy_ecs::change_detection::{Res, ResMut};
use bevy_ecs::prelude::IntoScheduleConfigs;
use bevy_egui::{egui, EguiContexts, EguiPrimaryContextPass};
use rfd::FileDialog;
use crate::algo::cnc::{reverse_lraclr, save_csv};
use crate::states::pipe_control::PipeSpecification;
use crate::states::scene_control::AppMode;
use crate::states::state_machine::{MachineRegisters, RobotState};
use crate::ui::UiOrder;

pub struct TopUiPanelPlugin;

impl Plugin for TopUiPanelPlugin {
    fn build(&self, app: &mut App) {
        app
            // Добавляем системы в расписание Update
            .add_systems(EguiPrimaryContextPass, ui_system.in_set(UiOrder::Top));

        // Если вам нужно инициализировать ресурсы при старте:
        // .init_resource::<MyUiState>()
    }
}




fn ui_system(
    mut contexts: EguiContexts,
    mut pipe_spec: ResMut<PipeSpecification>,
    mut next_state: ResMut<NextState<AppMode>>, // Переключаем режимы
    current_state: Res<State<AppMode>>, // Узнаем текущий режим
    curr_robot_state: Res<State<RobotState>>,
    mut next_robot_state: ResMut<NextState<RobotState>>,
    mut machine_registers:  ResMut<MachineRegisters>,
) {
    let Ok(ctx) = contexts.ctx_mut() else {
        return;
    };

    egui::TopBottomPanel::top("my_panel").show(ctx, |ui| {
        let eye_icon = if true {
            egui_material_icons::icons::ICON_DRY // Open Eye
        } else {
            egui_material_icons::icons::ICON_DO_NOT_TOUCH // Closed Eye (See-No-Evil)
        };
        let centerline_eye_icon = if true {
            egui_material_icons::icons::ICON_CYCLONE // Open Eye
        } else {
            egui_material_icons::icons::ICON_DESELECT // Closed Eye (See-No-Evil)
        };
        ui.horizontal_wrapped(|ui| {
            if ui.button("File").clicked() {
              if let Some(path) = rfd::FileDialog::new().add_filter("STEP", &["stp", "step"]).pick_file() {
                        match fs::read(path) {
                            Ok(stp) => {
                                pipe_spec.init_pipe(&stp);
                                next_state.set(AppMode::Restarting);
                            }
                            Err(_) => {}
                        }
                    }
            };
            ui.separator();
            if ui.button("Reverse").clicked() {
                let lraclr_arr = reverse_lraclr(&pipe_spec.segments);
                pipe_spec.segments=lraclr_arr;
                next_state.set( AppMode::Restarting);
            };

            ui.separator();
            if ui.button("CSV").clicked() {
                if let Some(path) = FileDialog::new().add_filter("CSV", &["csv"]).set_directory("/").save_file() {
                    save_csv(& pipe_spec.segments, &path);
                }


            }
            ui.separator();

            if ui.button("STP").clicked() {
                if let Some(path) = FileDialog::new().add_filter("STP", &["stp"]).set_directory("/").save_file() {
                    println!("{:?}", path);
                }
            }
            ui.separator();

            ui.menu_button("Demos", |ui| {
                ui.style_mut().wrap_mode = Some(egui::TextWrapMode::Extend);
                let mut stp_indx: usize = 50;
                if ui.button("Demo1").clicked() {
                    stp_indx=0;
                    //let stp: &[u8] =pipe_spec.demos[0].as_slice();
                    //stp = Vec::from((include_bytes!("../files/1.stp")).as_slice());
                    ui.close();
                };
                if ui.button("Demo2").clicked() {
                    //stp = Vec::from((include_bytes!("../files/2.stp")).as_slice());
                    stp_indx=1;
                    ui.close();
                };
                if ui.button("Demo3").clicked() {
                    stp_indx=2;
                    //stp = Vec::from((include_bytes!("../files/3.stp")).as_slice());
                    ui.close();
                };
                if ui.button("Demo4").clicked() {
                    stp_indx=3;
                    //stp = Vec::from((include_bytes!("../files/4.stp")).as_slice());
                    ui.close();
                };
                if ui.button("Demo5").clicked() {
                    stp_indx=4;
                    //stp = Vec::from((include_bytes!("../files/5.stp")).as_slice());
                    ui.close();
                };
                if ui.button("Demo6").clicked()
                {
                    stp_indx=5;
                    //stp = Vec::from((include_bytes!("../files/6.stp")).as_slice());
                    ui.close();
                };
                if ui.button("Demo7").clicked() {
                    stp_indx=6;
                   //stp = Vec::from((include_bytes!("../files/7.stp")).as_slice());
                    ui.close();
                };
                if ui.button("Demo8").clicked() {
                    stp_indx=7;
                    //stp = Vec::from((include_bytes!("../files/8.stp")).as_slice());
                    ui.close();
                };
                if ui.button("Demo9").clicked() {
                    stp_indx=8;
                    //stp = Vec::from((include_bytes!("../files/9.stp")).as_slice());
                    ui.close();
                };
                if ui.button("Demo10").clicked() {
                    stp_indx=9;
                    //stp = Vec::from((include_bytes!("../files/10.stp")).as_slice());
                    ui.close();
                };
                if ui.button("Demo11").clicked() {
                    stp_indx=10;
                    //stp = Vec::from((include_bytes!("../files/11.stp")).as_slice());
                    ui.close();
                };
                if ui.button("Demo12").clicked() {
                    stp_indx=11;
                    //stp = Vec::from((include_bytes!("../files/12.stp")).as_slice());
                    ui.close();
                };
                if ui.button("Demo13").clicked() {
                    stp_indx=12;
                    //stp = Vec::from((include_bytes!("../files/13.stp")).as_slice());
                    ui.close();
                };
                if ui.button("Demo14").clicked() {
                    stp_indx=13;
                    //stp = Vec::from((include_bytes!("../files/14.stp")).as_slice());
                    ui.close();
                };
                if ui.button("Demo15").clicked() {
                    stp_indx=14;
                    //stp = Vec::from((include_bytes!("../files/15.stp")).as_slice());
                    ui.close();
                };
                if ui.button("Demo16").clicked() {
                    stp_indx=15;
                    //stp = Vec::from((include_bytes!("../files/16.stp")).as_slice());
                    ui.close();
                };
                if ui.button("Demo17").clicked() {
                    stp_indx=16;
                    //stp = Vec::from((include_bytes!("../files/17.stp")).as_slice());
                    ui.close();
                };

                if(stp_indx!=50){
                    let stp=pipe_spec.demos[stp_indx].clone();
                    pipe_spec.init_pipe(&stp);
                    next_state.set(AppMode::Restarting);
                }
               });
            ui.separator();
            if ui.button(eye_icon).clicked() {

            }
            ui.separator();
            if ui.button(centerline_eye_icon).clicked() {

            }
            ui.separator();

        });
    });

}