use std::fs;
use std::fs::{File, OpenOptions};
use std::io::{BufWriter, Write};
use std::ops::{Add, Mul, Sub};
use std::path::PathBuf;
use std::rc::Rc;
use std::str::FromStr;
use std::sync::Arc;
use std::sync::atomic::AtomicUsize;
use std::time::{SystemTime, UNIX_EPOCH};
use bevy::asset::{Assets, Handle, RenderAssetUsages};
use bevy::color::LinearRgba;
use bevy::log::warn;
use bevy::math::ops::round;

use bevy::mesh::{Indices, PrimitiveTopology, VertexAttributeValues};
use bevy::pbr::{MeshMaterial3d, StandardMaterial};
use bevy::prelude::{Mesh, Mesh3d, Resource, Transform, Vec3};
use bevy::reflect::List;
use bevy_ecs::change_detection::{Res, ResMut};
use bevy_ecs::entity::Entity;
use bevy_ecs::name::Name;
use bevy_ecs::prelude::{Commands, Query};
use bevy_editor_cam::prelude::{EditorCam, EnabledMotion};
use bevy_egui::{egui, EguiContexts};
use bevy_egui::egui::{Color32, TextStyle};
use cgmath::num_traits::abs;
use cgmath::{Deg, InnerSpace, Point3, Rad, Vector3};
use chrono::NaiveDate;
use egui_alignments::center_vertical;
use is_odd::IsOdd;
use rfd::FileDialog;
use regex::Regex;
use crate::{on_mouse_button_click, BendCommands, MainPipe, OccupiedScreenSpace, PipeCenterLine, SharedMaterials, SimpleAnimation, VisibilityStore};
use crate::adds::line::{LineList, LineMaterial};
use crate::algo::cnc::{all_to_stp, cnc_to_poly, cnc_to_poly_animate, reverse_lraclr, tot_pipe_len, AnimState, AnimStatus, LRACLR};
use crate::algo::{analyze_stp, triangulate_cylinder, BendToro, MainCylinder};
use crate::algo::triangulation::triangulate_pipe;

pub static STRIGHT_SPEED: AtomicUsize = AtomicUsize::new(100);
pub static ROTATE_SPEED: AtomicUsize = AtomicUsize::new(50);
pub static ANGLE_SPEED: AtomicUsize = AtomicUsize::new(20);

#[derive(Resource)]
pub struct UiState {
    pub lrauis: Vec<LRAUI>,
    pub total_length: String,
    pub pipe_diameter: String,
}
impl UiState {
    pub fn update(&mut self, v: &Vec<LRACLR>) {
        let (len, pipe_d) = LRACLR::total_len_out_d(v);
        self.lrauis = vec![];
        self.total_length = ((len) as i32).to_string();
        self.pipe_diameter = ((pipe_d) as i32).to_string();
        v.iter().for_each(|lraclr| {
            let item = LRAUI::fromlra(lraclr);
            self.lrauis.push(item);
        })
    }

    pub fn to_lraclr(&self) -> Vec<LRACLR> {
        let mut v: Vec<LRACLR> = vec![];
        self.lrauis.iter().for_each(|lraui| {
            let lraclr = LRACLR {
                id1: lraui.id1,
                id2: lraui.id2,
                l: UiState::str_to_f64(&lraui.l),
                r: UiState::str_to_f64(&lraui.r),
                a: UiState::str_to_f64(&lraui.a),
                clr: UiState::str_to_f64(&lraui.clr),
                pipe_radius: UiState::str_to_f64(&lraui.pipe_radius),
            };
            v.push(lraclr);
        });
        v
    }

    fn str_to_f64(s: &str) -> f64 {
        let mut s = s.to_string();
        if s.ends_with('.') {
            s.push_str("0");
        }
        s.parse::<f64>().unwrap_or_else(|_| 1.0)
    }
}

pub struct LRAUI {
    pub id1: i32,
    pub id2: i32,
    pub l: String,
    pub r: String,
    pub a: String,
    pub bend_l: String,
    pub clr: String,
    pub pipe_radius: String,
}
impl LRAUI {
    pub fn fromlra(lraclr: &LRACLR) -> Self {
        Self {
            id1: lraclr.id1,
            id2: lraclr.id2,
            l: ((lraclr.l) as i32).to_string(),
            r: ((lraclr.r) as i32).to_string(),
            a: ((lraclr.a) as i32).to_string(),
            bend_l: (round((abs(Rad::from(Deg(lraclr.a)).0) * (lraclr.clr)) as f32) as i32).to_string(),
            clr: ((lraclr.clr) as i32).to_string(),
            pipe_radius: ((lraclr.pipe_radius) as i32).to_string(),
        }
    }
}

pub fn ui_system(mut contexts: EguiContexts,
                 mut occupied_screen_space: ResMut<OccupiedScreenSpace>,
                 mut commands: Commands,
                 mut ui_state: ResMut<UiState>,
                 mut d3_camera: Query<&mut EditorCam>,
                 mut visibility_store: ResMut<VisibilityStore>,
                 mut bend_commands: ResMut<BendCommands>,
                 mut query_pipes: Query<(&mut MeshMaterial3d<StandardMaterial>, &MainPipe)>,
                 mut query_meshes: Query<(Entity, &MainPipe)>,
                 mut query_centerlines: Query<(Entity, &PipeCenterLine)>,
                 shared_materials: Res<SharedMaterials>,
                 mut meshes: ResMut<Assets<Mesh>>,
                 mut query: Query<(&Name, &mut SimpleAnimation)>,
                 mut lines_materials: ResMut<Assets<LineMaterial>>,
) {
    let ctx = contexts.ctx_mut().expect("REASON");
    //let mut anim_key:f32=10.0;
    let mut is_cam_fixed = false;

    egui::TopBottomPanel::top("my_panel").show(ctx, |ui| {
        if (!is_cam_fixed) { is_cam_fixed = ui.rect_contains_pointer(ui.max_rect()); }
        ui.horizontal_wrapped(|ui| {
            if ui.button("File").clicked() {
                if (test_date()) {
                    if let Some(path) = rfd::FileDialog::new().add_filter("STEP", &["stp", "step"]).pick_file() {
                        match fs::read(path) {
                            Ok(stp) => {
                                for (entity, _) in &mut query_meshes {
                                    commands.entity(entity).despawn();
                                }
                                for (entity, _) in &mut query_centerlines {
                                    commands.entity(entity).despawn();
                                }
                                let lraclr_arr: Vec<LRACLR> = analyze_stp(&stp);
                                //let lraclr_arr: Vec<LRACLR> = convert_to_meter(&lraclr_arr_mm);
                                load_mesh(&lraclr_arr, &mut meshes, &mut commands, &shared_materials, &mut lines_materials, &mut ui_state, &mut bend_commands);
                                //bend_commands.straight = lraclr_arr;
                                bend_commands.original_file = stp;
                            }
                            Err(_) => {}
                        }
                    }
                }
            };
            ui.separator();
            if ui.button("Reverse").clicked() {
                for (entity, _) in &mut query_meshes {
                    commands.entity(entity).despawn();
                }
                for (entity, _) in &mut query_centerlines {
                    commands.entity(entity).despawn();
                }
                let lraclr_arr = reverse_lraclr(&bend_commands.straight);
                load_mesh(&lraclr_arr, &mut meshes, &mut commands, &shared_materials, &mut lines_materials, &mut ui_state, &mut bend_commands);
                //bend_commands.straight = lraclr_arr;
            };
            ui.separator();

            if ui.button("Simulate").clicked() {
                bend_commands.anim_state.status = AnimStatus::Enabled;
            };
            ui.separator();

            if ui.button("CSV").clicked() {
                if let Some(path) = FileDialog::new().add_filter("CSV", &["csv"]).set_directory("/").save_file() {
                    save_csv(&bend_commands.straight, &path);
                    println!("{:?}", path);
                }
            }
            ui.separator();
            ui.menu_button("Demos", |ui| {
                ui.style_mut().wrap_mode = Some(egui::TextWrapMode::Extend);
                let mut stp: Vec<u8> = vec![];
                if ui.button("Demo1").clicked() {
                    stp = Vec::from((include_bytes!("../files/1.stp")).as_slice());

                    ui.close_menu();
                };
                if ui.button("Demo2").clicked() {
                    stp = Vec::from((include_bytes!("../files/2.stp")).as_slice());
                    ui.close_menu();
                };
                if ui.button("Demo3").clicked() {
                    stp = Vec::from((include_bytes!("../files/3.stp")).as_slice());
                    ui.close_menu();
                };
                if ui.button("Demo4").clicked() {
                    stp = Vec::from((include_bytes!("../files/4.stp")).as_slice());
                    ui.close_menu();
                };
                if ui.button("Demo5").clicked() {
                    stp = Vec::from((include_bytes!("../files/5.stp")).as_slice());

                    ui.close_menu();
                };
                if ui.button("Demo6").clicked() {
                    stp = Vec::from((include_bytes!("../files/6.stp")).as_slice());
                    ui.close_menu();
                };
                if ui.button("Demo7").clicked() {
                    stp = Vec::from((include_bytes!("../files/7.stp")).as_slice());
                    ui.close_menu();
                };
                if ui.button("Demo8").clicked() {
                    stp = Vec::from((include_bytes!("../files/8.stp")).as_slice());
                    ui.close_menu();
                };
                if ui.button("Demo9").clicked() {
                    stp = Vec::from((include_bytes!("../files/9.stp")).as_slice());
                    ui.close_menu();
                };
                if ui.button("Demo10").clicked() {
                    stp = Vec::from((include_bytes!("../files/10.stp")).as_slice());
                    ui.close_menu();
                };
                if ui.button("Demo11").clicked() {
                    stp = Vec::from((include_bytes!("../files/11.stp")).as_slice());
                    ui.close_menu();
                };
                if ui.button("Demo12").clicked() {
                    stp = Vec::from((include_bytes!("../files/12.stp")).as_slice());
                    ui.close_menu();
                };
                if ui.button("Demo13").clicked() {
                    stp = Vec::from((include_bytes!("../files/13.stp")).as_slice());
                    ui.close_menu();
                };
                if ui.button("Demo14").clicked() {
                    stp = Vec::from((include_bytes!("../files/14.stp")).as_slice());
                    ui.close_menu();
                };
                if ui.button("Demo15").clicked() {
                    stp = Vec::from((include_bytes!("../files/15.stp")).as_slice());
                    ui.close_menu();
                };
                if ui.button("Demo16").clicked() {
                    stp = Vec::from((include_bytes!("../files/16.stp")).as_slice());
                    ui.close_menu();
                };
                if ui.button("Demo17").clicked() {
                    stp = Vec::from((include_bytes!("../files/17.stp")).as_slice());
                    ui.close_menu();
                };

                if (!stp.is_empty()) {
                    for (entity, _) in &mut query_meshes {
                        commands.entity(entity).despawn();
                    }
                    for (entity, _) in &mut query_centerlines {
                        commands.entity(entity).despawn();
                    }
                    let lraclr_arr: Vec<LRACLR> = analyze_stp(&stp);
                    //let lraclr_arr: Vec<LRACLR> = convert_to_meter(&lraclr_arr_mm);
                    load_mesh(&lraclr_arr, &mut meshes, &mut commands, &shared_materials, &mut lines_materials, &mut ui_state, &mut bend_commands);
                    //bend_commands.straight = lraclr_arr;
                    bend_commands.original_file = stp;
                }
            });
            ui.separator();

            if ui.button("A").clicked() {

                for (name, mut animation) in &mut query {
                    if (name.as_str() == "pens") {
                        animation.duration = 3.0;
                        animation.end_pos_x = 2000.0;
                    }
                }
            }
            ui.separator();

            if ui.button("B").clicked() {
              bend_commands.t=bend_commands.t-0.005;
            }
            ui.separator();


            let dorn_str = if (bend_commands.up_dir.z > 0.0) { "DORN R" } else { "DORN L" };

            if (ui.button(dorn_str)).clicked() {
                if (bend_commands.up_dir.z > 0.0) {
                    bend_commands.up_dir.z = -1.0;
                } else {
                    bend_commands.up_dir.z = 1.0;
                }

                for (entity, _) in &mut query_meshes {
                    commands.entity(entity).despawn();
                }
                for (entity, _) in &mut query_centerlines {
                    commands.entity(entity).despawn();
                }
                let lraclr_arr: Vec<LRACLR> = analyze_stp(&bend_commands.original_file);
                //let lraclr_arr: Vec<LRACLR> = convert_to_meter(&lraclr_arr_mm);
                load_mesh(&lraclr_arr, &mut meshes, &mut commands, &shared_materials, &mut lines_materials, &mut ui_state, &mut bend_commands);
                //bend_commands.straight = lraclr_arr;

            };
        });
    });


    occupied_screen_space.left = egui::SidePanel::left("left_panel").resizable(true).show(ctx, |ui| {
        if (!is_cam_fixed) { is_cam_fixed = ui.rect_contains_pointer(ui.max_rect()); }
        let col_width = 50.0;
        let col_heigth = 8.0;
        let mut new_diameter = f64::MAX;


        egui::ScrollArea::vertical().show(ui, |ui| {
            ui.horizontal(|ui| {

                // ui.with_layout(egui::Layout::left_to_right(egui::Align::Center), |ui| {

                let color = egui::Color32::from_rgb(255, 255, 255);
                let L = format!("Total len {} mm. ", ui_state.total_length);
                ui.label(egui::RichText::new(L).color(color));
                ui.label(egui::RichText::new("Pipe D= ").color(color));
                let radius_labl = ui.add(egui::TextEdit::singleline(&mut ui_state.pipe_diameter).text_color(color));
                if (radius_labl.changed() && is_signed_number(ui_state.pipe_diameter.as_str())) {
                    new_diameter = f64::from_str(ui_state.pipe_diameter.as_str()).unwrap();
                }
                // });

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
                             egui::Label::new(egui::RichText::new("Bend L").color(color)),
                );
                ui.separator();
                ui.add_sized([col_width, col_heigth],
                             egui::Label::new(egui::RichText::new("Bend R").color(color)),
                );
                ui.separator();
            });
            ui.separator();
            let mut seleced_id: i32 = {
                match bend_commands.anim_state.status {
                    AnimStatus::Enabled => {
                        bend_commands.anim_state.id
                    }
                    AnimStatus::Disabled => { i32::MAX }
                    AnimStatus::Finished => { i32::MAX }
                }
            };
            let mut opcode_id: i32 = {
                match bend_commands.anim_state.status {
                    AnimStatus::Enabled => {
                        println!("opcode_id: {}", bend_commands.anim_state.opcode as i32);
                        bend_commands.anim_state.opcode as i32
                    }
                    AnimStatus::Disabled => { i32::MAX }
                    AnimStatus::Finished => { i32::MAX }
                }
            };


            let mut is_changed = false;

            let mut counter: i32 = 0;
            let mut deleted_index: i32 = 0;
            let mut add_index: i32 = -1;
            let last_index = bend_commands.straight.len() as i32 - 1;

            for lra_item in &mut ui_state.lrauis {
                let color_white: Color32 = egui::Color32::from_rgb(255, 255, 255);
                let color_l: Color32 = {
                    match bend_commands.anim_state.status {
                        AnimStatus::Enabled => {
                            if (lra_item.id1 == bend_commands.selected_id && opcode_id == 0) {
                                egui::Color32::from_rgb(255, 0, 0)
                            } else {
                                egui::Color32::from_rgb(255, 255, 255)
                            }
                        }
                        AnimStatus::Disabled => {
                            if (lra_item.id1 == bend_commands.selected_id) {
                                egui::Color32::from_rgb(255, 0, 0)
                            } else {
                                egui::Color32::from_rgb(255, 255, 255)
                            }
                        }
                        AnimStatus::Finished => {
                            if (lra_item.id1 == bend_commands.selected_id) {
                                egui::Color32::from_rgb(255, 0, 0)
                            } else {
                                egui::Color32::from_rgb(255, 255, 255)
                            }
                        }
                    }
                };
                let color_r: Color32 = {
                    match bend_commands.anim_state.status {
                        AnimStatus::Enabled => {
                            if (lra_item.id1 == bend_commands.selected_id && opcode_id == 1) {
                                egui::Color32::from_rgb(255, 0, 0)
                            } else {
                                egui::Color32::from_rgb(255, 255, 255)
                            }
                        }
                        AnimStatus::Disabled => {
                            if (lra_item.id1 == bend_commands.selected_id) {
                                egui::Color32::from_rgb(255, 0, 0)
                            } else {
                                egui::Color32::from_rgb(255, 255, 255)
                            }
                        }
                        AnimStatus::Finished => {
                            if (lra_item.id1 == bend_commands.selected_id) {
                                egui::Color32::from_rgb(255, 0, 0)
                            } else {
                                egui::Color32::from_rgb(255, 255, 255)
                            }
                        }
                    }
                };
                let color_a: Color32 = {
                    match bend_commands.anim_state.status {
                        AnimStatus::Enabled => {
                            if (lra_item.id2 == bend_commands.selected_id && opcode_id == 2) {
                                egui::Color32::from_rgb(255, 0, 0)
                            } else {
                                egui::Color32::from_rgb(255, 255, 255)
                            }
                        }
                        AnimStatus::Disabled => {
                            if (lra_item.id2 == bend_commands.selected_id) {
                                egui::Color32::from_rgb(255, 0, 0)
                            } else {
                                egui::Color32::from_rgb(255, 255, 255)
                            }
                        }
                        AnimStatus::Finished => {
                            if (lra_item.id2 == bend_commands.selected_id) {
                                egui::Color32::from_rgb(255, 0, 0)
                            } else {
                                egui::Color32::from_rgb(255, 255, 255)
                            }
                        }
                    }
                };


                ui.horizontal(|ui| {
                    let l_labl = ui.add_sized([col_width, ui.available_height()], egui::TextEdit::singleline(&mut lra_item.l).text_color(color_l));

                    if (l_labl.clicked()) {
                        query_pipes.iter_mut().for_each(|(mut m, pipe)| {
                            match pipe {
                                MainPipe::Pipe(pipe) => {
                                    if (lra_item.id1 == pipe.id as i32) {
                                        seleced_id = lra_item.id1;
                                        m.0 = shared_materials.pressed_matl.clone();
                                    } else {
                                        m.0 = shared_materials.white_matl.clone();
                                    }
                                }
                                MainPipe::Tor(tor) => {
                                    m.0 = shared_materials.red_matl.clone();
                                }
                            };
                        });
                    }

                    if (l_labl.changed() && is_signed_number(lra_item.l.as_str())) {
                        is_changed = true;
                    }

                    ui.separator();
                    let r_labl = ui.add_sized([col_width, ui.available_height()], egui::TextEdit::singleline(&mut lra_item.r).text_color(color_r));

                    if (r_labl.changed() && is_signed_number(lra_item.r.as_str())) {
                        is_changed = true;
                    }

                    ui.separator();
                    let a_labl = ui.add_sized([col_width, ui.available_height()], egui::TextEdit::singleline(&mut lra_item.a).text_color(color_a));

                    if (a_labl.clicked()) {
                        query_pipes.iter_mut().for_each(|(mut m, pipe)| {
                            match pipe {
                                MainPipe::Pipe(pipe) => {
                                    m.0 = shared_materials.white_matl.clone();
                                }
                                MainPipe::Tor(tor) => {
                                    if (lra_item.id2 == tor.id as i32) {
                                        seleced_id = lra_item.id2;
                                        m.0 = shared_materials.pressed_matl.clone();
                                    } else {
                                        m.0 = shared_materials.red_matl.clone();
                                    }
                                }
                            };
                        });
                    }

                    if (a_labl.changed() && is_signed_number(lra_item.a.as_str())) {
                        is_changed = true;
                    }

                    ui.separator();
                    ui.add_sized([col_width, ui.available_height()], egui::Label::new(egui::RichText::new(&lra_item.bend_l).color(color_white)));
                    ui.separator();

                    let clr_labl = ui.add_sized([col_width, ui.available_height()], egui::TextEdit::singleline(&mut lra_item.clr).text_color(color_white));
                    if (clr_labl.changed() && is_signed_number(lra_item.clr.as_str())) {
                        is_changed = true;
                    }

                    ui.separator();

                    let is_add_button_disabled = if (counter != last_index) { true } else { false };
                    let add_button = ui.add_enabled(is_add_button_disabled, egui::Button::new("+"));
                    if (add_button.clicked()) {
                        add_index = counter;
                    }

                    let is_delete_button_disabled = if (counter != 0 && last_index >= 2 && counter != last_index)
                    { true } else { false };
                    let delete_button = ui.add_enabled(is_delete_button_disabled, egui::Button::new("x"));
                    if (delete_button.clicked()) {
                        deleted_index = counter;
                    }
                });

                counter = counter + 1;
            }

            if (is_changed) {
                for (entity, _) in &mut query_meshes {
                    commands.entity(entity).despawn();
                }
                for (entity, _) in &mut query_centerlines {
                    commands.entity(entity).despawn();
                }
                let lraclr_arr: Vec<LRACLR> = ui_state.to_lraclr().clone();
                //let lraclr_arr: Vec<LRACLR> = convert_to_meter(&lraclr_arr_mm);
                load_mesh(&lraclr_arr, &mut meshes, &mut commands, &shared_materials, &mut lines_materials, &mut ui_state, &mut bend_commands);
            }

            if (deleted_index != 0) {
                for (entity, _) in &mut query_meshes {
                    commands.entity(entity).despawn();
                }
                for (entity, _) in &mut query_centerlines {
                    commands.entity(entity).despawn();
                }
                let new_commands = delete_lra_row(deleted_index, &bend_commands.straight);
                ui_state.update(&new_commands);
                load_mesh(&new_commands, &mut meshes, &mut commands, &shared_materials, &mut lines_materials, &mut ui_state, &mut bend_commands);
                bend_commands.straight = new_commands;
            }

            if (add_index != -1) {
                for (entity, _) in &mut query_meshes {
                    commands.entity(entity).despawn();
                }
                for (entity, _) in &mut query_centerlines {
                    commands.entity(entity).despawn();
                }
                let new_commands = add_lra_row(deleted_index, &bend_commands.straight);
                ui_state.update(&new_commands);
                load_mesh(&new_commands, &mut meshes, &mut commands, &shared_materials, &mut lines_materials, &mut ui_state, &mut bend_commands);
                bend_commands.straight = new_commands;
            }

            if (seleced_id != i32::MAX) {
                bend_commands.selected_id = seleced_id;
            }
        });
        if (new_diameter < f64::MAX) {
            let mut new_lra = bend_commands.straight.clone();
            new_lra.iter_mut().for_each(|lra| {
                lra.pipe_radius = new_diameter;
            });
            for (entity, _) in &mut query_meshes {
                commands.entity(entity).despawn();
            }
            for (entity, _) in &mut query_centerlines {
                commands.entity(entity).despawn();
            }

            ui_state.update(&new_lra);
            load_mesh(&new_lra, &mut meshes, &mut commands, &shared_materials, &mut lines_materials, &mut ui_state, &mut bend_commands);
            bend_commands.straight = new_lra;
        }
        ui.allocate_rect(ui.available_rect_before_wrap(), egui::Sense::hover());
    }).response.rect.width();


    if (is_cam_fixed) {
        let mut c = d3_camera.single_mut().unwrap();
        c.enabled_motion = EnabledMotion {
            pan: false,
            orbit: false,
            zoom: false,
        };
    } else {
        let mut c = d3_camera.single_mut().unwrap();
        c.enabled_motion = EnabledMotion {
            pan: true,
            orbit: true,
            zoom: true,
        };
    };
}
pub fn reload_mesh(meshes: &mut ResMut<Assets<Mesh>>, commands: &mut Commands, shared_materials: &Res<SharedMaterials>, lines_materials: &mut ResMut<Assets<LineMaterial>>, ui_state: &mut UiState, bend_commands: &mut BendCommands) {
    //let mut the_up_dir: Vector3<f64> = Vector3::new(0., 0., 1.);
    bend_commands.straight = reverse_lraclr(&bend_commands.straight);
    ui_state.update(&bend_commands.straight);
    let mut dxf_lines_c: Vec<(Vec3, Vec3)> = vec![];
    let mut dxf_lines_t: Vec<(Vec3, Vec3)> = vec![];
    let (circles, tors) = cnc_to_poly(&bend_commands.straight, &bend_commands.up_dir);
    let mut counter = 0;
    for t in tors {
        let prev_dir = circles[counter].get_dir();
        let lines = t.to_lines(&prev_dir);
        dxf_lines_t.extend_from_slice(&lines);

        let mesh = t.to_mesh(&prev_dir, 8, 8);
        let handle: Handle<Mesh> = meshes.add(mesh);
        let entity: Entity = commands.spawn((
            Mesh3d(handle),
            MeshMaterial3d(shared_materials.red_matl.clone()),
            MainPipe::Tor(t),
            //Disabled,
        )).observe(on_mouse_button_click).id();

        counter = counter + 1;
    }

    for c in circles {
        let v1 = Vec3::new(c.ca.loc.x as f32, c.ca.loc.y as f32, c.ca.loc.z as f32);
        let v2 = Vec3::new(c.cb.loc.x as f32, c.cb.loc.y as f32, c.cb.loc.z as f32);
        dxf_lines_c.push((v1, v2));
        let mesh = c.to_mesh();
        let handle: Handle<Mesh> = meshes.add(mesh);
        let entity: Entity = commands.spawn((
            Mesh3d(handle),
            MeshMaterial3d(shared_materials.white_matl.clone()),
            MainPipe::Pipe(c),
            //Disabled,
        )).observe(on_mouse_button_click).id();
    };


    commands.spawn((
        Mesh3d(meshes.add(LineList {
            lines: dxf_lines_c,
        })),
        MeshMaterial3d(lines_materials.add(LineMaterial {
            color: LinearRgba::WHITE,
        })),
        PipeCenterLine
    ));
    commands.spawn((
        Mesh3d(meshes.add(LineList {
            lines: dxf_lines_t,
        })),
        MeshMaterial3d(lines_materials.add(LineMaterial {
            color: LinearRgba::RED,
        })),
        PipeCenterLine
    ));
}
pub fn load_mesh(lraclr_arr: &Vec<LRACLR>, meshes: &mut ResMut<Assets<Mesh>>, commands: &mut Commands, shared_materials: &Res<SharedMaterials>, lines_materials: &mut ResMut<Assets<LineMaterial>>, ui_state: &mut UiState, bend_commands: &mut BendCommands) {
    //let mut the_up_dir: Vector3<f64> = Vector3::new(0., 0., 1.);

    ui_state.update(lraclr_arr);
    let mut dxf_lines_c: Vec<(Vec3, Vec3)> = vec![];
    let mut dxf_lines_t: Vec<(Vec3, Vec3)> = vec![];

    //    let stp: Vec<u8> = Vec::from((include_bytes!("files/9.stp")).as_slice());
    //let lraclr_arr_mm: Vec<LRACLR> = analyze_stp(&stp);
    //let lraclr_arr: Vec<LRACLR> = convert_to_meter(&lraclr_arr_mm);
    let (circles, tors) = cnc_to_poly(&lraclr_arr, &bend_commands.up_dir);


    let mut counter = 0;
    for t in tors {
        let prev_dir = circles[counter].get_dir();
        let lines = t.to_lines(&prev_dir);
        dxf_lines_t.extend_from_slice(&lines);

        let mesh = t.to_mesh(&prev_dir, 8, 8);
        let handle: Handle<Mesh> = meshes.add(mesh);
        let entity: Entity = commands.spawn((
            Mesh3d(handle),
            MeshMaterial3d(shared_materials.red_matl.clone()),
            MainPipe::Tor(t),
            Transform::default(),
        )).observe(on_mouse_button_click).id();

        counter = counter + 1;
    }

  for c in circles {
        let v1 = Vec3::new(c.ca.loc.x as f32, c.ca.loc.y as f32, c.ca.loc.z as f32);
        let v2 = Vec3::new(c.cb.loc.x as f32, c.cb.loc.y as f32, c.cb.loc.z as f32);
        dxf_lines_c.push((v1, v2));
        let mesh = c.to_mesh();
        let handle: Handle<Mesh> = meshes.add(mesh);
        let entity: Entity = commands.spawn((
            Mesh3d(handle),
            MeshMaterial3d(shared_materials.white_matl.clone()),
            MainPipe::Pipe(c),
            Transform::default(),
        )).observe(on_mouse_button_click).id();
    };


/*    commands.spawn((
        Mesh3d(meshes.add(LineList {
            lines: dxf_lines_c,
        })),
        MeshMaterial3d(lines_materials.add(LineMaterial {
            color: LinearRgba::WHITE,
        })),
        PipeCenterLine
    ));*/
/*    commands.spawn((
        Mesh3d(meshes.add(LineList {
            lines: dxf_lines_t,
        })),
        MeshMaterial3d(lines_materials.add(LineMaterial {
            color: LinearRgba::RED,
        })),
        PipeCenterLine
    ));*/
    bend_commands.straight = lraclr_arr.clone();
    bend_commands.anim_state = AnimState::default();
}
pub fn load_mesh_by_t(lraclr_arr: &Vec<LRACLR>, meshes: &mut ResMut<Assets<Mesh>>, commands: &mut Commands, shared_materials: &Res<SharedMaterials>, lines_materials: &mut ResMut<Assets<LineMaterial>>, ui_state: &mut UiState, bend_commands: &mut BendCommands) {
    //let mut the_up_dir: Vector3<f64> = Vector3::new(0., 0., 1.);

    ui_state.update(lraclr_arr);
    //let mut dxf_lines_c: Vec<(Vec3, Vec3)> = vec![];
    //let mut dxf_lines_t: Vec<(Vec3, Vec3)> = vec![];
    let segment_len=tot_pipe_len(lraclr_arr)/1000.0;

    //    let stp: Vec<u8> = Vec::from((include_bytes!("files/9.stp")).as_slice());
    //let lraclr_arr_mm: Vec<LRACLR> = analyze_stp(&stp);
    //let lraclr_arr: Vec<LRACLR> = convert_to_meter(&lraclr_arr_mm);
    let (circles, tors) = cnc_to_poly(&lraclr_arr, &bend_commands.up_dir);

    let id_size:u64= (circles.len() + tors.len()) as u64;

    let mut t=0;


    for id in 0.. id_size {

        if(id.is_odd()){
            //tor

            let mut counter = 0;

            for tor in &tors {
                if(tor.id==id){
                    let prev_dir = &circles[counter].get_dir();
                    // let lines = t.to_lines(&prev_dir);
                    // dxf_lines_t.extend_from_slice(&lines);

                    let mesh = tor.to_mesh_with_seg_len(&prev_dir, 8, 8,segment_len);
                    let handle: Handle<Mesh> = meshes.add(mesh);
                    let entity: Entity = commands.spawn((
                        Mesh3d(handle),
                        MeshMaterial3d(shared_materials.red_matl.clone()),
                        MainPipe::Tor(tor.clone()),
                        Transform::default(),
                    )).observe(on_mouse_button_click).id();
                    counter = counter + 1;
                }

            }



        }  else{
            for c in &circles {
                if(c.id==id){

                    let meshes_s: Vec<Mesh> = c.clone().to_mesh_with_seg_len(segment_len);
                    for m in meshes_s {
                        let mut c_m = c.clone();
                        c_m.t=t;
                        t=t+1;
                        let handle: Handle<Mesh> = meshes.add(m);
                        let entity: Entity = commands.spawn((
                            Mesh3d(handle),
                            MeshMaterial3d(shared_materials.white_matl.clone()),
                            MainPipe::Pipe(c_m.clone()),
                            Transform::default(),
                        )).observe(on_mouse_button_click).id();
                    }

                }
            };
        }
        println!("Processing id: {:?}", id);
    }


    bend_commands.straight = lraclr_arr.clone();
    bend_commands.anim_state = AnimState::default();
}

pub fn interpolate_by_tA(cmnd: &Vec<LRACLR>, up_dir: &Vector3<f64>) -> Vec<(Mesh, i64)> {
    let step:i64=1000;
    let len: f64 = tot_pipe_len(cmnd);
    println!("len: {:?}", len);
    let r=cmnd[0].pipe_radius as f32;
    let mut ret:Vec<(Mesh,i64)> = vec![];

    let mut step_len=len/step as f64;
    let mut fragment_len=0.0;
    let mut id:u64 = 0;
    let (circles, tors) = cnc_to_poly(cmnd, up_dir);
    let indexes=&circles.len() +&tors.len();
    let mut x_dir=Vector3::new(1.0,0.0,0.0);
    let mut y_dir=Vector3::new(0.0,1.0,0.0);
    let mut z_dir=Vector3::new(0.0,0.0,1.0);
    let mut pt_a: Point3<f64>=Point3::new(0.0,0.0,0.0);
    let mut pt: Point3<f64>=Point3::new(0.0,0.0,0.0);
    let mut rot_deg=0.0;
    let mut dist =0.0;

    for t in 1..step{
        dist = dist+step_len;
        for i in (0..indexes).step_by(2) {
            match circles.iter().find(|cil|cil.id==i as u64) {
                None => { println!("Index NF{}",i);}
                Some(c) => {
                    let  min=fragment_len;
                    fragment_len=fragment_len+c.h;
                    let  max=fragment_len;
                    if(dist>=min && dist<max){
                        id=c.id;
                        rot_deg=LRACLR::rotate_by_id(c.id as i32, cmnd);
                        let offset=dist-min;
                        pt =c.ca.loc+ c.ca.dir*offset;
                        let pt2: Point3<f64> =c.ca.loc+ c.ca.dir*(offset+1.0);
                        x_dir=pt2.sub(pt);
                        match  tors.iter().find(|tor| tor.id==(i+1) as u64){
                            None => {
                                match  tors.iter().find(|tor| tor.id==(i-1) as u64) {
                                    None => {}
                                    Some(arc) => {
                                        let pt2=pt+arc.bend_plane_norm.normalize();
                                        z_dir=pt.sub(pt2);
                                        y_dir=z_dir.cross(x_dir);
                                        //rot_deg= LRACLR::rotate_by_id(arc.id as i32, cmnd);
                                    }
                                }

                            }
                            Some(arc) => {
                                let pt2=pt+arc.bend_plane_norm.normalize();
                                z_dir=pt.sub(pt2);
                                y_dir=z_dir.cross(x_dir);
                                //rot_deg=LRACLR::rotate_by_id(arc.id as i32, cmnd);
                            }
                        }
                    }else{
                        match  tors.iter().find(|tor| tor.id==(i+1) as u64) {
                            None => { }
                            Some(arc) => {
                                let min=fragment_len;
                                fragment_len=fragment_len+arc.get_len();
                                let max=fragment_len;
                                if(dist>=min && dist<max){
                                    id=arc.id;
                                    let offset=dist-min;
                                    let v1: Vector3<f64> = arc.ca.loc.sub(arc.bend_center_point);
                                    let v2: Vector3<f64> = arc.cb.loc.sub(arc.bend_center_point);
                                    let total_angle: f64 = (v1.dot(v2) / (v1.magnitude() * v2.magnitude())).acos();
                                    let total_arc_length: f64 = total_angle * arc.bend_radius;
                                    let theta = offset / arc.bend_radius;
                                    let axis = v1.cross(v2);
                                    let normalized_axis = axis.normalize();
                                    let v_target = v1 * theta.cos() + normalized_axis.cross(v1) * theta.sin();
                                    pt = arc.bend_center_point.add(v_target);
                                    //println!("ImHere T {} D {} min {} max {} Point {:?}", arc.id,dist,min,max,pt);
                                    let pt2=pt+arc.bend_plane_norm.normalize();
                                    let v3=pt.sub(arc.bend_center_point).normalize();
                                    z_dir=pt.sub(pt2);
                                    y_dir=(pt+v3).sub(pt);
                                    x_dir=y_dir.cross(z_dir);
                                    //rot_deg=LRACLR::rotate_by_id(arc.id as i32, cmnd);
                                }

                                //println!("L A {}",fragment_len);
                                //rintln!("Index {} C {}  T {}", i, c.id, arc.id);
                            }
                        }
                    }
                }
            };
        }

        let mesh=triangulate_cylinder(&pt_a, &pt,r,8);
        ret.push((mesh,t));
        println!("L A {:?} {:?} {:?} {:?}",t,dist,pt_a,pt);

        pt_a= pt;

    }
    ret
}

pub fn interpolate_by_t(cmnd: &Vec<LRACLR>, up_dir: &Vector3<f64>) -> (Vec<(Mesh, i64, u64)>, Vec<(Mesh, i64, u64)>) {
    let num_segments=64;
    let len: f64 = tot_pipe_len(cmnd);
    let pipe_radius=cmnd[0].pipe_radius as f32;
    let mut ret:Vec<(Mesh,i64,u64)> = vec![];
    let mut ret2:Vec<(Mesh,i64,u64)> = vec![];
    let step:i64=1000;
    let mut pt_a: Point3<f64>=Point3::new(0.0,0.0,0.0);
    let mut pt_m_a: Point3<f64>=Point3::new(0.0,0.0,0.0);
    for ti in 1..step{
       let t=ti as f64/step as f64;

       let( (pt,x_dir,y_dir,z_dir,rot_deg,id,cp,l)) =byt(t,cmnd,up_dir);
       let mesh=triangulate_pipe(&pt_a, &pt,&cp, pipe_radius, num_segments);
       ret.push((mesh,ti,id));
        pt_a=pt;
       let pt_m_b: Point3<f64>=Point3::new(-len*t,0.0,0.0);
       let mesh2=triangulate_pipe(&pt_m_a, &pt_m_b,&None, pipe_radius, num_segments);
       ret2.push((mesh2,ti,99999));
        pt_m_a= pt_m_b;
    }
    (ret,ret2)
}

pub fn byt(t: f64, cmnd: &Vec<LRACLR>, up_dir: &Vector3<f64>) -> (Point3<f64>, Vector3<f64>, Vector3<f64>, Vector3<f64>, f64, u64, Option<Point3<f32>>, f64) {
    let len: f64 = tot_pipe_len(cmnd);
    let dist = len * t;
    let mut id:u64 = 0;
    let mut fragment_len=0.0;
    //println!("LENGHT {} dist {} fragment_len {}",len,dist,fragment_len);
    let (circles, tors) = cnc_to_poly(cmnd, up_dir);
    let indexes=&circles.len() +&tors.len();
    let mut x_dir=Vector3::new(1.0,0.0,0.0);
    let mut y_dir=Vector3::new(0.0,1.0,0.0);
    let mut z_dir=Vector3::new(0.0,0.0,1.0);
    let mut pt: Point3<f64>=Point3::new(0.0,0.0,0.0);
    let mut rot_deg=0.0;
    let mut cp: Option<Point3<f32>>=None;

    for i in (0..indexes).step_by(2) {
        match circles.iter().find(|cil|cil.id==i as u64) {
            None => { println!("Index NF{}",i);}
            Some(c) => {
                let  min=fragment_len;
                fragment_len=fragment_len+c.h;
                let  max=fragment_len;
                if(dist>=min && dist<max)
                {
                    id=c.id;
                    cp=None;
                    rot_deg=LRACLR::rotate_by_id(c.id as i32, cmnd);
                    let offset=dist-min;
                    pt =c.ca.loc+ c.ca.dir*offset;
                    let pt2: Point3<f64> =c.ca.loc+ c.ca.dir*(offset+1.0);
                    x_dir=pt2.sub(pt);
                    match  tors.iter().find(|tor| tor.id==(i+1) as u64){
                        None => {
                            match  tors.iter().find(|tor| tor.id==(i-1) as u64) {
                                None => {}
                                Some(arc) => {
                                    let pt2=pt+arc.bend_plane_norm.normalize();
                                    z_dir=pt.sub(pt2);
                                    y_dir=z_dir.cross(x_dir);
                                    //rot_deg= LRACLR::rotate_by_id(arc.id as i32, cmnd);
                                }
                            }

                        }
                        Some(arc) => {
                            let pt2=pt+arc.bend_plane_norm.normalize();
                            z_dir=pt.sub(pt2);
                            y_dir=z_dir.cross(x_dir);
                            //rot_deg=LRACLR::rotate_by_id(arc.id as i32, cmnd);
                        }
                    }
                }
                else{
                    match  tors.iter().find(|tor| tor.id==(i+1) as u64) {
                        None => { }
                        Some(arc) => {
                            let min=fragment_len;
                            fragment_len=fragment_len+arc.get_len();
                            let max=fragment_len;
                            if(dist>=min && dist<max){
                                id=arc.id;
                                cp=Some(Point3::new(arc.bend_center_point.x as f32, arc.bend_center_point.y as f32, arc.bend_center_point.z as f32));
                                let offset=dist-min;
                                let v1: Vector3<f64> = arc.ca.loc.sub(arc.bend_center_point);
                                let v2: Vector3<f64> = arc.cb.loc.sub(arc.bend_center_point);
                                let total_angle: f64 = (v1.dot(v2) / (v1.magnitude() * v2.magnitude())).acos();
                                let total_arc_length: f64 = total_angle * arc.bend_radius;
                                let theta = offset / arc.bend_radius;
                                let axis = v1.cross(v2);
                                let normalized_axis = axis.normalize();
                                let v_target = v1 * theta.cos() + normalized_axis.cross(v1) * theta.sin();
                                pt = arc.bend_center_point.add(v_target);
                                //println!("ImHere T {} D {} min {} max {} Point {:?}", arc.id,dist,min,max,pt);
                                let pt2=pt+arc.bend_plane_norm.normalize();
                                let v3=pt.sub(arc.bend_center_point).normalize();
                                z_dir=pt.sub(pt2);
                                y_dir=(pt+v3).sub(pt);
                                x_dir=y_dir.cross(z_dir);
                            }
                        }
                    }
                }
            }
        };
    }

    (pt,x_dir,y_dir,z_dir,rot_deg,id,cp,len)

}

pub fn load_mesh_centerline(lraclr_arr: &Vec<LRACLR>, meshes: &mut ResMut<Assets<Mesh>>, commands: &mut Commands, shared_materials: &Res<SharedMaterials>, lines_materials: &mut ResMut<Assets<LineMaterial>>, ui_state: &mut UiState, bend_commands: &mut BendCommands) {
    //let mut the_up_dir: Vector3<f64> = Vector3::new(0., 0., 1.);

    ui_state.update(lraclr_arr);
    let mut dxf_lines_c: Vec<(Vec3, Vec3)> = vec![];
    let mut dxf_lines_t: Vec<(Vec3, Vec3)> = vec![];

    let (circles, tors) = cnc_to_poly(&lraclr_arr, &bend_commands.up_dir);

    let data: Vec<u8> =all_to_stp(&circles, &tors);
    fs::write("d:\\2\\output.stp", &data);

    let mut counter = 0;
    for t in tors {
        let prev_dir = circles[counter].get_dir();
        let lines = t.to_lines(&prev_dir);
        dxf_lines_t.extend_from_slice(&lines);
        counter = counter + 1;
    }

    for c in circles {
        let v1 = Vec3::new(c.ca.loc.x as f32, c.ca.loc.y as f32, c.ca.loc.z as f32);
        let v2 = Vec3::new(c.cb.loc.x as f32, c.cb.loc.y as f32, c.cb.loc.z as f32);
        dxf_lines_c.push((v1, v2));
    };


    commands.spawn((
        Mesh3d(meshes.add(LineList {
            lines: dxf_lines_c,
        })),
        MeshMaterial3d(lines_materials.add(LineMaterial {
            color: LinearRgba::rgb(1.0,1.0,0.0),
        })),
        Transform::default(),
        PipeCenterLine
    ));
    commands.spawn((
        Mesh3d(meshes.add(LineList {
            lines: dxf_lines_t,
        })),
        MeshMaterial3d(lines_materials.add(LineMaterial {
            color: LinearRgba::rgb(1.0,0.3,0.3),
        })),
        Transform::default(),
        PipeCenterLine
    ));
    bend_commands.straight = lraclr_arr.clone();
    bend_commands.anim_state = AnimState::default();
}

pub fn load_anim_mesh(circles: Vec<MainCylinder>, tors: Vec<BendToro>, meshes: &mut ResMut<Assets<Mesh>>, commands: &mut Commands, shared_materials: &Res<SharedMaterials>, lines_materials: &mut ResMut<Assets<LineMaterial>>) {
    let mut dxf_lines_c: Vec<(Vec3, Vec3)> = vec![];
    let mut dxf_lines_t: Vec<(Vec3, Vec3)> = vec![];
    let mut counter = 0;
    for t in tors {
        let prev_dir = circles[counter].get_dir();
        let lines = t.to_lines(&prev_dir);
        dxf_lines_t.extend_from_slice(&lines);

        let mesh = t.to_mesh(&prev_dir, 8, 8);
        let handle: Handle<Mesh> = meshes.add(mesh);
        let entity: Entity = commands.spawn((
            Mesh3d(handle),
            MeshMaterial3d(shared_materials.red_matl.clone()),
            MainPipe::Tor(t),
            //Disabled,
        )).observe(on_mouse_button_click).id();

        counter = counter + 1;
    }

    for c in circles {
        let v1 = Vec3::new(c.ca.loc.x as f32, c.ca.loc.y as f32, c.ca.loc.z as f32);
        let v2 = Vec3::new(c.cb.loc.x as f32, c.cb.loc.y as f32, c.cb.loc.z as f32);
        dxf_lines_c.push((v1, v2));
        let mesh = c.to_mesh();
        let handle: Handle<Mesh> = meshes.add(mesh);
        let entity: Entity = commands.spawn((
            Mesh3d(handle),
            MeshMaterial3d(shared_materials.white_matl.clone()),
            MainPipe::Pipe(c),
            //Disabled,
        )).observe(on_mouse_button_click).id();
    };


    commands.spawn((
        Mesh3d(meshes.add(LineList {
            lines: dxf_lines_c,
        })),
        MeshMaterial3d(lines_materials.add(LineMaterial {
            color: LinearRgba::WHITE,
        })),
        PipeCenterLine
    ));
    commands.spawn((
        Mesh3d(meshes.add(LineList {
            lines: dxf_lines_t,
        })),
        MeshMaterial3d(lines_materials.add(LineMaterial {
            color: LinearRgba::RED,
        })),
        PipeCenterLine
    ));
}


fn delete_lra_row(row_index: i32, lraclr: &Vec<LRACLR>) -> Vec<LRACLR> {
    let mut v: Vec<LRACLR> = vec![];
    let mut counter = 0;
    lraclr.iter().for_each(|lra| {
        counter = counter + 1;
        if (counter != row_index) {
            v.push(lra.clone());
        }
    });
    v
}
fn add_lra_row(row_index: i32, lraclr: &Vec<LRACLR>) -> Vec<LRACLR> {
    let mut v: Vec<LRACLR> = vec![];
    let mut counter = 0;
    let mut curr_id1 = 0;
    lraclr.iter().for_each(|lra| {
        if (counter == row_index) {
            let mut lra1 = lra.clone();
            lra1.id1 = curr_id1;
            lra1.id2 = lra1.id1 + 1;
            curr_id1 = lra1.id2 + 1;
            curr_id1 = curr_id1 + 1;
            v.push(lra1);

            let mut lra2 = lra.clone();
            lra2.id1 = curr_id1;
            lra2.id2 = lra2.id1 + 1;
            curr_id1 = lra2.id2 + 1;
            curr_id1 = curr_id1 + 1;
            v.push(lra2);
        } else {
            let mut lra1 = lra.clone();
            lra1.id1 = curr_id1;
            lra1.id2 = lra1.id1 + 1;
            curr_id1 = lra1.id2 + 1;
            v.push(lra1);
            curr_id1 = curr_id1 + 1;
        }
        counter = counter + 1;
    });
    v
}


pub fn save_csv(lraclr_arr: &Vec<LRACLR>, path: &PathBuf) {
    let mut s_out = String::new();
    if (!lraclr_arr.is_empty()) {
        for i in 0..lraclr_arr.len() - 1 {
            let lraclr = lraclr_arr[i].clone();
            s_out.push_str(format!("{}{}", i, ";").as_str());
            s_out.push_str(format!("{}{}", lraclr.l, ";").as_str());
            s_out.push_str(format!("{}{}", lraclr.r, ";").as_str());
            s_out.push_str(format!("{}{}", lraclr.a, ";").as_str());
            s_out.push_str(format!("{}{}", lraclr.clr, ";").as_str());
            s_out.push_str("0\r\n");
        }
        let last = lraclr_arr.last().unwrap().clone();
        s_out.push_str(format!("{}{}", lraclr_arr.len() - 1, ";").as_str());
        s_out.push_str(format!("{}{}", last.l, ";").as_str());
        s_out.push_str("\r\n");
        //let mut d="C:\\tmp\\".to_string();
        //let mut d = "".to_string();
        //d.push_str(SystemTime::now().duration_since(UNIX_EPOCH).unwrap().as_secs_f64().to_string().as_str());
        //d.push_str(".csv");

        let f = OpenOptions::new().create(true).append(true).open(path).expect("Unable to open file");
        let mut f = BufWriter::new(f);
        f.write_all(s_out.as_bytes()).expect("Unable to write data");
    }
}

pub fn test_date() -> bool {
    let today = chrono::Utc::now().date_naive();
    let date2 = NaiveDate::from_ymd_opt(2026, 1, 20).unwrap();
    today < date2
}

fn is_signed_number(s: &str) -> bool {
    let re = Regex::new(r"^[-+]?\d+$").unwrap();
    re.is_match(s)
}






