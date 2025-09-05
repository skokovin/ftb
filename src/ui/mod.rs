use std::fs;
use std::fs::{File, OpenOptions};
use std::io::{BufWriter, Write};
use std::path::PathBuf;
use std::rc::Rc;
use std::str::FromStr;
use std::sync::Arc;
use std::sync::atomic::AtomicUsize;
use std::time::{SystemTime, UNIX_EPOCH};
use bevy::asset::{Assets, Handle};
use bevy::color::LinearRgba;
use bevy::log::warn;
use bevy::math::ops::round;
use bevy::math::Vec3;
use bevy::pbr::{MeshMaterial3d, StandardMaterial};
use bevy::prelude::{Mesh, Mesh3d, Resource};

use bevy_ecs::change_detection::{Res, ResMut};
use bevy_ecs::entity::Entity;
use bevy_ecs::prelude::{Commands, Query};
use bevy_editor_cam::prelude::{EditorCam, EnabledMotion};
use bevy_egui::{egui, EguiContexts};
use bevy_egui::egui::{Color32, TextStyle};
use cgmath::num_traits::abs;
use cgmath::{Deg, Rad, Vector3};
use chrono::NaiveDate;
use egui_alignments::center_vertical;
use rfd::FileDialog;
use regex::Regex;
use crate::{on_mouse_button_click, BendCommands, OccupiedScreenSpace, PipeCenterLine, SharedMaterials, VisibilityStore};
use crate::adds::line::{LineList, LineMaterial};
use crate::algo::cnc::{cnc_to_poly, cnc_to_poly_animate, reverse_lraclr, AnimState, AnimStatus, LRACLR};
use crate::algo::{analyze_stp, BendToro, MainCylinder, MainPipe};

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
        let mesh = c.to_mesh(16);
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
            //Disabled,
        )).observe(on_mouse_button_click).id();

        counter = counter + 1;
    }

    for c in circles {
        let v1 = Vec3::new(c.ca.loc.x as f32, c.ca.loc.y as f32, c.ca.loc.z as f32);
        let v2 = Vec3::new(c.cb.loc.x as f32, c.cb.loc.y as f32, c.cb.loc.z as f32);
        dxf_lines_c.push((v1, v2));
        let mesh = c.to_mesh(16);
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
        let mesh = c.to_mesh(16);
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
