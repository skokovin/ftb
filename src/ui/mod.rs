use std::fs;
use std::fs::File;
use bevy::asset::{Assets, Handle};
use bevy::color::LinearRgba;
use bevy::math::ops::round;
use bevy::math::Vec3;
use bevy::pbr::{MeshMaterial3d, StandardMaterial};
use bevy::prelude::{Mesh, Mesh3d};
use bevy_ecs::change_detection::{Res, ResMut};
use bevy_ecs::entity::Entity;
use bevy_ecs::prelude::{Commands, Query};
use bevy_editor_cam::prelude::{EditorCam, EnabledMotion};
use bevy_egui::{egui, EguiContexts};
use bevy_egui::egui::TextStyle;
use cgmath::num_traits::abs;
use cgmath::{Deg, Rad, Vector3};
use crate::{on_mouse_button_click, BendCommands, OccupiedScreenSpace, PipeCenterLine, SharedMaterials, VisibilityStore};
use crate::adds::line::{LineList, LineMaterial};
use crate::algo::cnc::{cnc_to_poly, reverse_lraclr, LRACLR};
use crate::algo::{analyze_stp, convert_to_meter, MainPipe};

pub mod app_settings;

pub fn ui_system(mut contexts: EguiContexts,
                 mut occupied_screen_space: ResMut<OccupiedScreenSpace>,
                 mut commands: Commands,
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
    egui::TopBottomPanel::top("my_panel").show(ctx, |ui| {
        ui.horizontal_wrapped(|ui| {
            if ui.button("File").clicked() {
                if let Some(path) = rfd::FileDialog::new().add_filter("STEP", &["stp", "step"]).pick_file() {
                    match fs::read(path) {
                        Ok(stp) => {
                            for (entity,_) in &mut query_meshes {
                                commands.entity(entity).despawn();
                            }
                            for (entity,_) in &mut query_centerlines {
                                commands.entity(entity).despawn();
                            }
                            let lraclr_arr_mm: Vec<LRACLR> = analyze_stp(&stp);
                            let lraclr_arr: Vec<LRACLR> = convert_to_meter(&lraclr_arr_mm);
                            load_mesh(&lraclr_arr, &mut meshes, &mut commands, &shared_materials, &mut lines_materials);
                            bend_commands.straight = lraclr_arr;
                        }
                        Err(_) => {}
                    }
                }
            };
            ui.separator();
            if ui.button("Reverse").clicked() {
                for (entity,_) in &mut query_meshes {
                    commands.entity(entity).despawn();
                }
                for (entity,_) in &mut query_centerlines {
                    commands.entity(entity).despawn();
                }
               let lraclr_arr= reverse_lraclr( &bend_commands.straight);
                load_mesh(&lraclr_arr, &mut meshes, &mut commands, &shared_materials, &mut lines_materials);
                bend_commands.straight = lraclr_arr;
            };
            ui.separator();
            if ui.button("Simulate").clicked() {};
            ui.separator();
            if ui.button("CSV").clicked() {}
            ui.separator();
            ui.menu_button("Demos", |ui| {
                ui.style_mut().wrap_mode = Some(egui::TextWrapMode::Extend);
                let mut stp: Vec<u8>=vec![];
                if ui.button("Demo1").clicked() {
                    stp= Vec::from((include_bytes!("../files/1.stp")).as_slice());

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

                if(!stp.is_empty()){
                    for (entity,_) in &mut query_meshes {
                        commands.entity(entity).despawn();
                    }
                    for (entity,_) in &mut query_centerlines {
                        commands.entity(entity).despawn();
                    }
                    let lraclr_arr_mm: Vec<LRACLR> = analyze_stp(&stp);
                    let lraclr_arr: Vec<LRACLR> = convert_to_meter(&lraclr_arr_mm);
                    load_mesh(&lraclr_arr, &mut meshes, &mut commands, &shared_materials, &mut lines_materials);
                    bend_commands.straight = lraclr_arr;
                }

            });
            ui.separator();
            ui.label("DORN L");
        });
    });

    occupied_screen_space.left = egui::SidePanel::left("left_panel").resizable(true).show(ctx, |ui| {
        if (ui.rect_contains_pointer(ui.max_rect())) {
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

        //let cylinders: Vec<(Entity, &MainCylinder)> =query_cilinder.iter().collect_vec();
        //let tors: Vec<(Entity, &BendToro)> =query_tor.iter().collect_vec();
        //let len_tors=tors.len();
        let col_width = 50.0;
        let col_heigth = 8.0;
        let color = egui::Color32::from_rgb(255, 255, 255);
        egui::ScrollArea::vertical().show(ui, |ui| {
            ui.horizontal(|ui| {
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

            let _ = &bend_commands.straight.iter().for_each(|lraclr| {
                ui.horizontal(|ui| {
                    let l_labl = ui.add_sized([col_width, col_heigth], egui::Label::new(egui::RichText::new(((lraclr.l * 1000.0) as i32).to_string()).color(color)));
                    if (l_labl.clicked()) {
                        query_pipes.iter_mut().for_each(|(mut m, pipe)| {
                            match pipe {
                                MainPipe::Pipe(pipe) => {
                                    if (lraclr.id1 == pipe.id as i32) {
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

                    ui.separator();
                    ui.add_sized([col_width, col_heigth], egui::Label::new(egui::RichText::new(((lraclr.r) as i32).to_string()).color(color)));
                    ui.separator();
                    let a_labl = ui.add_sized([col_width, col_heigth], egui::Label::new(egui::RichText::new(((lraclr.a) as i32).to_string()).color(color)));
                    if (a_labl.clicked()) {
                        query_pipes.iter_mut().for_each(|(mut m, pipe)| {
                            match pipe {
                                MainPipe::Pipe(pipe) => {
                                    m.0 = shared_materials.white_matl.clone();
                                }
                                MainPipe::Tor(tor) => {
                                    if (lraclr.id2 == tor.id as i32) {
                                        m.0 = shared_materials.pressed_matl.clone();
                                    } else {
                                        m.0 = shared_materials.red_matl.clone();
                                    }
                                }
                            };
                        });
                    }
                    ui.separator();
                    ui.add_sized([col_width, col_heigth], egui::Label::new(egui::RichText::new((round((abs(Rad::from(Deg(lraclr.a)).0) * (lraclr.clr * 1000.0)) as f32) as i32).to_string()).color(color)));
                    ui.separator();
                    ui.add_sized([col_width, col_heigth],
                                 egui::Label::new(egui::RichText::new(((lraclr.clr * 1000.0) as i32).to_string()).color(color)),
                    );
                    ui.separator();
                });
            });
        });

        let height = TextStyle::Body.resolve(ui.style()).size;
        ui.allocate_rect(ui.available_rect_before_wrap(), egui::Sense::hover());
    }).response.rect.width();
}

pub fn load_mesh(lraclr_arr: &Vec<LRACLR>, meshes: &mut ResMut<Assets<Mesh>>, commands: &mut Commands, shared_materials: &Res<SharedMaterials>, lines_materials: &mut ResMut<Assets<LineMaterial>>) {
    let mut the_up_dir: Vector3<f64> = Vector3::new(0., 0., 1.);
    let mut dxf_lines_c: Vec<(Vec3, Vec3)> = vec![];
    let mut dxf_lines_t: Vec<(Vec3, Vec3)> = vec![];

    //    let stp: Vec<u8> = Vec::from((include_bytes!("files/9.stp")).as_slice());
    //let lraclr_arr_mm: Vec<LRACLR> = analyze_stp(&stp);
    //let lraclr_arr: Vec<LRACLR> = convert_to_meter(&lraclr_arr_mm);
    let (circles, tors) = cnc_to_poly(&lraclr_arr, &the_up_dir);

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
    //lraclr_arr
}
