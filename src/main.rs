#![cfg_attr(not(debug_assertions), windows_subsystem = "windows")]
use std::fs::File;
use std::sync::atomic::AtomicUsize;
use bevy::asset::{AssetServer, Assets, Handle};
use bevy::color::palettes::tailwind::{CYAN_300, GRAY_300, RED_300, YELLOW_300};
use bevy::core_pipeline::bloom::Bloom;
use bevy::core_pipeline::smaa::Smaa;
use bevy::core_pipeline::tonemapping::Tonemapping;
use bevy::DefaultPlugins;
use bevy::image::Image;
use bevy::pbr::{ScreenSpaceAmbientOcclusion, StandardMaterial};
use bevy::prelude::{App, Camera, Camera3d, Click, Color, Commands, Component, Deref, DerefMut, EnvironmentMapLight, LinearRgba, MaterialPlugin, Mesh, Mesh3d, MeshMaterial3d, MeshPickingPlugin, Msaa, PluginGroup, PointLight, Pointer, PointerButton, PostStartup, PreStartup, Query, Res, ResMut, Resource, Startup, Transform, UVec2, Update, Vec3, Window, WindowPlugin, With};
use bevy::prelude::ops::round;
use bevy::render::camera::Viewport;
use bevy::window::{ExitCondition, PrimaryWindow};
use bevy::winit::WinitWindows;
use bevy_ecs::change_detection::Mut;
use bevy_ecs::entity::Entity;
use bevy_ecs::entity_disabling::Disabled;
use bevy_ecs::event::EventWriter;
use bevy_ecs::prelude::{ContainsEntity, NonSend, Trigger, Without};
use bevy_ecs::query::QueryEntityError;
use bevy_editor_cam::DefaultEditorCamPlugins;
use bevy_editor_cam::prelude::{projections, EditorCam, EnabledMotion, OrbitConstraint};
use bevy_egui::{egui, EguiContexts, EguiGlobalSettings, EguiPlugin, EguiPrimaryContextPass};
use bevy_egui::egui::TextStyle;

use bevy_http_client::{HttpClientPlugin, HttpRequest};
use cgmath::{Deg, Point3, Rad, Vector3};
use cgmath::num_traits::abs;
use ruststep::itertools::Itertools;
use winit::window::Icon;
use crate::adds::line::{LineList, LineMaterial};
use crate::algo::{analyze_stp, analyze_stp_path, convert_to_meter, BendToro, MainCylinder, MainPipe};
use crate::algo::cnc::{cnc_to_poly, LRACLR};
use crate::ui::{load_mesh, ui_system};


mod algo;
mod ui;
mod adds;

pub enum ActiveArea {
    D3Spase,
    UI,
}
const CAMERA_TARGET: Vec3 = Vec3::new(0., 0., 0.);
#[derive(Component, Clone)]
struct PipeCenterLine;
#[derive(Resource, Deref, DerefMut)]
struct OriginalCameraTransform(Transform);
#[derive(Default, Resource)]
struct OccupiedScreenSpace {
    left: f32,
    top: f32,
    right: f32,
    bottom: f32,
}

#[derive(Resource)]
pub struct VisibilityStore {
    pub visible_roots: Vec<Entity>,
}
#[derive(Resource)]
pub struct BendCommands {
    straight: Vec<LRACLR>,
    selected_id: i32,
    up_dir:cgmath::Vector3<f64>,
    original_file:Vec<u8>
}
impl Default for BendCommands {
    fn default() -> Self { BendCommands{ straight: vec![], selected_id: i32::MIN ,up_dir:Vector3::new(0.0, 0.0, -1.0),original_file:vec![]}}
}

#[derive(Resource)]
struct SharedMaterials {
    diffuse_map: Handle<Image>,
    specular_map: Handle<Image>,
    white_matl: Handle<StandardMaterial>,
    ground_matl: Handle<StandardMaterial>,
    hover_matl: Handle<StandardMaterial>,
    pressed_matl: Handle<StandardMaterial>,
    red_matl: Handle<StandardMaterial>,
}




fn main() {
    let vis_stor = VisibilityStore {
        visible_roots: vec![],
    };


    let egui_settings = EguiGlobalSettings {
        auto_create_primary_context: true,
        enable_focused_non_window_context_updates: false,
        input_system_settings: Default::default(),
        enable_absorb_bevy_input_system: false,
        enable_cursor_icon_updates: false,
    };


    App::new().insert_resource(vis_stor).insert_resource(egui_settings).init_resource::<OccupiedScreenSpace>().init_resource::<BendCommands>().add_plugins((
        DefaultPlugins,
        HttpClientPlugin,
        MeshPickingPlugin,
        DefaultEditorCamPlugins,
        EguiPlugin::default(),
        MaterialPlugin::<LineMaterial>::default(),
    )).add_systems(PreStartup, setup_scene).add_systems(Startup, (setup_scene_utils, setup_drawings_layer)).add_systems(PostStartup, after_setup_scene).add_systems(EguiPrimaryContextPass, (ui_system,)).add_systems(Update, (update_camera_transform_system)).run();
}
fn setup_scene(mut commands: Commands,
               mut materials: ResMut<Assets<StandardMaterial>>,
               asset_server: Res<AssetServer>,
               windows: NonSend<WinitWindows>,) {


    for window in windows.windows.values() {
        window.set_title("Cansa Makina's pipe bend app");
        window.set_window_icon(Some(load_icon()));
    }

    let diffuse_map: Handle<Image> = asset_server.load("environment_maps/diffuse_rgb9e5_zstd.ktx2");
    let specular_map: Handle<Image> = asset_server.load("environment_maps/specular_rgb9e5_zstd.ktx2");

    let white_matl: Handle<StandardMaterial> = materials.add(Color::WHITE);
    let ground_matl: Handle<StandardMaterial> = materials.add(Color::from(GRAY_300));
    let hover_matl: Handle<StandardMaterial> = materials.add(Color::from(CYAN_300));
    let pressed_matl: Handle<StandardMaterial> = materials.add(Color::from(YELLOW_300));
    let red_matl: Handle<StandardMaterial> = materials.add(Color::from(RED_300));
    let shm = SharedMaterials {
        diffuse_map,
        specular_map,
        white_matl,
        ground_matl,
        hover_matl,
        pressed_matl,
        red_matl,
    };

    /*    commands.spawn((
            PointLight {
                color: Default::default(),
                shadows_enabled: false,
                affects_lightmapped_mesh_diffuse: false,
                shadow_depth_bias: 0.0,
                shadow_normal_bias: 0.0,
                intensity: 100.,
                range: 2000.0,
                radius: 2000.0,
                shadow_map_near_z: 0.0,
            },
        ));*/

    let cam_trans = Transform::from_xyz(2.0, 0., 2.0).looking_at(CAMERA_TARGET, Vec3::Y);
    commands.insert_resource(OriginalCameraTransform(cam_trans));
    commands.spawn((
        Camera3d::default(),
        Camera {
            hdr: true,
            ..Default::default()
        },
        cam_trans.clone(),
        Tonemapping::AcesFitted,
        Bloom::default(),
        EnvironmentMapLight {
            intensity: 1000.0,
            rotation: Default::default(),
            diffuse_map: shm.diffuse_map.clone(),
            specular_map: shm.specular_map.clone(),
            affects_lightmapped_mesh_diffuse: true,
        },
        EditorCam {
            orbit_constraint: OrbitConstraint::Free,
            last_anchor_depth: -cam_trans.translation.length() as f64,
            orthographic: projections::OrthographicSettings {
                scale_to_near_clip: 1_000_f32, // Needed for SSAO to work in ortho
                ..Default::default()
            },
            ..Default::default()
        },
        ScreenSpaceAmbientOcclusion::default(),
        Smaa::default(),
        Msaa::Off,
    ));
    commands.insert_resource(shm);
}
fn after_setup_scene() {}


fn update_camera_transform_system(
    occupied_screen_space: Res<OccupiedScreenSpace>,
    //original_camera_transform: Res<OriginalCameraTransform>,
    windows: Query<&Window, With<PrimaryWindow>>,
    mut camera: Query<(&mut Camera)>,

    //mut camera_query: Query<(&Projection, &mut Transform)>,
) -> bevy_ecs::error::Result {
    let window = windows.single()?;
    let left_taken = occupied_screen_space.left / window.width();
    let w = window.width() * window.scale_factor() - occupied_screen_space.right * window.scale_factor() - occupied_screen_space.left * window.scale_factor();
    let h = window.height() * window.scale_factor() - occupied_screen_space.top * window.scale_factor() - occupied_screen_space.bottom * window.scale_factor();
    let top_taken = occupied_screen_space.top / window.height();
    let bottom_taken = occupied_screen_space.bottom / window.height();
    let mut c = camera.single_mut().unwrap();
    match &c.viewport {
        None => {}
        Some(vp) => {
            c.viewport = Some(Viewport {
                physical_position: UVec2::new(
                    (occupied_screen_space.left * window.scale_factor()) as u32,
                    (occupied_screen_space.top * window.scale_factor()) as u32,
                ),
                physical_size: UVec2::new(w as u32, h as u32),
                depth: vp.clone().depth,
            });
        }
    }

    Ok(())
}

fn setup_scene_utils(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut lines_materials: ResMut<Assets<LineMaterial>>,
) {
    commands.spawn((
        Mesh3d(meshes.add(LineList {
            lines: vec![(Vec3::ZERO, Vec3::new(0.05, 0.0, 0.0))],
        })),
        MeshMaterial3d(lines_materials.add(LineMaterial {
            color: LinearRgba::GREEN,
        })),
    ));

    commands.spawn((
        Mesh3d(meshes.add(LineList {
            lines: vec![(Vec3::ZERO, Vec3::new(0.0, 5.0, 0.0))],
        })),
        MeshMaterial3d(lines_materials.add(LineMaterial {
            color: LinearRgba::RED,
        })),
    ));
    commands.spawn((
        Mesh3d(meshes.add(LineList {
            lines: vec![(Vec3::ZERO, Vec3::new(0.0, 0.0, 5.0))],
        })),
        MeshMaterial3d(lines_materials.add(LineMaterial {
            color: LinearRgba::BLUE,
        })),
    ));
}

fn setup_drawings_layer(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut bend_commands: ResMut<BendCommands>,
    mut lines_materials: ResMut<Assets<LineMaterial>>,
    shared_materials: Res<SharedMaterials>,
) {
    let stp: Vec<u8> = Vec::from((include_bytes!("files/9.stp")).as_slice());
    let lraclr_arr_mm: Vec<LRACLR> = analyze_stp(&stp);
    let lraclr_arr: Vec<LRACLR> = convert_to_meter(&lraclr_arr_mm);
    load_mesh(&lraclr_arr,&mut meshes,&mut commands,&shared_materials,&mut lines_materials,&bend_commands.up_dir);
    bend_commands.straight = lraclr_arr;
    bend_commands.original_file = stp;
}
fn on_mouse_button_click(
    click: Trigger<Pointer<Click>>,
    mut commands: Commands,
    mut query: Query<(Entity, &mut MeshMaterial3d<StandardMaterial>)>,
    mut query_pipes: Query<(Entity, &mut MainPipe)>,
    //mut query_mat_pipes: Query<(&mut MeshMaterial3d<StandardMaterial>, &MainPipe)>,
    shared_materials: Res<SharedMaterials>,
    mut bend_commands: ResMut<BendCommands>,
) {
    match click.button {
        PointerButton::Primary => {
            query.iter_mut().for_each(|(e, mut m)| {
                match query_pipes.get_mut(e) {
                    Ok((ent, mut pipe)) => {
                        match pipe.as_mut() {
                            MainPipe::Pipe(pipe) => { m.0 = shared_materials.white_matl.clone(); }
                            MainPipe::Tor(tor) => { m.0 = shared_materials.red_matl.clone(); }
                        };
                    }
                    Err(_) => {}
                }
            });
            let (e2, mut m2) = query.get_mut(click.target.entity()).unwrap();
            m2.0 = shared_materials.pressed_matl.clone();
            match query_pipes.get_mut(click.target.entity()) {
                Ok((ent, mut pipe)) => {
                    match pipe.as_mut() {
                        MainPipe::Pipe(pipe) => {
                            bend_commands.selected_id= pipe.id as i32;
                            println!("cil {:?}", pipe.id); 
                        }
                        MainPipe::Tor(tor) => {
                            bend_commands.selected_id= tor.id as i32;
                            println!("cil {:?}", tor.id); 
                        }
                    };
                }
                Err(_) => {}
            }
        }
        PointerButton::Secondary => {}
        PointerButton::Middle => {}
    }
}


fn load_icon() -> Icon {
    // Load your icon image here (example uses a small embedded image)
    let icon_buf = include_bytes!("../assets/icons/img.png");
    let image = image::load_from_memory(icon_buf)
        .expect("Failed to load icon")
        .into_rgba8();
    let icon: Icon = Icon::from_rgba(image.to_vec(), image.width(), image.height()).unwrap();
    icon
}