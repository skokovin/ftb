//#![cfg_attr(not(debug_assertions), windows_subsystem = "windows")]

use std::f32::consts::PI;
use std::fs::File;
use std::sync::atomic::AtomicUsize;
use std::time::Duration;
use bevy::anti_alias::smaa::Smaa;
use bevy::asset::{AssetServer, Assets, Handle};
use bevy::camera::Viewport;
use bevy::color::palettes::tailwind::{BLUE_500, CYAN_300, GRAY_300, GRAY_600, RED_300, RED_500, YELLOW_300};


use bevy::core_pipeline::tonemapping::Tonemapping;
use bevy::DefaultPlugins;
use bevy::image::Image;
use bevy::light::CascadeShadowConfigBuilder;
use bevy::pbr::{ScreenSpaceAmbientOcclusion, StandardMaterial};
use bevy::platform::collections::HashMap;
use bevy::post_process::bloom::Bloom;
use bevy::window::{ExitCondition, PrimaryWindow};
use bevy::winit::WinitWindows;
use bevy_ecs::change_detection::Mut;
use bevy_ecs::entity::Entity;
use bevy_ecs::entity_disabling::Disabled;
use bevy_ecs::event::EventWriter;
use bevy_ecs::name::Name;
use bevy_ecs::prelude::{ContainsEntity, EventReader, NonSend, Trigger, Without};
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
use crate::algo::{analyze_stp, analyze_stp_path, BendToro, MainCylinder, MainPipe};
use crate::algo::cnc::{cnc_to_poly, cnc_to_poly_animate, reverse_lraclr, AnimState, AnimStatus, LRACLR};
use crate::ui::{byt, load_anim_mesh, load_mesh, load_mesh_centerline, reload_mesh, ui_system, UiState, LRAUI};

use bevy::prelude::*;
mod algo;
mod ui;
mod adds;

pub enum ActiveArea {
    D3Spase,
    UI,
}
const CAMERA_TARGET: Vec3 = Vec3::new(0., 0., 0.);

#[derive(Event,Message, Debug)] // Using Debug for easy printing
pub struct TickEvent;
#[derive(Resource)]
struct TickTimer(Timer);

#[derive(Component, Clone)]
struct TestLines;

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
#[derive(Component)]
struct SimpleAnimation {
    start_pos_x: f32,
    end_pos_x: f32,
    duration: f32,
    elapsed: f32,
}
#[derive(Resource)]
pub struct VisibilityStore {
    pub visible_roots: Vec<Entity>,
}
#[derive(Resource)]
pub struct BendCommands {
    pub straight: Vec<LRACLR>,
    pub selected_id: i32,
    pub up_dir: cgmath::Vector3<f64>,
    pub original_file: Vec<u8>,
    pub anim_state: AnimState,
    pub t: f64,
}
impl Default for BendCommands {
    fn default() -> Self { BendCommands { straight: vec![], selected_id: i32::MIN, up_dir: Vector3::new(0.0, 0.0, -1.0), original_file: vec![], anim_state: AnimState::default(), t: 1.0 } }
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

#[derive(Resource)]
struct MachineAssets {
    handles: HashMap<String, Handle<Scene>>,
    meshes_loaded: bool,
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


    App::new()
        .insert_resource(Time::<Fixed>::from_hz(30.0))
        .insert_resource(vis_stor).insert_resource(egui_settings)
        .init_resource::<OccupiedScreenSpace>().init_resource::<BendCommands>()
        .insert_resource(UiState { lrauis: vec![], total_length: "0".to_string(), pipe_diameter: "0".to_string() })
        .add_message::<TickEvent>()
        // 2. Insert the timer resource
        .insert_resource(TickTimer(Timer::new(
            Duration::from_millis(20),
            TimerMode::Repeating, // Make the timer repeat
        )))
        .add_plugins((
        DefaultPlugins.set(WindowPlugin {
            primary_window: Some(Window {
                title: "Cansa Makina's pipe bend app".to_string(),
                ..default()
            }),
            ..default()
        }),
        HttpClientPlugin,
        MeshPickingPlugin,
        DefaultEditorCamPlugins,
        EguiPlugin::default(),
        MaterialPlugin::<LineMaterial>::default(),
    )).add_systems(PreStartup, setup_scene)
        .add_systems(EguiPrimaryContextPass, (ui_system,))
        .add_systems(FixedUpdate, animation_system)

/*        .add_systems(Startup, (setup_scene_utils, setup_machine, setup_drawings_layer))
        .add_systems(PostStartup, after_setup_scene)
        .add_systems(Update, (update_camera_transform_system, animate_simple))*/

        .add_systems(Startup, (setup_scene_utils, setup_drawings_layer,setup_machine))
        .add_systems(PostStartup, after_setup_scene)
        .add_systems(Update, (tick_timer_system, event_listener_system,update_camera_transform_system,test_system))
        .run();
}


fn setup_scene(mut commands: Commands,
               mut materials: ResMut<Assets<StandardMaterial>>,
               asset_server: Res<AssetServer>,
) {
    let diffuse_map: Handle<Image> = asset_server.load("environment_maps/diffuse_rgb9e5_zstd.ktx2");
    let specular_map: Handle<Image> = asset_server.load("environment_maps/specular_rgb9e5_zstd.ktx2");

    let white_matl: Handle<StandardMaterial> = materials.add(
        StandardMaterial {
            base_color: Color::from(BLUE_500), // Deep red color
            metallic: 0.95,                         // Highly metallic (0.0 to 1.0)
            perceptual_roughness: 0.7,              // Very smooth/polished (0.0 to 1.0)
            reflectance: 0.7,                       // How much it reflects its environment
            ..default()
        }
    );
    let ground_matl: Handle<StandardMaterial> = materials.add(Color::from(GRAY_300));
    let hover_matl: Handle<StandardMaterial> = materials.add(Color::from(CYAN_300));
    let pressed_matl: Handle<StandardMaterial> = materials.add(Color::from(YELLOW_300));
    let red_matl: Handle<StandardMaterial> = materials.add(StandardMaterial {
        base_color: Color::from(RED_500), // Deep red color
        metallic: 0.95,                         // Highly metallic (0.0 to 1.0)
        perceptual_roughness: 0.7,              // Very smooth/polished (0.0 to 1.0)
        reflectance: 0.7,                       // How much it reflects its environment
        ..default()
    });
    let shm = SharedMaterials {
        diffuse_map,
        specular_map,
        white_matl,
        ground_matl,
        hover_matl,
        pressed_matl,
        red_matl,
    };
    let cam_trans = Transform::from_xyz(3000.0, 0., 3000.0).looking_at(CAMERA_TARGET, Vec3::Z);
    //let cam_trans =  Transform::from_xyz(0.7, 0.7, 1.0).looking_at(Vec3::new(0.0, 0.3, 0.0), Vec3::Y);
    commands.insert_resource(OriginalCameraTransform(cam_trans));
    commands.spawn((
        Camera3d::default(),
        Camera {
            ..Default::default()
        },
        cam_trans.clone(),
        Tonemapping::AcesFitted,
        Bloom::default(),
        EnvironmentMapLight {
            intensity: 7000.0,
            rotation: Default::default(),
            diffuse_map: shm.diffuse_map.clone(),
            specular_map: shm.specular_map.clone(),
            affects_lightmapped_mesh_diffuse: true,
        },
        /*     DirectionalLight {
            illuminance: light_consts::lux::FULL_DAYLIGHT,
            shadows_enabled: true,
            ..default()
        },
        CascadeShadowConfigBuilder {
                 maximum_distance: 3000.0,
                 first_cascade_far_bound: 900.0,
                 ..default()
             }.build(),*/
        EditorCam {
            orbit_constraint: OrbitConstraint::Free,
            last_anchor_depth: -cam_trans.translation.length() as f64,
            orthographic: projections::OrthographicSettings {
                scale_to_near_clip: 1_f32, // Needed for SSAO to work in ortho
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


fn setup_machine(asset_server: Res<AssetServer>, mut commands: Commands) {
    let t: Transform = Transform {
        translation: Vec3::new(-3058.0, -289.37, -154.89), //Vec3::new(289.37, -3058.0, -154.89)
        rotation: Quat::from_rotation_y(std::f32::consts::PI) * Quat::from_rotation_z(std::f32::consts::PI / 2.0),
        scale: Vec3::new(1000.0, 1000.0, 1000.0),
    };

    let dayama_alt: SceneRoot = SceneRoot(asset_server.load(GltfAssetLabel::Scene(0).from_asset("machines/m2/dayama_alt2.glb")));
    let dayamam_kizak: SceneRoot = SceneRoot(asset_server.load(GltfAssetLabel::Scene(0).from_asset("machines/m2/dayamam_kizak2.glb")));
    let dayamam_kizak_arka: SceneRoot = SceneRoot(asset_server.load(GltfAssetLabel::Scene(0).from_asset("machines/m2/dayamam_kizak_arka2.glb")));
    let malafa: SceneRoot = SceneRoot(asset_server.load(GltfAssetLabel::Scene(0).from_asset("machines/m2/malafa.glb")));
    let mengene: SceneRoot = SceneRoot(asset_server.load(GltfAssetLabel::Scene(0).from_asset("machines/m2/mengene.glb")));
    let mengene_alt: SceneRoot = SceneRoot(asset_server.load(GltfAssetLabel::Scene(0).from_asset("machines/m2/mengene_alt.glb")));
    let palka2m: SceneRoot = SceneRoot(asset_server.load(GltfAssetLabel::Scene(0).from_asset("machines/m2/palka2m.glb")));
    let palkam: SceneRoot = SceneRoot(asset_server.load(GltfAssetLabel::Scene(0).from_asset("machines/m2/palkam.glb")));
    let pens: SceneRoot = SceneRoot(asset_server.load(GltfAssetLabel::Scene(0).from_asset("machines/m2/pens.glb")));
    let sasi: SceneRoot = SceneRoot(asset_server.load(GltfAssetLabel::Scene(0).from_asset("machines/m2/sasi.glb")));


    commands.spawn(
        (
            dayamam_kizak_arka,
            Transform {
                translation: Vec3::new(-3058.0, -289.37, -154.89), //Vec3::new(289.37, -3058.0, -154.89)
                rotation: Quat::from_rotation_y(std::f32::consts::PI) * Quat::from_rotation_z(std::f32::consts::PI / 2.0),
                scale: Vec3::new(1000.0, 1000.0, 1000.0),
            },
            Name::new("dayamam_kizak_arka"),
            SimpleAnimation {
                start_pos_x: 0.0,
                end_pos_x: 0.0,
                duration: 0.0,
                elapsed: 0.0,
            },
        )
    );
    commands.spawn(
        (
            dayamam_kizak,
            Transform {
                translation: Vec3::new(-3058.0, -289.37, -154.89), //Vec3::new(289.37, -3058.0, -154.89)
                rotation: Quat::from_rotation_y(std::f32::consts::PI) * Quat::from_rotation_z(std::f32::consts::PI / 2.0),
                scale: Vec3::new(1000.0, 1000.0, 1000.0),
            },
            Name::new("dayamam_kizak"),
            SimpleAnimation {
                start_pos_x: 0.0,
                end_pos_x: 0.0,
                duration: 0.0,
                elapsed: 0.0,
            },
        )
    );
    commands.spawn(
        (
            dayama_alt,
            Transform {
                translation: Vec3::new(-3058.0, -289.37, -154.89), //Vec3::new(289.37, -3058.0, -154.89)
                rotation: Quat::from_rotation_y(std::f32::consts::PI) * Quat::from_rotation_z(std::f32::consts::PI / 2.0),
                scale: Vec3::new(1000.0, 1000.0, 1000.0),
            },
            Name::new("dayama_alt"),
            SimpleAnimation {
                start_pos_x: 0.0,
                end_pos_x: 0.0,
                duration: 0.0,
                elapsed: 0.0,
            },
        )
    );

    commands.spawn(
        (
            mengene_alt,
            Transform {
                translation: Vec3::new(-3058.0, -289.37, -154.89), //Vec3::new(289.37, -3058.0, -154.89)
                rotation: Quat::from_rotation_y(std::f32::consts::PI) * Quat::from_rotation_z(std::f32::consts::PI / 2.0),
                scale: Vec3::new(1000.0, 1000.0, 1000.0),
            },
            Name::new("mengene_alt"),
            SimpleAnimation {
                start_pos_x: 0.0,
                end_pos_x: 0.0,
                duration: 0.0,
                elapsed: 0.0,
            },
        )
    );

    commands.spawn(
        (
            mengene,
            Transform {
                translation: Vec3::new(-3058.0, -289.37, -154.89), //Vec3::new(289.37, -3058.0, -154.89)
                rotation: Quat::from_rotation_y(std::f32::consts::PI) * Quat::from_rotation_z(std::f32::consts::PI / 2.0),
                scale: Vec3::new(1000.0, 1000.0, 1000.0),
            },
            Name::new("mengene"),
            SimpleAnimation {
                start_pos_x: 0.0,
                end_pos_x: 0.0,
                duration: 0.0,
                elapsed: 0.0,
            },
        )
    );
    commands.spawn(
        (
            palka2m,
            Transform {
                translation: Vec3::new(-3058.0, -289.37, -154.89), //Vec3::new(289.37, -3058.0, -154.89)
                rotation: Quat::from_rotation_y(std::f32::consts::PI) * Quat::from_rotation_z(std::f32::consts::PI / 2.0),
                scale: Vec3::new(1000.0, 1000.0, 1000.0),
            },
            Name::new("palka2m"),
            SimpleAnimation {
                start_pos_x: 0.0,
                end_pos_x: 0.0,
                duration: 0.0,
                elapsed: 0.0,
            },
        )
    );

    commands.spawn(
        (
            palkam,
            Transform {
                translation: Vec3::new(-3058.0, -289.37, -154.89), //Vec3::new(289.37, -3058.0, -154.89)
                rotation: Quat::from_rotation_y(std::f32::consts::PI) * Quat::from_rotation_z(std::f32::consts::PI / 2.0),
                scale: Vec3::new(1000.0, 1000.0, 1000.0),
            },
            Name::new("palkam"),
            SimpleAnimation {
                start_pos_x: 0.0,
                end_pos_x: 0.0,
                duration: 0.0,
                elapsed: 0.0,
            },
        )
    );

    commands.spawn(
        (
            pens,
            Transform {
                translation: Vec3::new(-3058.0, -289.37, -154.89), //Vec3::new(289.37, -3058.0, -154.89)
                rotation: Quat::from_rotation_y(std::f32::consts::PI) * Quat::from_rotation_z(std::f32::consts::PI / 2.0),
                scale: Vec3::new(1000.0, 1000.0, 1000.0),
            },
            Name::new("pens"),
            SimpleAnimation {
                start_pos_x: t.translation.x,
                end_pos_x: 0.0,
                duration: 0.0,
                elapsed: 0.0,
            },
        )
    );

    commands.spawn(
        (
            malafa,
            Transform {
                translation: Vec3::new(-3058.0, -289.37, -154.89), //Vec3::new(289.37, -3058.0, -154.89)
                rotation: Quat::from_rotation_y(std::f32::consts::PI) * Quat::from_rotation_z(std::f32::consts::PI / 2.0),
                scale: Vec3::new(1000.0, 1000.0, 1000.0),
            },
            Name::new("malafa"),
            SimpleAnimation {
                start_pos_x: 0.0,
                end_pos_x: 0.0,
                duration: 0.0,
                elapsed: 0.0,
            },
        )
    );
    commands.spawn(
        (
            sasi,
            Transform {
                translation: Vec3::new(-3058.0, -289.37, -154.89), //Vec3::new(289.37, -3058.0, -154.89)
                rotation: Quat::from_rotation_y(std::f32::consts::PI) * Quat::from_rotation_z(std::f32::consts::PI / 2.0),
                scale: Vec3::new(1000.0, 1000.0, 1000.0),
            },
            Name::new("sasi"),
            SimpleAnimation {
                start_pos_x: 0.0,
                end_pos_x: 0.0,
                duration: 0.0,
                elapsed: 0.0,
            },
        )
    );

    // Light
    commands.spawn((
        DirectionalLight {
            illuminance: light_consts::lux::FULL_DAYLIGHT,
            shadows_enabled: true,
            ..default()
        },
        CascadeShadowConfigBuilder {
            maximum_distance: 3000.0,
            first_cascade_far_bound: 900.0,
            ..default()
        }.build(),
        Transform::from_rotation(Quat::from_euler(EulerRot::ZYX, 0.0, PI * -0.15, PI * -0.15)),
    ));
}


fn after_setup_scene() {
    /*  windows: NonSend<WinitWindows>,*/
    /*    for window in windows.windows.values() {
            window.set_title("Cansa Makina's pipe bend app");
            window.set_window_icon(Some(load_icon()));
        }*/
}


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

fn setup_scene_utils(mut commands: Commands, mut meshes: ResMut<Assets<Mesh>>, mut lines_materials: ResMut<Assets<LineMaterial>>) {
    let l=100.0;
    commands.spawn((
        Mesh3d(meshes.add(LineList {
            lines: vec![(Vec3::ZERO, Vec3::new(1.0*l, 0.0, 0.0))],
        })),
        MeshMaterial3d(lines_materials.add(LineMaterial {
            color: LinearRgba::GREEN,
        })),
    ));

    commands.spawn((
        Mesh3d(meshes.add(LineList {
            lines: vec![(Vec3::ZERO, Vec3::new(0.0, 1.0*l, 0.0))],
        })),
        MeshMaterial3d(lines_materials.add(LineMaterial {
            color: LinearRgba::RED,
        })),
    ));
    commands.spawn((
        Mesh3d(meshes.add(LineList {
            lines: vec![(Vec3::ZERO, Vec3::new(0.0, 0.0, 1.0*l))],
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
    mut ui_state: ResMut<UiState>,
    mut lines_materials: ResMut<Assets<LineMaterial>>,
    shared_materials: Res<SharedMaterials>,
) {
    let stp: Vec<u8> = Vec::from((include_bytes!("files/9.stp")).as_slice());
    let lraclr_arr: Vec<LRACLR> = analyze_stp(&stp);
   // load_mesh(&lraclr_arr, &mut meshes, &mut commands, &shared_materials, &mut lines_materials, &mut ui_state, &mut bend_commands);
    load_mesh_centerline(&lraclr_arr, &mut meshes, &mut commands, &shared_materials, &mut lines_materials, &mut ui_state, &mut bend_commands);
    bend_commands.original_file = stp;

}

fn animation_system(time: Res<Time<Fixed>>, mut bend_commands: ResMut<BendCommands>, mut query_meshes: Query<(Entity, &MainPipe)>, mut query_centerlines: Query<(Entity, &PipeCenterLine)>, mut commands: Commands,
                    shared_materials: Res<SharedMaterials>,
                    mut meshes: ResMut<Assets<Mesh>>,
                    mut ui_state: ResMut<UiState>,
                    mut lines_materials: ResMut<Assets<LineMaterial>>, ) {
    match bend_commands.anim_state.status {
        AnimStatus::Enabled => {
            bend_commands.anim_state.dt = time.delta_secs_f64();
            let (c, t) = cnc_to_poly_animate(&mut bend_commands);
            for (entity, _) in &mut query_meshes {
                commands.entity(entity).despawn();
            }
            for (entity, _) in &mut query_centerlines {
                commands.entity(entity).despawn();
            }
            load_anim_mesh(c, t, &mut meshes, &mut commands, &shared_materials, &mut lines_materials);
        }
        AnimStatus::Disabled => {}
        AnimStatus::Finished => {
            bend_commands.anim_state.status = AnimStatus::Disabled;
            reload_mesh(&mut meshes, &mut commands, &shared_materials, &mut lines_materials, &mut ui_state, &mut bend_commands);
            println!("anim finished");
        }
    }
}

fn on_mouse_button_click(
    click: On<Pointer<Click>>,
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
            let (e2, mut m2) = query.get_mut(click.target().entity()).unwrap();
            m2.0 = shared_materials.pressed_matl.clone();
            match query_pipes.get_mut(click.target().entity()) {
                Ok((ent, mut pipe)) => {
                    match pipe.as_mut() {
                        MainPipe::Pipe(pipe) => {
                            bend_commands.selected_id = pipe.id as i32;
                            println!("cil {:?}", pipe.id);
                        }
                        MainPipe::Tor(tor) => {
                            bend_commands.selected_id = tor.id as i32;
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


fn animate_simple(
    time: Res<Time>,
    mut query: Query<(&mut Transform, &Name, &mut SimpleAnimation)>,
) {
    for (mut transform, name, mut animation) in &mut query {
        if (animation.duration != 0.0) {
            animation.elapsed += time.delta_secs();
            // Loop the animation


            // Calculate progress (0.0 to 1.0)
            let progress = animation.elapsed / animation.duration;

            // Use smooth step for easing
            let t = smooth_step(progress);


            // Interpolate position
            transform.translation.x = animation.start_pos_x.lerp(animation.end_pos_x, t);
            println!("{:?}", transform.translation.x);
            if animation.elapsed > animation.duration {
                animation.elapsed = 0.0;
                animation.duration = 0.0;
                transform.translation.x = animation.end_pos_x;
                animation.start_pos_x = transform.translation.x;
                //animation.start_pos_x = transform.translation.x;
            }
        }
    }
}

fn test_system(mut commands: Commands,
               mut meshes: ResMut<Assets<Mesh>>,
               query_testlines: Query<(Entity, &TestLines)>,
               mut lines_materials: ResMut<Assets<LineMaterial>>,
               mut bend_commands: ResMut<BendCommands>,
               mut query_transform: Query<&mut Transform, With<PipeCenterLine>>,

               /*    mut query_transform: Query<(&mut Transform, &Name, &mut PipeCenterLine)>,*/
             /*  mut query_transform: Query<(Entity, &PipeCenterLine)>*/

) {

    for (entity, testline) in &query_testlines {
            commands.entity(entity).despawn();
    }


    let t=bend_commands.t;
    let lraclr_arr=&bend_commands.straight;


    let (pt,xv,yv,zv)=byt(t,lraclr_arr,&bend_commands.up_dir);

    let v0=Vec3::new(pt.x as f32,pt.y as f32,pt.z as f32);
    let vx=v0+Vec3::new(xv.x as f32,xv.y as f32,xv.z as f32)*100.0;
    let vy=v0+Vec3::new(yv.x as f32,yv.y as f32,yv.z as f32)*100.0;
    let vz=v0+Vec3::new(zv.x as f32,zv.y as f32,zv.z as f32)*100.0;



/*    let source_pos = pt;
    let source_x = Vec3::new(xv.x as f32,xv.y as f32,xv.z as f32);
    let source_y = Vec3::new(yv.x as f32,yv.y as f32,yv.z as f32);
    let source_z = Vec3::new(zv.x as f32,zv.y as f32,zv.z as f32);


    let dest_pos = Vec3::new(0.0, 0.0, 0.0);
    let dest_x = Vec3::X;  // X-axis now points where Y used to
    let dest_y = Vec3::Y; // Y-axis now points where -X used to
    let dest_z = Vec3::Z;  // Z-axis is unchanged
*/

    let dest_pos =-Vec3::new(pt.x as f32, pt.y as f32, pt.z as f32);
    let source_x = Vec3::new(xv.x as f32,xv.y as f32,xv.z as f32);
    let source_y = Vec3::new(yv.x as f32,yv.y as f32,yv.z as f32);
    let source_z = Vec3::new(zv.x as f32,zv.y as f32,zv.z as f32);


    let source_pos = Vec3::new(0.0, 0.0, 0.0);
    let dest_x = Vec3::X;  // X-axis now points where Y used to
    let dest_y = Vec3::Y; // Y-axis now points where -X used to
    let dest_z = Vec3::Z;  // Z-axis is unchanged


    let m_source = Mat3::from_cols(source_x, source_y, source_z);
    let m_dest = Mat3::from_cols(dest_x, dest_y, dest_z);

    let rot_matrix = m_dest * m_source.transpose();
    //let final_transform_matrix = Mat4::from_mat3_translation(rot_matrix, dest_pos);

    let final_transform_matrix =Mat4::from_mat3(rot_matrix)  * Mat4::from_translation(dest_pos);


    //let final_transform: Transform = Transform::from_matrix(final_transform_matrix);

    query_transform.iter_mut().for_each(|mut transform| {
        *transform=Transform::from_matrix(final_transform_matrix);
        //println!("A {:?}", transform.translation.x);
    });


    {
        commands.spawn(
            (
                Mesh3d(meshes.add(LineList {
                    lines: vec![(v0, vx)],
                })),
                MeshMaterial3d(lines_materials.add(LineMaterial {
                    color: LinearRgba::rgb(1.0,0.3,0.5),
                })),
                Transform::from_matrix(final_transform_matrix),
                TestLines
            ));

        commands.spawn((
            Mesh3d(meshes.add(LineList {
                lines: vec![(v0, vy)],
            })),
            MeshMaterial3d(lines_materials.add(LineMaterial {
                color: LinearRgba::RED,
            })),
            Transform::from_matrix(final_transform_matrix),
            TestLines
        ));
        commands.spawn((
            Mesh3d(meshes.add(LineList {
                lines: vec![(v0, vz)],
            })),
            MeshMaterial3d(lines_materials.add(LineMaterial {
                color: LinearRgba::BLUE,
            })),
            Transform::from_matrix(final_transform_matrix),
            TestLines
        ));
    }

}

fn tick_timer_system(
    mut timer: ResMut<TickTimer>,
    time: Res<Time>,
    mut event_writer: MessageWriter<TickEvent>,
) {
    // Tick the timer with the time that has passed since the last frame
    timer.0.tick(time.delta());

    // The `just_finished()` method returns true only once per completion
    if timer.0.just_finished() {
        // 📨 Send the event
        event_writer.write(TickEvent);
       // println!("Sent TickEvent!");
    }
}

// This system listens for and reacts to the event
fn event_listener_system(
    mut events: MessageReader<TickEvent>,
    mut bend_commands: ResMut<BendCommands>,
) {
    // The .read() method iterates through all events of this type
    for event in events.read() {
        if(bend_commands.t>0.0){
            bend_commands.t-=0.001;
        }else{
            bend_commands.t=1.0;
        }

        //println!("-> 🎧 Received TickEvent: {:?}", event);
    }
}


fn smooth_step(t: f32) -> f32 {
    t * t * (3.0 - 2.0 * t)
}


fn load_icon() -> Icon {
    // Load your icon image here (example uses a small embedded image)
    let icon_buf = include_bytes!("../assets/icons/img.png");
    let image = image::load_from_memory(icon_buf).expect("Failed to load icon").into_rgba8();
    let icon: Icon = Icon::from_rgba(image.to_vec(), image.width(), image.height()).unwrap();
    icon
}