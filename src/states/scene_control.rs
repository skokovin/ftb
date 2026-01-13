use bevy::app::{App, Plugin, Startup};
use bevy::asset::{AssetServer, Assets};
use bevy::color::palettes::tailwind::{BLUE_500, CYAN_300, GRAY_300, RED_500, YELLOW_300};
use bevy::core_pipeline::tonemapping::Tonemapping;
use bevy::light::DirectionalLight;
use bevy::pbr::StandardMaterial;
use bevy::post_process::bloom::Bloom;
use bevy::prelude::*;
use bevy::render::view::Hdr;
use bevy_ecs::change_detection::{Res, ResMut};
use bevy_ecs::prelude::Commands;
use crate::render::line::{LineList, LineMaterial};
use crate::states::pipe_control::PipeMesh;
use crate::states::state_machine::{MachineRegisters, RobotState};
use crate::ui::camera::{cad_camera_controller, CadCamera};
#[derive(Component)]
pub struct Resettable;

#[derive(States, Debug, Clone, Copy, Hash, PartialEq, Eq, Default)]
pub enum AppMode {
    #[default]
    Loading,
    Restarting,
    StandBy,
    Pause,
    Editing,
    Simulating,
    Rotating,
    ReturnBendingHead,
}
#[derive(Resource)]
pub struct SharedMaterials {
    diffuse_map: Handle<Image>,
    specular_map: Handle<Image>,
    white_matl: Handle<StandardMaterial>,
    ground_matl: Handle<StandardMaterial>,
    hover_matl: Handle<StandardMaterial>,
    pressed_matl: Handle<StandardMaterial>,
    red_matl: Handle<StandardMaterial>,
}

pub struct AppScenePlugin;

impl Plugin for AppScenePlugin {
    fn build(&self, app: &mut App) {
        //app.add_systems(Startup, (init_scene));
        app.add_systems(OnEnter(AppMode::Loading), (loading_scene, init_scene));
        app.add_systems(OnEnter(AppMode::Restarting), cleanup_scene);
        app.add_systems(OnEnter(AppMode::StandBy), stand_by);
        app.add_systems(OnEnter(AppMode::Simulating), simulating);
        app.add_systems(Update, (cad_camera_controller, draw_gizmos));
        //app.add_systems(Update, update_pipe_system);
    }
}

fn cleanup_scene(mut commands: Commands, mut next_state: ResMut<NextState<AppMode>>, old_meshes: Query<Entity, With<Resettable>>) {
    for entity in old_meshes.iter() {
        commands.entity(entity).despawn();
    }
    next_state.set(AppMode::StandBy);
}

fn loading_scene(mut next_state: ResMut<NextState<AppMode>>) {
    next_state.set(AppMode::Restarting);
}

fn stand_by(mut status: ResMut<MachineRegisters>, // Читаем и пишем статус станка
            mut next_state: ResMut<NextState<AppMode>>, // Переключаем режимы
            current_state: Res<State<AppMode>>, // Узнаем текущий режим
            curr_robot_state: Res<State<RobotState>>,
            mut next_robot_state: ResMut<NextState<RobotState>>, ) {
    status.robot_state = curr_robot_state.get().clone();
    next_robot_state.set(RobotState::PipeLoading);
}
fn simulating(mut status: ResMut<MachineRegisters>, // Читаем и пишем статус станка
              mut next_state: ResMut<NextState<AppMode>>, // Переключаем режимы
              current_state: Res<State<AppMode>>, // Узнаем текущий режим
              curr_robot_state: Res<State<RobotState>>,
              mut next_robot_state: ResMut<NextState<RobotState>>, ) {

    println!("curr_robot_state {:?} {:?}", curr_robot_state.get(),status.robot_state);

    if (status.robot_state == RobotState::PipeLoaded) {
        next_robot_state.set(RobotState::Feeding);
        println!("next_robot_state {:?}", next_robot_state);
    } else {
        next_robot_state.set(status.robot_state);
    }
}

fn init_scene(mut commands: Commands, mut materials: ResMut<Assets<StandardMaterial>>, asset_server: Res<AssetServer>) {
    commands.spawn((
        DirectionalLight {
            shadows_enabled: false,
            shadow_depth_bias: 0.5,
            shadow_normal_bias: 0.5,
            ..default()
        },
        Transform::from_xyz(100.0, 100.0, 100.0),
        Resettable,
    ));

    let diffuse_map: Handle<Image> = asset_server.load("environment_maps/diffuse_rgb9e5_zstd.ktx2");
    let specular_map: Handle<Image> = asset_server.load("environment_maps/specular_rgb9e5_zstd.ktx2");

    let white_matl: Handle<StandardMaterial> = materials.add(
        StandardMaterial {
            base_color: Color::from(BLUE_500), // Deep red color
            metallic: 0.5,                         // Highly metallic (0.0 to 1.0)
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
        metallic: 0.5,                         // Highly metallic (0.0 to 1.0)
        perceptual_roughness: 0.7,              // Very smooth/polished (0.0 to 1.0)
        reflectance: 0.7,
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
    //let cam_trans = Transform::from_xyz(3000.0, 0., 3000.0).looking_at(CAMERA_TARGET, Vec3::Z);
    //let cam_trans =  Transform::from_xyz(0.7, 0.7, 1.0).looking_at(Vec3::new(0.0, 0.3, 0.0), Vec3::Y);

    commands.spawn((
        Camera3d::default(),
        Transform::from_xyz(3000.0, 3000.0, 3000.0)
            .looking_at(Vec3::ZERO, Vec3::Z),
        CadCamera {
            focus_point: Vec3::ZERO, // Смотрим в центр
            dist: 5000.0,            // Дистанция
            ..default()
        },
        Tonemapping::AcesFitted,
        Bloom::default(),
        EnvironmentMapLight {
            intensity: 3000.0,
            rotation: Default::default(),
            diffuse_map: shm.diffuse_map.clone(),
            specular_map: shm.specular_map.clone(),
            affects_lightmapped_mesh_diffuse: false,
        },
        Msaa::Sample4,
        Hdr,
    ));
    commands.insert_resource(shm);
}

fn setup_axes(mut commands: Commands, mut meshes: ResMut<Assets<Mesh>>, mut lines_materials: ResMut<Assets<LineMaterial>>) {
    let l = 100.0;

    commands.spawn((
        Mesh3d(meshes.add(LineList {
            lines: vec![(Vec3::ZERO, Vec3::new(1.0 * l, 0.0, 0.0))],
        })),
        MeshMaterial3d(lines_materials.add(LineMaterial {
            color: LinearRgba::GREEN,
        })),
    ));

    commands.spawn((
        Mesh3d(meshes.add(LineList {
            lines: vec![(Vec3::ZERO, Vec3::new(0.0, 1.0 * l, 0.0))],
        })),
        MeshMaterial3d(lines_materials.add(LineMaterial {
            color: LinearRgba::RED,
        })),
    ));
    commands.spawn((
        Mesh3d(meshes.add(LineList {
            lines: vec![(Vec3::ZERO, Vec3::new(0.0, 0.0, 1.0 * l))],
        })),
        MeshMaterial3d(lines_materials.add(LineMaterial {
            color: LinearRgba::BLUE,
        })),
    ));
}
fn draw_gizmos(mut gizmos: Gizmos) {
    gizmos.axes(Transform::IDENTITY, 1000.0);
}



