use std::f32::consts::PI;
use bevy::app::{App, Plugin, Update};
use bevy::prelude::*;
use bevy_ecs::prelude::*;
use crate::algo::solids::generate_roller;
use crate::states::pipe_control::PipeSpecification;
use crate::states::scene_control::{AppMode, Resettable};
use crate::states::state_machine::{MachineRegisters, RobotState};

#[derive(Component)]
pub enum MachinePartKind {
    DayamaAlt,
    DayamamKizak,
    DayamamKizakArka,
    MALAFA,
    MENGENE,
    MengeneAlt,
    PALKA2M,
    PALKAM,
    PENS,
    SASI,
    PALKA3,
    DornA,
    DornB,
    Static,
}

pub struct MachineControlPlugin;

impl Plugin for MachineControlPlugin {
    fn build(&self, app: &mut App) {
        app.add_systems(OnEnter(RobotState::PipeLoaded), (setup_machine_hierarchy,setup_start_positions.after(setup_machine_hierarchy)));
        //app.add_systems(OnEnter(AppMode::StandBy), (setup_machine_hierarchy,setup_start_positions.after(setup_machine_hierarchy)));
        //app.add_systems(Startup, (init_scene));
       // app.add_systems(OnEnter(AppMode::Loading), (crate::states::scene_control::loading_scene, crate::states::scene_control::init_scene));
      //  app.add_systems(OnEnter(AppMode::Restarting), crate::states::scene_control::cleanup_scene);
      //  app.add_systems(OnEnter(AppMode::StandBy), crate::states::scene_control::stand_by);
       // app.add_systems(OnEnter(AppMode::Simulating), crate::states::scene_control::simulating);
      //  app.add_systems(Update, (cad_camera_controller, crate::states::scene_control::draw_gizmos));
        app.add_systems(Update, update_machine);
    }
}

fn setup_machine_hierarchy(mut commands: Commands,
                           asset_server: Res<AssetServer>,
                           mut meshes: ResMut<Assets<Mesh>>,
                           pipe_spec: Res<PipeSpecification>,
                           mut materials: ResMut<Assets<StandardMaterial>>,
                           mut machine_registers: ResMut<MachineRegisters>
) {
    let material_handle_gray = materials.add(StandardMaterial {
        base_color: Color::srgb(0.7, 0.7, 0.7),
        perceptual_roughness: 0.5,
        metallic: 0.2,
        double_sided:true,
        cull_mode: None,
        ..default()
    });


    let material_handle_red = materials.add(StandardMaterial {
        base_color: Color::srgb(1.0, 0.0, 0.0),
        perceptual_roughness: 0.5,
        metallic: 0.2,
        double_sided:true,
        cull_mode: None,
        ..default()
    });

    let material_handle_green = materials.add(StandardMaterial {
        base_color: Color::srgb(0.0, 1.0, 0.0),
        perceptual_roughness: 0.5,
        metallic: 0.2,
        ..default()
    });

    let material_handle_aqua = materials.add(StandardMaterial {
        base_color: Color::srgb(0.0, 1.0, 1.0),
        perceptual_roughness: 0.5,
        metallic: 0.2,
        ..default()
    });
    let material_handle_yellow = materials.add(StandardMaterial {
        base_color: Color::srgb(1.0, 1.0, 0.0),
        perceptual_roughness: 0.5,
        metallic: 0.2,
        ..default()
    });

    let material_handle_blue = materials.add(StandardMaterial {
        base_color: Color::srgb(0.0, 0.0, 1.0),
        perceptual_roughness: 0.5,
        metallic: 0.2,
        ..default()
    });

    let mut roller_a = generate_roller(pipe_spec.roller_a, pipe_spec.diameter);
    let roller_b = generate_roller(pipe_spec.roller_b, pipe_spec.diameter);

    //let roller_a_scaled=roller_a[0].clone().transformed_by( Transform::from_xyz(3058.0, 289.37, 154.89)).rotated_by(  Quat::from_rotation_y(PI)*Quat::from_rotation_z(PI / 2.0)).scaled_by(Vec3::splat(0.001));
    let mut roller_a_positioned=roller_a[0].clone()
        .translated_by(Vec3::new(3058.0, 289.37, 154.89))
        .rotated_by(  Quat::from_rotation_y(PI)*Quat::from_rotation_z(PI / 2.0))
        .scaled_by(Vec3::splat(0.001))
        .translated_by(Vec3::new(-0.199, 0.0, 0.015));

    let roller_a_helper_positioned=roller_a[1].clone()
        .translated_by(Vec3::new(3058.0, 289.37, 154.89))
        .rotated_by(  Quat::from_rotation_y(PI)*Quat::from_rotation_z(PI / 2.0))
        .scaled_by(Vec3::splat(0.001))
        .translated_by(Vec3::new(-0.199, 0.0, 0.015));

    let roller_a_helper_b_positioned=roller_a[2].clone()
        .translated_by(Vec3::new(3058.0, 289.37, 154.89))
        .rotated_by(  Quat::from_rotation_y(PI)*Quat::from_rotation_z(PI / 2.0))
        .scaled_by(Vec3::splat(0.001))
        .translated_by(Vec3::new(-0.131, 0.0, 0.015));

    let roller_a_support_positioned=roller_a[3].clone()
        .translated_by(Vec3::new(3058.0, 289.37, 154.89))
        .rotated_by(  Quat::from_rotation_y(PI)*Quat::from_rotation_z(PI / 2.0))
        .scaled_by(Vec3::splat(0.001))
        .translated_by(Vec3::new( -0.188, -0.304,0.015));


    let roller_b_positioned=roller_b[0].clone()
        .translated_by(Vec3::new(3058.0, 289.37, 154.89))
        .rotated_by(  Quat::from_rotation_y(PI)*Quat::from_rotation_z(PI / 2.0))
        .scaled_by(Vec3::splat(0.001))
        .translated_by(Vec3::new(-0.199, 0.0, -0.037));

    let roller_b_helper_positioned=roller_b[1].clone()
        .translated_by(Vec3::new(3058.0, 289.37, 154.89))
        .rotated_by(  Quat::from_rotation_y(PI)*Quat::from_rotation_z(PI / 2.0))
        .scaled_by(Vec3::splat(0.001))
        .translated_by(Vec3::new(-0.199, 0.0, -0.037));

    let roller_b_helper_b_positioned=roller_b[2].clone()
        .translated_by(Vec3::new(3058.0, 289.37, 154.89))
        .rotated_by(  Quat::from_rotation_y(PI)*Quat::from_rotation_z(PI / 2.0))
        .scaled_by(Vec3::splat(0.001))
        .translated_by(Vec3::new(-0.131, 0.0, -0.037));

    let roller_b_support_positioned=roller_b[3].clone()
        .translated_by(Vec3::new(3058.0, 289.37, 154.89))
        .rotated_by(  Quat::from_rotation_y(PI)*Quat::from_rotation_z(PI / 2.0))
        .scaled_by(Vec3::splat(0.001))
        .translated_by(Vec3::new( -0.188, -0.304,-0.037));


    let handle_roller_a = meshes.add(roller_a_positioned);
    let handle_roller_helper_a_a = meshes.add(roller_a_helper_positioned);
    let handle_roller_helper_a_b = meshes.add(roller_a_helper_b_positioned);
    let roller_a_support_handler = meshes.add(roller_a_support_positioned);

    let handle_roller_b = meshes.add(roller_b_positioned);
    let handle_roller_helper_b = meshes.add(roller_b_helper_positioned);
    let handle_roller_helper_b_b = meshes.add(roller_b_helper_b_positioned);
    let roller_b_support_handler = meshes.add(roller_b_support_positioned);


    let dayama_alt: SceneRoot = SceneRoot(asset_server.load(GltfAssetLabel::Scene(0).from_asset("machines/m2/dayama_alt2.glb")));
    let dayamam_kizak: SceneRoot = SceneRoot(asset_server.load(GltfAssetLabel::Scene(0).from_asset("machines/m2/dayamam_kizak3.glb")));
    let dayamam_kizak_arka: SceneRoot = SceneRoot(asset_server.load(GltfAssetLabel::Scene(0).from_asset("machines/m2/dayamam_kizak_arka3.glb")));
    let malafa: SceneRoot = SceneRoot(asset_server.load(GltfAssetLabel::Scene(0).from_asset("machines/m2/malafa.glb")));
    let mengene: SceneRoot = SceneRoot(asset_server.load(GltfAssetLabel::Scene(0).from_asset("machines/m2/mengene2.glb")));
    let mengene_alt: SceneRoot = SceneRoot(asset_server.load(GltfAssetLabel::Scene(0).from_asset("machines/m2/mengene_alt2.glb")));
    let palka2m: SceneRoot = SceneRoot(asset_server.load(GltfAssetLabel::Scene(0).from_asset("machines/m2/palka2m.glb")));
    let palkam: SceneRoot = SceneRoot(asset_server.load(GltfAssetLabel::Scene(0).from_asset("machines/m2/palkam.glb")));
    let pens: SceneRoot = SceneRoot(asset_server.load(GltfAssetLabel::Scene(0).from_asset("machines/m2/pens.glb")));
    let sasi: SceneRoot = SceneRoot(asset_server.load(GltfAssetLabel::Scene(0).from_asset("machines/m2/sasi.glb")));
    let palka3: SceneRoot = SceneRoot(asset_server.load(GltfAssetLabel::Scene(0).from_asset("machines/m2/palka3.glb")));

    commands.spawn((
        Transform::from_xyz(-3058.0, -289.37, -154.89)
            .with_rotation(Quat::from_rotation_y(PI) * Quat::from_rotation_z(PI / 2.0))
            .with_scale(Vec3::splat(1000.0)),
        Visibility::default(),
        InheritedVisibility::default(),
        MachinePartKind::Static,
        Resettable,
    )).with_children(|parent| {
        parent.spawn((
            sasi,
            Transform::IDENTITY, // Локально стоит в нуле
            MachinePartKind::SASI,
        ));

        parent.spawn((
            malafa,
            Transform::IDENTITY,  // Стартовая позиция каретки
            MachinePartKind::MALAFA,
        ));

        // --- ДОРН (Mandrel) ---
        parent.spawn((
            pens,
            Transform::from_xyz(0.00, 0.0, 0.0),  //3.525
            MachinePartKind::PENS,
        ));
        // --- Z MOVEMENT BLOCK ---
        parent.spawn((
            palka2m,
            Transform::IDENTITY,  // Стартовая позиция каретки
            MachinePartKind::PALKA2M,
        )).with_children(|z_movement| {

            // --- Y MOVEMENT BLOCK ---palkam
            z_movement.spawn((
                palkam,
                Transform::IDENTITY,  // Стартовая позиция каретки
                MachinePartKind::DayamaAlt,
            ));
            // --- Y MOVEMENT BLOCK ---
            z_movement.spawn((
                dayama_alt,
                Transform::IDENTITY,  // Стартовая позиция каретки
                MachinePartKind::PALKAM,
            )).with_children(|y_movement| {
                y_movement.spawn((
                    palka3,
                    Transform::from_xyz(0.0903719, 3.058, -0.3658858), //3.525
                    MachinePartKind::PALKA3,
                ));


                // --- Y MOVEMENT SUB BLOCK A---
                y_movement.spawn((
                    dayamam_kizak_arka,
                    Transform::IDENTITY,  // Стартовая позиция каретки
                    MachinePartKind::DayamamKizakArka,
                )).with_children(|y_subpart| {
                    // --- A SUB BLOCK---
                    y_subpart.spawn((
                        dayamam_kizak,
                        Transform::IDENTITY,
                        MachinePartKind::DayamamKizak,
                    ));

                    y_subpart.spawn((
                        Mesh3d(roller_a_support_handler),
                        MeshMaterial3d(material_handle_aqua.clone()),
                        Transform::IDENTITY,
                        MachinePartKind::DayamamKizak,
                        Visibility::Visible,
                    ));

                    y_subpart.spawn((
                        Mesh3d(roller_b_support_handler),
                        MeshMaterial3d(material_handle_yellow.clone()),
                        Transform::IDENTITY,
                        MachinePartKind::DayamamKizak,
                        Visibility::Visible,
                    ));

                });

                let rotation_pivot_offset = Vec3::new(0.0903719, 3.0580, 0.0);
                y_movement.spawn((
                    // Ставим пивот в точку вращения
                    Transform::from_translation(rotation_pivot_offset),
                    Visibility::default(),
                    InheritedVisibility::default(),
                    MachinePartKind::MengeneAlt,
                )).with_children(|pivot| {
                    // --- MAIN ROTATION BLOCK ---
                    pivot.spawn((
                        mengene_alt,
                        Transform::from_translation(-rotation_pivot_offset),
                    )).with_children(|rot_subb| {

                        // --- ROTATION SUBBLOCK ---
                        rot_subb.spawn((
                            mengene,
                            Transform::IDENTITY,  // Стартовая позиция каретки
                            MachinePartKind::MENGENE,
                        ));

                        rot_subb.spawn((
                            Mesh3d(handle_roller_a),
                            MeshMaterial3d(material_handle_red.clone()),
                            Transform::IDENTITY,
                            MachinePartKind::DornA,
                            Visibility::Visible,
                        ));
                        rot_subb.spawn((
                            Mesh3d(handle_roller_helper_a_a),
                            MeshMaterial3d(material_handle_red.clone()),
                            Transform::IDENTITY,
                            MachinePartKind::DornA,
                            Visibility::Visible,
                        ));
                        rot_subb.spawn((
                            Mesh3d(handle_roller_helper_a_b),
                            MeshMaterial3d(material_handle_green.clone()),
                            Transform::IDENTITY,
                            MachinePartKind::MENGENE,
                            Visibility::Visible,
                        ));


                        rot_subb.spawn((
                            Mesh3d(handle_roller_b),
                            MeshMaterial3d(material_handle_gray.clone()),
                            Transform::IDENTITY,
                            MachinePartKind::DornA,
                            Visibility::Visible,
                        ));
                        rot_subb.spawn((
                            Mesh3d(handle_roller_helper_b),
                            MeshMaterial3d(material_handle_gray.clone()),
                            Transform::IDENTITY,
                            MachinePartKind::DornA,
                            Visibility::Visible,
                        ));

                        rot_subb.spawn((
                            Mesh3d(handle_roller_helper_b_b),
                            MeshMaterial3d(material_handle_blue.clone()),
                            Transform::IDENTITY,
                            MachinePartKind::MENGENE,
                            Visibility::Visible,
                        ));

                    });
                });
            });
        });
    });
}

fn debug_draw_pivot(mut gizmos: Gizmos, query: Query<(&GlobalTransform, &MachinePartKind)>) {
    for (transform, part_type) in query.iter() {

        if let MachinePartKind::MengeneAlt = part_type {
            gizmos.axes(*transform, 1.5);
        }
    }
}

fn setup_start_positions(
    mut machine_registers: ResMut<MachineRegisters>,
    mut query: Query<(&mut Transform, &MachinePartKind)>,
) {
    println!("setup_start_positions {:?} {:?}",machine_registers.dx,machine_registers.mov_pusher_x);

    for (mut transform, part_type) in query.iter_mut() {
        match part_type {
            MachinePartKind::PENS => {
                transform.translation.y = (machine_registers.dx) as f32 / 1000.0 + machine_registers.mov_pusher_x as f32;//- 0.615
            }
            MachinePartKind::MengeneAlt => {
                transform.rotation = Quat::from_rotation_z(machine_registers.bend_angle as f32);
            }
            MachinePartKind::PALKA2M => {
                transform.translation.z = machine_registers.mov_z as f32;
            }
            MachinePartKind::DayamamKizakArka => {
                transform.translation.x = machine_registers.mov_static_y as f32;
            }
            MachinePartKind::DayamamKizak => {
                transform.translation.y = machine_registers.mov_static_x as f32;
            }

            MachinePartKind::MENGENE => {
                transform.translation.x = machine_registers.mov_rotated_y as f32 + machine_registers.mov_rotated_y_clamp as f32;
            }

            MachinePartKind::DayamaAlt | MachinePartKind::PALKAM => {
                transform.translation.x = machine_registers.mov_y as f32;
            }

            _ => {}
        }
    }
}

fn update_machine(
    mut machine_registers: ResMut<MachineRegisters>,
    mut query: Query<(&mut Transform, &MachinePartKind)>,
) {


    for (mut transform, part_type) in query.iter_mut() {
        match part_type {
            MachinePartKind::PENS => {
               // transform.translation.y = (machine_registers.dx+machine_registers.mov_pusher_x) as f32 / 1000.0  + machine_registers.mov_pusher_x as f32;;
                transform.translation.y = (machine_registers.dx) as f32 / 1000.0  + machine_registers.mov_pusher_x as f32;;
            }
            MachinePartKind::MengeneAlt => {
                transform.rotation = Quat::from_rotation_z(machine_registers.bend_angle as f32);
            }
            MachinePartKind::PALKA2M => {
                transform.translation.z = machine_registers.mov_z as f32;// + machine_registers.mov_z_clamp as f32;
                //transform.translation.z += machine_registers.mov_z_clamp as f32;
            }
            MachinePartKind::DayamamKizakArka => {
                transform.translation.x = machine_registers.mov_static_y as f32 + machine_registers.mov_static_y_clamp as f32;
            }
            MachinePartKind::DayamamKizak => {
                transform.translation.y = machine_registers.mov_static_x as f32;
            }

            MachinePartKind::MENGENE => {
                transform.translation.x = machine_registers.mov_rotated_y as f32 + machine_registers.mov_rotated_y_clamp as f32;
            }

            MachinePartKind::DayamaAlt | MachinePartKind::PALKAM => {
                transform.translation.x = machine_registers.mov_y as f32 + machine_registers.mov_y_clamp as f32;;
            }

            _ => {}
        }
    }
}