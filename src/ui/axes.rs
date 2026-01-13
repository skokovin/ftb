use bevy::app::{App, Plugin};
use bevy::prelude::*;
use crate::render::line::{LineList, LineMaterial};

pub struct AxesPlugin;

impl Plugin for AxesPlugin{
    fn build(&self, app: &mut App) {
        app.add_systems(Startup,setup_axes);
    }
}

fn setup_axes(mut commands: Commands, mut meshes: ResMut<Assets<Mesh>>,mut lines_materials: ResMut<Assets<LineMaterial>>) {
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

