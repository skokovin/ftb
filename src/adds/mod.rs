use bevy::asset::RenderAssetUsages;
use bevy::mesh::{Indices, PrimitiveTopology, VertexAttributeValues};
use bevy::prelude::{Mesh, Vec3};


pub mod line;


fn create_torus_mesh(radius: f32, tube_radius: f32, segments: usize, tube_segments: usize) -> Mesh {
    let mut mesh = Mesh::new(PrimitiveTopology::TriangleList,  RenderAssetUsages::default() );

    // Generate vertices
    let mut vertices: Vec<Vec3> = Vec::new();
    let mut normals: Vec<Vec3> = Vec::new();
    let mut uvs: Vec<[f32; 2]> = Vec::new();

    for i in 0..=segments {
        let segment_angle = 2.0 * std::f32::consts::PI * i as f32 / segments as f32;
        let segment_pos = Vec3::new(
            segment_angle.cos() * radius,
            0.0,
            segment_angle.sin() * radius,
        );

        for j in 0..=tube_segments {
            let tube_angle = 2.0 * std::f32::consts::PI * j as f32 / tube_segments as f32;
            let tube_pos = Vec3::new(
                tube_angle.cos(),
                tube_angle.sin(),
                0.0,
            );

            let normal = segment_pos.normalize() * tube_pos.x + Vec3::Y * tube_pos.y;
            let vertex = segment_pos + normal * tube_radius;

            vertices.push(vertex);
            normals.push(normal.normalize());
            uvs.push([
                i as f32 / segments as f32,
                j as f32 / tube_segments as f32,
            ]);
        }
    }

    // Generate indices
    let mut indices: Vec<u32> = Vec::new();
    for i in 0..segments {
        for j in 0..tube_segments {
            let a = (i * (tube_segments + 1)) + j;
            let b = ((i + 1) * (tube_segments + 1)) + j;
            let c = ((i + 1) * (tube_segments + 1)) + j + 1;
            let d = (i * (tube_segments + 1)) + j + 1;

            indices.push(a as u32);
            indices.push(b as u32);
            indices.push(d as u32);

            indices.push(b as u32);
            indices.push(c as u32);
            indices.push(d as u32);
        }
    }

    // Set mesh attributes
    mesh.insert_attribute(
        Mesh::ATTRIBUTE_POSITION,
        VertexAttributeValues::Float32x3(
            vertices.iter().map(|v| [v.x, v.y, v.z]).collect(),
        ),
    );

    mesh.insert_attribute(
        Mesh::ATTRIBUTE_NORMAL,
        VertexAttributeValues::Float32x3(
            normals.iter().map(|n| [n.x, n.y, n.z]).collect(),
        ),
    );

    mesh.insert_attribute(
        Mesh::ATTRIBUTE_UV_0,
        VertexAttributeValues::Float32x2(uvs),
    );

    mesh.insert_indices(Indices::U32(indices));

    mesh
}