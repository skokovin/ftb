use cgmath::{InnerSpace, Point3, Quaternion, Rad, Rotation3, Vector3};
use glam::Vec3;
use std::f32::consts::{PI, TAU};
use std::fs::File;
use std::io::{BufWriter, Write};
use bevy::asset::RenderAssetUsages;
use bevy::mesh::{Indices, PrimitiveTopology, VertexAttributeValues};
use bevy::pbr::LightEntity::Point;
use bevy::prelude::Mesh;

pub fn triangulate_pipe(pt1_: &Point3<f64>, pt2_: &Point3<f64>, cp: &Option<Point3<f32>>, radius:f32, num_segments:u32) -> Mesh{

    let pt1: Point3<f32>= Point3::new(pt1_.x as f32, pt1_.y as f32, pt1_.z as f32);
    let pt2: Point3<f32>= Point3::new(pt2_.x as f32, pt2_.y as f32, pt2_.z as f32);

    let mut vertices: Vec<glam::Vec3> = Vec::new();
    let mut indices: Vec<u32> = Vec::new();
    let mut normals: Vec<glam::Vec3> = Vec::new();

    let mut indice:u32=0;

    let (up, fwd_a, left_a, fwd_b, left_b)={
        match cp {
            None => {
                let fwd1: Vector3<f32> =(pt2-pt1).normalize();
                let left1= perpendicular_rand_dir(&fwd1).normalize();
                let up=left1.cross(fwd1).normalize();

                (up,fwd1.clone(),left1.clone(),fwd1.clone(),left1.clone())
            }
            Some(center_p) => {
                let up: Vector3<f32> = {
                    let a = pt1 - center_p;
                    let b = pt2 - center_p;
                    a.cross(b).normalize()
                };

                let left1=(center_p-pt1).normalize();
                let fwd1: Vector3<f32> =left1.cross(up).normalize();
                let left2=(center_p-pt2).normalize();
                let fwd2: Vector3<f32> =left2.cross(up).normalize();
                (up,fwd1,left1,fwd2,left2)
            }
        }
    };

    let mut pt_a0=pt1+ left_a *radius;
    let mut pt_b0=pt2+ left_b *radius;

    for i in 1..=num_segments {
        //let t = 1 as f32 / num_segments as f32;
        let t = i as f32 / num_segments as f32;
        let angle =Rad( t * TAU);
        //let angle =Rad( t * PI);



        let rotation_a: Quaternion<f32> = Quaternion::from_axis_angle(fwd_a,angle);
        let left1_a = rotation_a * left_a;
        let pt_a1=pt1+left1_a*radius;

        let rotation_b: Quaternion<f32> = Quaternion::from_axis_angle(fwd_b, angle);
        let left1_b = rotation_b * left_b;
        let pt_b1: Point3<f32> =pt2+left1_b*radius;


        // --- Треугольник 1 (b0, a1, a0) ---

        // Вычисляем нормаль грани
        let v1 = pt_a1 - pt_b0;
        let v2 = pt_a0 - pt_b0;
        let face_norm1 = v1.cross(v2).normalize();
        let glam_norm1 = vec_from_norm(face_norm1)*-1.0;

        // Вычисляем нормаль грани
        let v1 = pt_b1 - pt_b0;
        let v2 = pt_a1 - pt_b0;
        let face_norm2 = v1.cross(v2).normalize();
        let glam_norm2 = vec_from_norm(face_norm2)*-1.0;




        vertices.push(vec_from_pt(pt_a1));
        normals.push(glam_norm1);
        indices.push(indice);
        indice=indice+1;

        vertices.push(vec_from_pt(pt_b0));
        normals.push(glam_norm1);
        indices.push(indice);
        indice=indice+1;

        vertices.push(vec_from_pt(pt_a0));
        normals.push(glam_norm1);
        indices.push(indice);
        indice=indice+1;


        vertices.push(vec_from_pt(pt_b1));
        normals.push(glam_norm2);
        indices.push(indice);
        indice=indice+1;

        vertices.push(vec_from_pt(pt_b0));
        normals.push(glam_norm2);
        indices.push(indice);
        indice=indice+1;

        vertices.push(vec_from_pt(pt_a1));
        normals.push(glam_norm2);
        indices.push(indice);
        indice=indice+1;

        pt_a0=pt_a1;
        pt_b0=pt_b1;
    }

    let mut mesh = Mesh::new(PrimitiveTopology::TriangleList,  RenderAssetUsages::default() );

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



    mesh.insert_indices(Indices::U32(indices));



    mesh

}

fn perpendicular_rand_dir(src:&Vector3<f32>)->Vector3<f32>{
    //https://math.stackexchange.com/questions/137362/how-to-find-perpendicular-vector-to-another-vector
    let nx=src.z.copysign(src.x);
    let ny=src.z.copysign(src.y);
    let nz=-(src.x.abs()+src.y.abs()).copysign(src.z);
    Vector3::new(nx, ny, nz)
}
fn vec_from_pt(v:Point3<f32>)->glam::Vec3{
    glam::Vec3::new(v.x,v.y,v.z)
}
fn vec_from_norm(v: Vector3<f32>)->glam::Vec3{
    glam::Vec3::new(v.x,v.y,v.z)
}

fn export_to_pt_str(points: &Vec<Vec3>, counter: &str) {
    let path = format!("d:\\pipe_project\\{}.pt", counter);
    match File::create(path) {
        Ok(file) => {
            let mut writer = BufWriter::new(file);
            points.iter().for_each(|p| {
                let line = format!("{} {} {} \r\n", p.x, p.y, p.z);
                writer.write_all(&line.as_bytes());
            });
            writer.flush();
        }
        Err(_) => {}
    }
}