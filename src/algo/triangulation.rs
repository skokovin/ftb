use cgmath::{InnerSpace, Point3, Quaternion, Rad, Rotation3, Vector3};
use glam::Vec3;
use std::f32::consts::{PI, TAU};
use std::fs::File;
use std::io::{BufWriter, Write};
use bevy::asset::RenderAssetUsages;
use bevy::mesh::{Indices, PrimitiveTopology, VertexAttributeValues};
use bevy::pbr::LightEntity::Point;
use bevy::prelude::Mesh;
use crate::algo::cnc::{byt, tot_pipe_len, LRACLR};

pub const T_INCREMENTS: i64=1000;
pub fn interpolate_by_t(cmnd: &Vec<LRACLR>, up_dir: &Vector3<f64>) -> (Vec<(Mesh, i64, u64)>, Vec<(Mesh, i64, u64)>) {
    let num_segments = 64;
    let len: f64 = tot_pipe_len(cmnd);
    let pipe_radius = cmnd[0].pipe_radius as f32;
    let mut ret: Vec<(Mesh, i64, u64)> = vec![];
    let mut ret2: Vec<(Mesh, i64, u64)> = vec![];
    let step: i64 = T_INCREMENTS;
    let mut pt_a: Point3<f64> = Point3::new(0.0, 0.0, 0.0);
    let mut pt_m_a: Point3<f64> = Point3::new(0.0, 0.0, 0.0);
    for ti in 1..step {
        let t = ti as f64 / step as f64;

        let ( (pt, x_dir, y_dir, z_dir, rot_deg, id, cp, l,theta, bend_radius)) = byt(t, cmnd, up_dir);
        let mesh = triangulate_pipe(&pt_a, &pt, &cp, pipe_radius, num_segments);
        ret.push((mesh, ti, id));
        pt_a = pt;
        let pt_m_b: Point3<f64> = Point3::new(-len * t, 0.0, 0.0);
        let mesh2 = triangulate_pipe(&pt_m_a, &pt_m_b, &None, pipe_radius, num_segments);
        ret2.push((mesh2, ti, 99999));
        pt_m_a = pt_m_b;
    }
    (ret, ret2)
}


pub fn triangulate_pipe(pt1_: &Point3<f64>, pt2_: &Point3<f64>, cp_: &Option<Point3<f64>>, radius:f32, num_segments:u32) -> Mesh{

    let pt1: Point3<f32>= Point3::new(pt1_.x as f32, pt1_.y as f32, pt1_.z as f32);
    let pt2: Point3<f32>= Point3::new(pt2_.x as f32, pt2_.y as f32, pt2_.z as f32);


    let mut vertices: Vec<glam::Vec3> = Vec::new();
    let mut indices: Vec<u32> = Vec::new();
    let mut normals: Vec<glam::Vec3> = Vec::new();

    let mut indice:u32=0;

    let (up, fwd_a, left_a, fwd_b, left_b)={
        match cp_ {
            None => {
                let fwd1: Vector3<f32> =(pt2-pt1).normalize();
                let left1= perpendicular_rand_dir(&fwd1).normalize();
                let up=left1.cross(fwd1).normalize();

                (up,fwd1.clone(),left1.clone(),fwd1.clone(),left1.clone())
            }
            Some(cp) => {
                let center_p: Point3<f32>= Point3::new(cp.x as f32, cp.y as f32, cp.z as f32);
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

        let v1 = pt_a1 - pt_b0;
        let v2 = pt_a0 - pt_b0;
        let face_norm1 = v1.cross(v2).normalize();
        let glam_norm1 = vec_from_norm(face_norm1)*-1.0;

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