use std::f64::consts::PI;
use bevy::asset::RenderAssetUsages;
use bevy::mesh::{GenerateTangentsError, Indices, Mesh, PrimitiveTopology};
use cgmath::{EuclideanSpace, InnerSpace, MetricSpace, Rad};
use truck_base::cgmath64::{Point3, Vector3, Zero};
use truck_base::tolerance::TOLERANCE;
use truck_meshalgo::prelude::{NormalFilters, OptimizingFilter, RobustMeshableShape, StructuringFilter};
use truck_meshalgo::tessellation::{MeshableShape, MeshedShape};
use truck_modeling::{builder, Shell, Solid, Vertex, Wire};
use truck_polymesh::PolygonMesh;
use truck_topology::shell;

pub fn generate_roller(r_bend: f64, d_pipe: f64) -> [Mesh; 4] {


    //let total_l=198.0;
    ///let l=total_l-r_bend;
    let r_b = r_bend;
    let r_p = d_pipe / 2.0;
    let h = 50.0;
    let c = h / 2.0;

    let fixed_len=226.0;
    let helper_1_thin=28.0;
    let helper_2_thin=fixed_len-helper_1_thin-r_b;



    let p0 = Point3::new(0.0, 3.0, 0.0);  // Точка на оси
    let p1 = Point3::new(0.0, 3.0, h); // Точка на оси
    let p2 = Point3::new(0.0, r_b, h);
    let p3 = Point3::new(0.0, r_b,  c + r_p);
    let p4 = Point3::new(0.0, r_b - r_p,  c);
    let p5 = Point3::new(0.0, r_b,  c - r_p);
    let p6 = Point3::new(0.0, r_b, 0.0);

    /*    let p0 = Point3::new (0.0, 3.0, 0.0);
        let p1 =Point3::new (0.0, 3.0, 50.0);
        let p2 =Point3::new (0.0, 84.0, 50.0);
        let p3 =Point3::new (0.0, 84.0, 41.85);
        let p4 =Point3::new (0.0, 67.15, 25.0);
        let p5 =Point3::new (0.0, 84.0, 8.15);
        let p6 =Point3::new(0.0, 84.0, 0.0);*/

    let v0 = builder::vertex(p0);
    let v1 = builder::vertex(p1);
    let v2 = builder::vertex(p2);
    let v3 = builder::vertex(p3);
    let v4 = builder::vertex(p4);
    let v5 = builder::vertex(p5);
    let v6 = builder::vertex(p6);

    /*    let wire: Wire = vec![
            builder::line(&v0, &v1),
            builder::line(&v1, &v2),
            builder::line(&v2, &v3),
            //builder::line(&v3, &v4),
            //builder::line(&v4, &v5),
            builder::circle_arc(&v3, &v5, p4),
            builder::line(&v5, &v6),
            builder::line(&v6, &v0),
        ].into();*/

    let wire: Wire = vec![
        builder::line(&v0, &v6),            // Низ
        builder::line(&v6, &v5),            // Внешняя стенка (низ)
        builder::circle_arc(&v5, &v3, p4),  // Дуга: начало v5, конец v3, через p4
        builder::line(&v3, &v2),            // Внешняя стенка (верх)
        builder::line(&v2, &v1),            // Верх
        builder::line(&v1, &v0),            // Внутренняя стенка (ось)
    ].into();

    let shell: Shell = builder::rsweep(&wire, Point3::origin(), Vector3::unit_z(), Rad(PI * 2.0), 2).into();


    let pp0 = Point3::new(0.0, r_b - helper_1_thin, 0.0);
    let pp1 = Point3::new(0.0, r_b - helper_1_thin, h);
    let pp2 = Point3::new(0.0, r_b, h);
    let pp3 = Point3::new(0.0, r_b, (h - c) + r_p);
    let pp4 = Point3::new(0.0, r_b - r_p, h - c);
    let pp5 = Point3::new(0.0, r_b, (h - c) - r_p);
    let pp6 = Point3::new(0.0, r_b, 0.0);

    let vv0 = builder::vertex(pp0);
    let vv1 = builder::vertex(pp1);
    let vv2 = builder::vertex(pp2);
    let vv3 = builder::vertex(pp3);
    let vv4 = builder::vertex(pp4);
    let vv5 = builder::vertex(pp5);
    let vv6 = builder::vertex(pp6);

    let wire_a: Wire = vec![
        builder::line(&vv0, &vv1),
        builder::line(&vv1, &vv2),
        builder::line(&vv2, &vv3),
        builder::circle_arc(&vv3, &vv5, pp4),
        builder::line(&vv5, &vv6),
        builder::line(&vv6, &vv0),
    ].into();

    let cup_a = builder::try_attach_plane(&[wire_a.inverse()]).unwrap();
    let solid_a = builder::tsweep(&cup_a, Vector3::new(60.0, 0.0, 0.0));
    let shell_a = solid_a.into_boundaries().pop().unwrap();


    let ppp0 = Point3::new(0.0, r_b , 0.0);
    let ppp1 = Point3::new(0.0, r_b, (h - c) - r_p);
    let ppp2 = Point3::new(0.0, r_b + r_p, h - c);
    let ppp3 = Point3::new(0.0, r_b, (h - c) + r_p);
    let ppp4 = Point3::new(0.0, r_b , h);
    let ppp5 = Point3::new(0.0,  r_b+helper_2_thin, h);
    let ppp6 = Point3::new(0.0, r_b+helper_2_thin, 0.0);

    let vvv0 = builder::vertex(ppp0);
    let vvv1 = builder::vertex(ppp1);
    let vvv2 = builder::vertex(ppp2);
    let vvv3 = builder::vertex(ppp3);
    let vvv4 = builder::vertex(ppp4);
    let vvv5 = builder::vertex(ppp5);
    let vvv6 = builder::vertex(ppp6);

    let wire_b: Wire = vec![
        builder::line(&vvv0, &vvv1),
        builder::circle_arc(&vvv1, &vvv3, ppp2),
        builder::line(&vvv3, &vvv4),
        builder::line(&vvv4, &vvv5),
        builder::line(&vvv5, &vvv6),
        builder::line(&vvv6, &vvv0),
    ].into();

    let cup_b = builder::try_attach_plane(&[wire_b.inverse()]).unwrap();
    let solid_b = builder::tsweep(&cup_b, Vector3::new(60.0, 0.0, 0.0));
    let shell_b = solid_b.into_boundaries().pop().unwrap();

    let solid_support = builder::tsweep(&cup_b, Vector3::new(300.0, 0.0, 0.0));
    let shell_support = solid_support.into_boundaries().pop().unwrap();


    let roller = mesher(&shell);
    let roller_helper_a = mesher(&shell_a);
    let roller_helper_b = mesher(&shell_b);
    let roller_support = mesher(&shell_support);

    [roller, roller_helper_a,roller_helper_b,roller_support]
}

pub fn generate_roller_old(r_bend: f64, d_pipe: f64) -> [Mesh; 4] {
    let fixed_len=226.0;
    let helper_1_thin=28.0;
    let helper_2_thin=28.0;

    let total_l=198.0;
    ///let l=total_l-r_bend;
    let r_b = r_bend;
    let r_p = d_pipe / 2.0;
    let h = 50.0;
    let c = h / 2.0;

   let p0 = Point3::new(0.0, 3.0, 0.0);  // Точка на оси
    let p1 = Point3::new(0.0, 3.0, h); // Точка на оси
    let p2 = Point3::new(0.0, r_b, h);
    let p3 = Point3::new(0.0, r_b,  c + r_p);
    let p4 = Point3::new(0.0, r_b - r_p,  c);
    let p5 = Point3::new(0.0, r_b,  c - r_p);
    let p6 = Point3::new(0.0, r_b, 0.0);

/*    let p0 = Point3::new (0.0, 3.0, 0.0);
    let p1 =Point3::new (0.0, 3.0, 50.0);
    let p2 =Point3::new (0.0, 84.0, 50.0);
    let p3 =Point3::new (0.0, 84.0, 41.85);
    let p4 =Point3::new (0.0, 67.15, 25.0);
    let p5 =Point3::new (0.0, 84.0, 8.15);
    let p6 =Point3::new(0.0, 84.0, 0.0);*/




    let v0 = builder::vertex(p0);
    let v1 = builder::vertex(p1);
    let v2 = builder::vertex(p2);
    let v3 = builder::vertex(p3);
    let v4 = builder::vertex(p4);
    let v5 = builder::vertex(p5);
    let v6 = builder::vertex(p6);

/*    let wire: Wire = vec![
        builder::line(&v0, &v1),
        builder::line(&v1, &v2),
        builder::line(&v2, &v3),
        //builder::line(&v3, &v4),
        //builder::line(&v4, &v5),
        builder::circle_arc(&v3, &v5, p4),
        builder::line(&v5, &v6),
        builder::line(&v6, &v0),
    ].into();*/

    let wire: Wire = vec![
        builder::line(&v0, &v6),            // Низ
        builder::line(&v6, &v5),            // Внешняя стенка (низ)
        builder::circle_arc(&v5, &v3, p4),  // Дуга: начало v5, конец v3, через p4
        builder::line(&v3, &v2),            // Внешняя стенка (верх)
        builder::line(&v2, &v1),            // Верх
        builder::line(&v1, &v0),            // Внутренняя стенка (ось)
    ].into();

    let shell: Shell = builder::rsweep(&wire, Point3::origin(), Vector3::unit_z(), Rad(PI * 2.0), 2).into();


    let pp0 = Point3::new(0.0, r_b - 28.0, 0.0);
    let pp1 = Point3::new(0.0, r_b - 28.0, h);
    let pp2 = Point3::new(0.0, r_b, h);
    let pp3 = Point3::new(0.0, r_b, (h - c) + r_p);
    let pp4 = Point3::new(0.0, r_b - r_p, h - c);
    let pp5 = Point3::new(0.0, r_b, (h - c) - r_p);
    let pp6 = Point3::new(0.0, r_b, 0.0);

    let vv0 = builder::vertex(pp0);
    let vv1 = builder::vertex(pp1);
    let vv2 = builder::vertex(pp2);
    let vv3 = builder::vertex(pp3);
    let vv4 = builder::vertex(pp4);
    let vv5 = builder::vertex(pp5);
    let vv6 = builder::vertex(pp6);

    let wire_a: Wire = vec![
        builder::line(&vv0, &vv1),
        builder::line(&vv1, &vv2),
        builder::line(&vv2, &vv3),
        builder::circle_arc(&vv3, &vv5, pp4),
        builder::line(&vv5, &vv6),
        builder::line(&vv6, &vv0),
    ].into();

    let cup_a = builder::try_attach_plane(&[wire_a.inverse()]).unwrap();
    let solid_a = builder::tsweep(&cup_a, Vector3::new(60.0, 0.0, 0.0));
    let shell_a = solid_a.into_boundaries().pop().unwrap();


    let ppp0 = Point3::new(0.0, r_b , 0.0);
    let ppp1 = Point3::new(0.0, r_b, (h - c) - r_p);
    let ppp2 = Point3::new(0.0, r_b + r_p, h - c);
    let ppp3 = Point3::new(0.0, r_b, (h - c) + r_p);
    let ppp4 = Point3::new(0.0, r_b , h);
    let ppp5 = Point3::new(0.0, total_l, h);
    let ppp6 = Point3::new(0.0, total_l, 0.0);

    let vvv0 = builder::vertex(ppp0);
    let vvv1 = builder::vertex(ppp1);
    let vvv2 = builder::vertex(ppp2);
    let vvv3 = builder::vertex(ppp3);
    let vvv4 = builder::vertex(ppp4);
    let vvv5 = builder::vertex(ppp5);
    let vvv6 = builder::vertex(ppp6);

    let wire_b: Wire = vec![
        builder::line(&vvv0, &vvv1),
        builder::circle_arc(&vvv1, &vvv3, ppp2),
        builder::line(&vvv3, &vvv4),
        builder::line(&vvv4, &vvv5),
        builder::line(&vvv5, &vvv6),
        builder::line(&vvv6, &vvv0),
    ].into();

    let cup_b = builder::try_attach_plane(&[wire_b.inverse()]).unwrap();
    let solid_b = builder::tsweep(&cup_b, Vector3::new(60.0, 0.0, 0.0));
    let shell_b = solid_b.into_boundaries().pop().unwrap();

    let solid_support = builder::tsweep(&cup_b, Vector3::new(300.0, 0.0, 0.0));
    let shell_support = solid_support.into_boundaries().pop().unwrap();


    let roller = mesher(&shell);
    let roller_helper_a = mesher(&shell_a);
    let roller_helper_b = mesher(&shell_b);
    let roller_support = mesher(&shell_support);

    [roller, roller_helper_a,roller_helper_b,roller_support]
}
fn mesher(shell: &Shell) -> Mesh {

    let tolerance = 0.5;

    let mut truck_mesh: PolygonMesh = shell.robust_triangulation(tolerance).to_polygon();
    //truck_mesh.add_smooth_normals(0.8, true);
    truck_mesh.add_smooth_normals(1.0, true);


    let mut positions: Vec<[f32; 3]> = vec![];
    let mut normals: Vec<[f32; 3]> = vec![];
    let mut indices: Vec<u32> = Vec::new();
    let mut uvs: Vec<[f32; 2]> = vec![];

    truck_mesh.tri_faces().into_iter().for_each(|vtx| {
        let pi0 = vtx[0].pos;
        let pi1 = vtx[1].pos;
        let pi2 = vtx[2].pos;

        let ni0 = vtx[0].nor.unwrap();
        let ni1 = vtx[1].nor.unwrap();
        let ni2 = vtx[2].nor.unwrap();

        let uvi0 = vtx[0].uv.unwrap();
        let uvi1 = vtx[1].uv.unwrap();
        let uvi2 = vtx[2].uv.unwrap();

        let v0 = truck_mesh.positions()[pi0];
        let v1 = truck_mesh.positions()[pi1];
        let v2 = truck_mesh.positions()[pi2];

        let n0 = truck_mesh.normals()[ni0].normalize();
        let n1 = truck_mesh.normals()[ni1].normalize();
        let n2 = truck_mesh.normals()[ni2].normalize();

        let uv0 = truck_mesh.uv_coords()[uvi0].normalize();
        let uv1 = truck_mesh.uv_coords()[uvi1].normalize();
        let uv2 = truck_mesh.uv_coords()[uvi2].normalize();

        positions.push([v0.x as f32, v0.y as f32, v0.z as f32]);
        indices.push(positions.len() as u32 - 1);
        positions.push([v1.x as f32, v1.y as f32, v1.z as f32]);
        indices.push(positions.len() as u32 - 1);
        positions.push([v2.x as f32, v2.y as f32, v2.z as f32]);
        indices.push(positions.len() as u32 - 1);

        normals.push([n0.x as f32, n0.y as f32, n0.z as f32]);
        normals.push([n1.x as f32, n1.y as f32, n1.z as f32]);
        normals.push([n2.x as f32, n2.y as f32, n2.z as f32]);

        uvs.push([uv0.x as f32, uv0.y as f32]);
        uvs.push([uv1.x as f32, uv1.y as f32]);
        uvs.push([uv2.x as f32, uv2.y as f32]);
    });


    // 4. Создаем Bevy Mesh
    let mut bevy_mesh = Mesh::new(PrimitiveTopology::TriangleList, RenderAssetUsages::default());
    bevy_mesh.insert_attribute(Mesh::ATTRIBUTE_POSITION, positions);
    bevy_mesh.insert_attribute(Mesh::ATTRIBUTE_NORMAL, normals);
    bevy_mesh.insert_attribute(Mesh::ATTRIBUTE_UV_0, uvs);
    bevy_mesh.insert_indices(Indices::U32(indices));
    //bevy_mesh.with_computed_normals()
    bevy_mesh
}


pub fn truck_to_bevy_mesh_bottle() -> Mesh {
    let b = test_rotate2();
    //let b=  bottle(1.4, 1.0, 0.6);
    let s = truck_to_bevy_mesh2(&b);
    s
}

pub fn truck_to_bevy_mesh2(solid: &Shell) -> Mesh {
    let tolerance = 0.005; // Хорошая точность для мелких деталей

    // 1. Превращаем Solid в PolygonMesh
    let mut truck_mesh: PolygonMesh = solid.triangulation(tolerance).to_polygon();
    truck_mesh.add_smooth_normals(0.8, true);


    let mut positions: Vec<[f32; 3]> = vec![];
    let mut normals: Vec<[f32; 3]> = vec![];
    let mut indices: Vec<u32> = Vec::new();
    let mut uvs: Vec<[f32; 2]> = vec![];

    truck_mesh.tri_faces().into_iter().for_each(|vtx| {
        let pi0 = vtx[0].pos;
        let pi1 = vtx[1].pos;
        let pi2 = vtx[2].pos;

        let ni0 = vtx[0].nor.unwrap();
        let ni1 = vtx[1].nor.unwrap();
        let ni2 = vtx[2].nor.unwrap();

        let uvi0 = vtx[0].uv.unwrap();
        let uvi1 = vtx[1].uv.unwrap();
        let uvi2 = vtx[2].uv.unwrap();

        let v0 = truck_mesh.positions()[pi0];
        let v1 = truck_mesh.positions()[pi1];
        let v2 = truck_mesh.positions()[pi2];

        let n0 = truck_mesh.normals()[ni0].normalize();
        let n1 = truck_mesh.normals()[ni1].normalize();
        let n2 = truck_mesh.normals()[ni2].normalize();

        let uv0 = truck_mesh.uv_coords()[uvi0].normalize();
        let uv1 = truck_mesh.uv_coords()[uvi1].normalize();
        let uv2 = truck_mesh.uv_coords()[uvi2].normalize();


        positions.push([v0.x as f32, v0.y as f32, v0.z as f32]);
        indices.push(positions.len() as u32 - 1);
        positions.push([v1.x as f32, v1.y as f32, v1.z as f32]);
        indices.push(positions.len() as u32 - 1);
        positions.push([v2.x as f32, v2.y as f32, v2.z as f32]);
        indices.push(positions.len() as u32 - 1);

        normals.push([n0.x as f32, n0.y as f32, n0.z as f32]);
        normals.push([n1.x as f32, n1.y as f32, n1.z as f32]);
        normals.push([n2.x as f32, n2.y as f32, n2.z as f32]);

        uvs.push([uv0.x as f32, uv0.y as f32]);
        uvs.push([uv1.x as f32, uv1.y as f32]);
        uvs.push([uv2.x as f32, uv2.y as f32]);
    });


    // 4. Создаем Bevy Mesh
    let mut bevy_mesh = Mesh::new(PrimitiveTopology::TriangleList, RenderAssetUsages::default());
    bevy_mesh.insert_attribute(Mesh::ATTRIBUTE_POSITION, positions);
    //bevy_mesh.insert_attribute(Mesh::ATTRIBUTE_NORMAL, normals);
    //bevy_mesh.insert_attribute(Mesh::ATTRIBUTE_UV_0, uvs);
    bevy_mesh.insert_indices(Indices::U32(indices));
    bevy_mesh.with_computed_normals()

    /*  match bevy_mesh.generate_tangents() {
          Ok(m) => {
              bevy_mesh
          }
          Err(e) => {
              println!("TANGENTS E {:?}", e);
              bevy_mesh
          }
      }*/
}

pub fn truck_to_bevy_mesh3(solid: &Shell) -> Mesh {
    let tolerance = 0.005; // Хорошая точность для мелких деталей

    // 1. Превращаем Solid в PolygonMesh
    let mut truck_mesh: PolygonMesh = solid.triangulation(tolerance).to_polygon();
    truck_mesh.add_smooth_normals(0.8, true);


    let mut positions: Vec<[f32; 3]> = vec![];
    let mut normals: Vec<[f32; 3]> = vec![];
    let mut indices: Vec<u32> = Vec::new();
    let mut uvs: Vec<[f32; 2]> = vec![];

    for face in truck_mesh.tri_faces() {
        for vertex in face {
            let p = truck_mesh.positions()[vertex.pos];
            positions.push([p.x as f32, p.y as f32, p.z as f32]);
            let mut n = truck_mesh.normals()[vertex.nor.unwrap() as usize];
            n.normalize();
            normals.push([n.x as f32, n.y as f32, n.z as f32]);
            indices.push((positions.len() - 1) as u32);
            let uv = truck_mesh.uv_coords()[vertex.uv.unwrap()];
            uvs.push([uv.x as f32, uv.y as f32]);
        }
    }


    // 4. Создаем Bevy Mesh
    let mut bevy_mesh = Mesh::new(PrimitiveTopology::TriangleList, RenderAssetUsages::default());
    bevy_mesh.insert_attribute(Mesh::ATTRIBUTE_POSITION, positions);
    bevy_mesh.insert_attribute(Mesh::ATTRIBUTE_NORMAL, normals);
    bevy_mesh.insert_attribute(Mesh::ATTRIBUTE_UV_0, uvs);
    bevy_mesh.insert_indices(Indices::U32(indices));
    match bevy_mesh.generate_tangents() {
        Ok(m) => {
            bevy_mesh
        }
        Err(e) => {
            println!("TANGENTS E {:?}", e);
            bevy_mesh
        }
    }
}

pub fn truck_to_bevy_mesh(solid: &Solid) -> Mesh {
    let tolerance = 0.01; // Хорошая точность для мелких деталей

    // 1. Превращаем Solid в PolygonMesh
    let mut truck_mesh: PolygonMesh = solid.triangulation(tolerance).to_polygon();
    truck_mesh.add_smooth_normals(1.0, true);


    let mut positions: Vec<[f32; 3]> = vec![];
    let mut normals: Vec<[f32; 3]> = vec![];
    let mut indices: Vec<u32> = Vec::new();
    let mut uvs: Vec<[f32; 2]> = vec![];

    for face in truck_mesh.tri_faces() {
        for vertex in face {
            let p = truck_mesh.positions()[vertex.pos];
            positions.push([p.x as f32, p.y as f32, p.z as f32]);
            let n = truck_mesh.normals()[vertex.nor.unwrap() as usize];
            normals.push([n.x as f32, n.y as f32, n.z as f32]);
            indices.push((positions.len() - 1) as u32);
            let uv = truck_mesh.uv_coords()[vertex.uv.unwrap()];
            uvs.push([uv.x as f32, uv.y as f32]);
        }
    }


    // 4. Создаем Bevy Mesh
    let mut bevy_mesh = Mesh::new(PrimitiveTopology::TriangleList, RenderAssetUsages::default());
    bevy_mesh.insert_attribute(Mesh::ATTRIBUTE_POSITION, positions);
    bevy_mesh.insert_attribute(Mesh::ATTRIBUTE_NORMAL, normals);
    bevy_mesh.insert_attribute(Mesh::ATTRIBUTE_UV_0, uvs);
    bevy_mesh.insert_indices(Indices::U32(indices));
    let bevy_mesh2 = bevy_mesh.with_generated_tangents().unwrap();
    bevy_mesh2
}

pub fn truck_to_bevy_mesh_strange(solid: &Solid) -> Mesh {
    let tolerance = 0.005; // Хорошая точность для мелких деталей

    // 1. Превращаем Solid в PolygonMesh
    let mut truck_mesh: PolygonMesh = solid.triangulation(tolerance).to_polygon();
    truck_mesh.add_smooth_normals(TOLERANCE * 10.0, true);

    let mut positions: Vec<[f32; 3]> = vec![];
    truck_mesh.positions().iter().for_each(|p| {
        positions.push([
            p.x as f32,
            p.y as f32,
            p.z as f32,
        ]);
    });
    let mut normals: Vec<[f32; 3]> = vec![];
    /*    truck_mesh.normals().iter().for_each(|n|{
            normals.push([
                n.x as f32,
                n.y as f32,
                n.z as f32,
            ]);
        });*/
    let mut indices: Vec<u32> = Vec::new();
    for face in truck_mesh.face_iter() {
        for index in face {
            indices.push(index.pos as u32);
            let n = truck_mesh.normals().get(index.nor.unwrap()).unwrap();
            normals.push([
                n.x as f32,
                n.y as f32,
                n.z as f32,
            ]);
        }
    }


    truck_mesh.face_iter().for_each(|face| {
        for index in face {
            indices.push(index.pos.clone() as u32);
        }
    });

    // 4. Создаем Bevy Mesh
    let mut bevy_mesh = Mesh::new(PrimitiveTopology::TriangleList, RenderAssetUsages::default());
    bevy_mesh.insert_attribute(Mesh::ATTRIBUTE_POSITION, positions);
    bevy_mesh.insert_attribute(Mesh::ATTRIBUTE_NORMAL, normals);
    bevy_mesh.insert_indices(Indices::U32(indices));
    bevy_mesh
}

pub fn truck_to_bevy_mesh_old(solid: &Solid) -> Mesh {
    let tolerance = 0.005; // Хорошая точность для мелких деталей

    // 1. Превращаем Solid в PolygonMesh
    let mut truck_mesh: PolygonMesh = solid.triangulation(tolerance).to_polygon();
    truck_mesh.add_smooth_normals(0.1, true);

    // 2. "Разворачиваем" меш.
    // Expander превращает сложную индексацию в плоский список вершин.
    // Это решает проблему несовпадения индексов нормалей и позиций.
    let expanded = truck_mesh.expands(|attr| (attr.position, attr.normal.unwrap_or(Vector3::zero())));

    // 3. Собираем данные.
    // В expanded_mesh вершины идут подряд треугольниками (0-1-2, 3-4-5...)
    let mut positions: Vec<[f32; 3]> = vec![]; // Vec::with_capacity(expanded.attributes().len());
    let mut normals: Vec<[f32; 3]> = vec![];; //Vec::with_capacity(expanded.attributes().len());

    for (v, n) in expanded.attributes() {
        positions.push([
            v.x as f32,
            v.y as f32,
            v.z as f32,
        ]);
        normals.push([
            n.x as f32,
            n.y as f32,
            n.z as f32,
        ]);
    }

    let mut indices: Vec<u32> = Vec::new();

    for face in expanded.faces().face_iter() {
        // Каждая грань - это набор индексов, указывающих на массив attributes
        for index in face {
            indices.push(index.clone() as u32);
        }
    }
    // 4. Создаем Bevy Mesh
    let mut bevy_mesh = Mesh::new(PrimitiveTopology::TriangleList, RenderAssetUsages::default());
    bevy_mesh.insert_attribute(Mesh::ATTRIBUTE_POSITION, positions);
    bevy_mesh.insert_attribute(Mesh::ATTRIBUTE_NORMAL, normals);
    bevy_mesh.insert_indices(Indices::U32(indices));
    bevy_mesh
}

fn body_shell(bottom: f64, height: f64, width: f64, thickness: f64) -> Shell {
    let vertex0 = builder::vertex(Point3::new(-width / 2.0, bottom, thickness / 4.0));
    let vertex1 = builder::vertex(Point3::new(width / 2.0, bottom, thickness / 4.0));
    let transit = Point3::new(0.0, bottom, thickness / 2.0);
    let arc0 = builder::circle_arc(&vertex0, &vertex1, transit);
    let arc1 = builder::rotated(&arc0, Point3::origin(), Vector3::unit_y(), Rad(PI));
    let face = builder::homotopy(&arc0, &arc1.inverse());
    let solid = builder::tsweep(&face, Vector3::new(0.0, height, 0.0));
    solid.into_boundaries().pop().unwrap()
}

fn cylinder(bottom: f64, height: f64, radius: f64) -> Shell {
    let vertex = builder::vertex(Point3::new(0.0, bottom, radius));
    let circle = builder::rsweep(&vertex, Point3::origin(), Vector3::unit_y(), Rad(7.0), 2);
    let disk = builder::try_attach_plane(&[circle]).unwrap();
    let solid = builder::tsweep(&disk, Vector3::new(0.0, height, 0.0));
    solid.into_boundaries().pop().unwrap()
}

fn grue_body_neck(body: &mut Shell, neck: Shell) {
    let body_seiling = body.last_mut().unwrap();
    let wire = neck[0].boundaries()[0].clone();
    body_seiling.add_boundary(wire);
    body.extend(neck.into_iter().skip(1));
}

fn bottle(height: f64, width: f64, thickness: f64) -> Solid {
    let mut body = body_shell(0.0, height, width, thickness);
    let neck = cylinder(height, height / 10.0, thickness / 4.0);
    grue_body_neck(&mut body, neck);

    let eps = height / 50.0;
    let mut inner_body = body_shell(
        eps,
        height - 2.0 * eps,
        width - 2.0 * eps,
        thickness - 2.0 * eps,
    );
    let inner_neck = cylinder(height - eps, height / 10.0 + eps, thickness / 4.0 - eps);
    grue_body_neck(&mut inner_body, inner_neck);

    let inner_hat = inner_body.pop().unwrap();
    let wire = inner_hat.into_boundaries()[0].inverse();
    body.last_mut().unwrap().add_boundary(wire);
    body.extend(inner_body.into_iter().map(|face| face.inverse()));
    Solid::new(vec![body])
}

fn test_rotate() -> Solid {
    let p0 = Point3::new(0.0, 0.0, -4.0); // Точка на оси

    let p1 = Point3::new(1.0, 0.0, -4.0);

    let p1_5 = Point3::new(1.5, 0.0, -0.0);

    let p2 = Point3::new(1.0, 0.0, 4.0);

    let p3 = Point3::new(0.0, 0.0, 4.0);  // Точка на оси

    let v0 = builder::vertex(p0);
    let v1 = builder::vertex(p1);
    let v1_5 = builder::vertex(p1_5);
    let v2 = builder::vertex(p2);
    let v3 = builder::vertex(p3);
    // Собираем замкнутый контур
    let wire: Wire = vec![
        builder::line(&v0, &v1), // Низ
        builder::line(&v1, &v1_5), // Боковая стенка
        builder::line(&v1_5, &v2), // Боковая стенка
        builder::line(&v2, &v3), // Верх
        builder::line(&v3, &v0), // Ось (замыкающая линия)
    ].into();


    let shell: Shell = builder::rsweep(&wire.inverse(), Point3::origin(), Vector3::unit_z(), Rad(7.0), 2).into();
    let solid: Solid = Solid::new(vec![shell]);
    solid
}

fn test_rotate2() -> Shell {
    let p0 = Point3::new(0.0, 0.0, -4.0); // Точка на оси

    let p1 = Point3::new(1.0, 0.0, -4.0);

    let p1_5 = Point3::new(1.5, 0.0, -0.0);

    let p2 = Point3::new(1.0, 0.0, 4.0);

    let p3 = Point3::new(0.0, 0.0, 4.0);  // Точка на оси

    let v0 = builder::vertex(p0);
    let v1 = builder::vertex(p1);
    let v1_5 = builder::vertex(p1_5);
    let v2 = builder::vertex(p2);
    let v3 = builder::vertex(p3);
    // Собираем замкнутый контур
    let wire: Wire = vec![
        builder::line(&v0, &v1), // Низ
        builder::line(&v1, &v1_5), // Боковая стенка
        builder::line(&v1_5, &v2), // Боковая стенка
        builder::line(&v2, &v3), // Верх
        builder::line(&v3, &v0), // Ось (замыкающая линия)
    ].into();


    let shell: Shell = builder::rsweep(&wire.inverse(), Point3::origin(), Vector3::unit_z(), Rad(7.0), 2).into();

    shell
}



