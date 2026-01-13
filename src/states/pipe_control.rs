use std::collections::HashSet;
use bevy::prelude::*;
use is_odd::IsOdd;
use ordered_float::OrderedFloat;
use crate::algo::analyze_stp;
use crate::algo::cnc::{byt, LRACLR};
use crate::algo::triangulation::{interpolate_by_t};
use crate::states::scene_control::{AppMode, Resettable};
use crate::states::state_machine::{MachineRegisters, RobotState};
const DEFAULT_DEMO:usize = 1;
#[derive(Resource)]
pub struct PipeSpecification {
    pub demos:Vec<Vec<u8>> ,
    pub segments: Vec<LRACLR>,
    pub diameter: f64,
    pub thickness: f64,
    pub material_changed: bool,
    pub len: f64,
    pub roller_a:f64,
    pub roller_b:f64,
    pub roller_c:f64,
}
impl Default for PipeSpecification {
    fn default() -> Self {
        let mut demos=  vec![];
        demos.push(Vec::from((include_bytes!("../files/1.stp")).as_slice()));
        demos.push(Vec::from((include_bytes!("../files/2.stp")).as_slice()));
        demos.push(Vec::from((include_bytes!("../files/3.stp")).as_slice()));
        demos.push(Vec::from((include_bytes!("../files/4.stp")).as_slice()));
        demos.push(Vec::from((include_bytes!("../files/5.stp")).as_slice()));
        demos.push(Vec::from((include_bytes!("../files/6.stp")).as_slice()));
        demos.push(Vec::from((include_bytes!("../files/7.stp")).as_slice()));
        demos.push(Vec::from((include_bytes!("../files/8.stp")).as_slice()));
        demos.push(Vec::from((include_bytes!("../files/9.stp")).as_slice()));
        demos.push(Vec::from((include_bytes!("../files/10.stp")).as_slice()));
        demos.push(Vec::from((include_bytes!("../files/11.stp")).as_slice()));
        demos.push(Vec::from((include_bytes!("../files/12.stp")).as_slice()));
        demos.push(Vec::from((include_bytes!("../files/13.stp")).as_slice()));
        demos.push(Vec::from((include_bytes!("../files/14.stp")).as_slice()));
        demos.push(Vec::from((include_bytes!("../files/15.stp")).as_slice()));
        demos.push(Vec::from((include_bytes!("../files/16.stp")).as_slice()));
        demos.push(Vec::from((include_bytes!("../files/17.stp")).as_slice()));

        let mut p=Self {
            demos:demos,
            segments: vec![],
            diameter: 50.0,
            thickness: 2.0,
            material_changed: true,
            len:200.0,
            roller_a: -10.0,
            roller_b: -10.0,
            roller_c: -10.0,
        };
        let startup_pipe=p.demos[DEFAULT_DEMO].clone();
        p.init_pipe(&startup_pipe);
        p
    }
}

impl PipeSpecification {
    pub fn init_pipe(&mut self,stp:&Vec<u8>){
        let lraclr_arr: Vec<LRACLR> = analyze_stp(&stp);
        let (tot_l,out_d)=LRACLR::total_len_out_d(&lraclr_arr);
        let f=lraclr_arr.first().unwrap();
        self.len=tot_l;
        self.diameter = f.pipe_radius*2.0;
        self.segments=lraclr_arr;

        let mut bend_radiuses = HashSet::new();

        self.segments.iter().for_each(|s|{
            if(s.clr>0.0){
                bend_radiuses.insert(OrderedFloat(s.clr));
            }

        });
        let mut sorted_vec: Vec<_> = bend_radiuses.into_iter().collect();
        sorted_vec.sort();

        let count = sorted_vec.len();
        match count {
            1 => {
                self.roller_a = sorted_vec[0].0;
                self.roller_b = sorted_vec[0].0;
                self.roller_c = sorted_vec[0].0;
            }
            2 => {
                self.roller_a = sorted_vec[0].0;
                self.roller_b = sorted_vec[1].0;
                self.roller_c = sorted_vec[1].0;
            }
            3 => {
                self.roller_a = sorted_vec[0].0;
                self.roller_b = sorted_vec[1].0;
                self.roller_c = sorted_vec[2].0;
            }
            _ => {
                self.roller_a = sorted_vec[0].0;
                self.roller_b = sorted_vec[0].0;
                self.roller_c = sorted_vec[0].0;
            }
        }
    }
}

#[derive(Component)]
pub struct PipeMesh {
    t: i64,
}
#[derive(Component)]
pub struct PipeMeshStright {
    t: i64,
}


pub struct PipeViewPlugin;

impl Plugin for PipeViewPlugin {
    fn build(&self, app: &mut App) {
        app.add_systems(OnEnter(AppMode::StandBy), setup_pipe_system);
        //app.add_systems(Startup, setup_pipe_system);
        //app.add_systems(Update, update_pipe_system);
        app.add_systems(Update,(
            update_curved,update_straignt,
            //update_curved.run_if(in_state(RobotState::Feeding).or(in_state(RobotState::Rotating)).or(in_state(RobotState::Bending)).or(in_state(RobotState::ManualSetup))  ),
            //update_straignt.run_if(in_state(RobotState::Feeding).or(in_state(RobotState::Rotating)).or(in_state(RobotState::Bending)).or(in_state(RobotState::ManualSetup)) ),

            ));
    }
}

fn update_curved( mut commands: Commands,
                  pipe_spec: Res<PipeSpecification>,
                  machine_staus: Res<MachineRegisters>,
                  mut meshes: ResMut<Assets<Mesh>>,
                  mut materials: ResMut<Assets<StandardMaterial>>,
                  mut query_pipes: Query<(&mut Transform, &mut Visibility, Option<&PipeMesh>)>,
){
    if pipe_spec.segments.is_empty() {
        return;
    }

    for (mut transform, mut visibility, pipe) in query_pipes.iter_mut() {
        match pipe {
            None => {}
            Some(p) => {
                if (p.t > (machine_staus.t * 1000.0) as i64) {
                    *visibility = Visibility::Hidden;
                } else {
                    *visibility = Visibility::Visible;
                }
                *transform = Transform::from_matrix(machine_staus.tm);
            }
        }

    }

}
fn update_straignt( mut commands: Commands,
                    pipe_spec: Res<PipeSpecification>,
                    machine_staus: Res<MachineRegisters>,
                    mut meshes: ResMut<Assets<Mesh>>,
                    mut materials: ResMut<Assets<StandardMaterial>>,
                    mut query_pipes: Query<(&mut Transform, &mut Visibility, Option<&PipeMeshStright>)>,
){
    if pipe_spec.segments.is_empty() {
        return;
    }

    for (mut transform, mut visibility, pipe_stright) in query_pipes.iter_mut() {
        match pipe_stright {
            None => {}
            Some(p) => {
                if (p.t < (machine_staus.t * 1000.0) as i64) {
                    *visibility = Visibility::Hidden;
                } else {
                    *visibility = Visibility::Visible;
                }
                transform.translation.x = machine_staus.dx as f32;
            }
        }
    }
}

fn setup_pipe_system(
    mut commands: Commands,
    pipe_spec: Res<PipeSpecification>,
    machine_staus: Res<MachineRegisters>,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
    curr_app_state: Res<State<AppMode>>,
    mut next_app_state: ResMut<NextState<AppMode>>,
    curr_robot_state: Res<State<RobotState>>,
    mut next_robot_state: ResMut<NextState<RobotState>>,
) {

    if pipe_spec.segments.is_empty() {
        return;
    }

    let (meshes_t, meshes_m_t) = interpolate_by_t(&pipe_spec.segments, &machine_staus.up_dir);

    let material_handle_gray = materials.add(StandardMaterial {
        base_color: Color::srgb(0.7, 0.7, 0.7),
        perceptual_roughness: 0.5,
        metallic: 0.2,
        ..default()
    });

    let material_handle_red = materials.add(StandardMaterial {
        base_color: Color::srgb(1.0, 0.0, 0.0),
        perceptual_roughness: 0.5,
        metallic: 0.2,
        ..default()
    });

    meshes_t.into_iter().for_each(|(m, t, id)| {
        if (id.is_odd()) {
            let handle: Handle<Mesh> = meshes.add(m);
            commands.spawn((
                Mesh3d(handle),
                MeshMaterial3d(material_handle_gray.clone()),
                PipeMesh {
                    t: t
                },
                Resettable,
                Transform::default(),
                Visibility::Hidden,
            ));
        } else {
            let handle: Handle<Mesh> = meshes.add(m);
            commands.spawn((
                Mesh3d(handle),
                MeshMaterial3d(material_handle_red.clone()),
                PipeMesh {
                    t: t
                },
                Resettable,
                Transform::default(),
                Visibility::Hidden,
            ));
        }
    });

    meshes_m_t.into_iter().for_each(|(m, t, id)| {
        let handle: Handle<Mesh> = meshes.add(m);
        commands.spawn((
            Mesh3d(handle),
            MeshMaterial3d(material_handle_gray.clone()),
            PipeMeshStright {
                t: t
            },
            Resettable,
            Transform::default(),
            Visibility::Visible,
        ));
    });

    next_robot_state.set(RobotState::PipeLoading);
}


/*fn update_pipe_system(
    mut commands: Commands,
    pipe_spec: Res<PipeSpecification>,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
    mut machine_staus: ResMut<MachineRegisters>,
    curr_state: Res<State<AppMode>>,
    mut next_state: ResMut<NextState<AppMode>>,
    mut query_pipes: Query<(&mut Transform, &mut Visibility, Option<&PipeMesh>, Option<&PipeMeshStright>,)>,
) {
    if (*curr_state.get() != AppMode::ReturnBendingHead) {
        let (tm, dx, last_dx, is_paused, curr_bend_angle) = calculate_pipe_matrix(&mut machine_staus, &pipe_spec.segments);
        machine_staus.prev_bend_angle = machine_staus.bend_angle;
        machine_staus.bend_angle = curr_bend_angle;

        machine_staus.dx = dx;

        if is_paused {
            next_state.set(AppMode::Rotating); // Пауза
        } else {
            if (*curr_state.get() == AppMode::Rotating) {
                next_state.set(AppMode::Simulating); // Старт
            }
        }

        for (mut transform, mut visibility, pipe, pipe_stright) in query_pipes.iter_mut() {
            match pipe {
                None => {}
                Some(p) => {
                    if (p.t > (machine_staus.t * 1000.0) as i64) {
                        *visibility = Visibility::Hidden;
                    } else {
                        *visibility = Visibility::Visible;
                    }
                    *transform = Transform::from_matrix(tm);
                }
            }

            match pipe_stright {
                None => {}
                Some(p) => {
                    if (p.t < (machine_staus.t * 1000.0) as i64) {
                        *visibility = Visibility::Hidden;
                    } else {
                        *visibility = Visibility::Visible;
                    }
                    transform.translation.x = dx as f32;
                }
            }
        }
    }
}*/




