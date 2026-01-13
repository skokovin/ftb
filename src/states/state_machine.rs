use std::cmp::PartialEq;
use bevy::prelude::*;
use is_odd::IsOdd;
use truck_base::cgmath64::Vector3;
use crate::algo::cnc::{byt, LRACLR};
use crate::states::pipe_control::PipeSpecification;
use crate::states::scene_control::AppMode;
use crate::ui::camera::cad_camera_controller;

#[derive(States, Debug, Clone, Copy, Eq, PartialEq, Hash, Default)]
pub enum RobotState {
    #[default]
    Idle,
    PipeLoading, // Робот стоит, ничего не делает
    PipeLoaded,    // "Загрузить трубу"
    Unclamping,     // "Разжать суппорт"
    Feeding,        // "Продвинуть вперед" (Move Z)
    Clamping,       // "Зажать суппорт"
    Bending,        // "Гнуть"
    Rotating,       // "Поворачивать" (Rotate Y)
    ManualSetup,
    Finished,       // Конец программы
}
#[derive(Resource)]
pub struct MachineRegisters {
    pub current_id: u64,
    pub carriage_pos: f32, // Позиция каретки (мм)
    pub rotation_y: f32,   // Поворот трубы (градусы)
    pub bend_angle: f64,   // Угол гиба (градусы)
    pub prev_bend_angle: f64,
    pub t: f32,
    pub dt: f32,
    pub playback_speed: f32,
    pub up_dir: cgmath::Vector3<f64>,
    pub bending_state: u32,
    pub current_bend_radius: f64,
    pub rot_angle: f64,
    pub rot_step: f64,
    pub dx: f64,
    pub last_dx: f64,
    pub tm: Mat4,
    pub mov_y: f64,
    pub mov_y_clamp: f64,
    pub mov_z: f64,
    pub mov_to_z: f64,
    pub mov_static_y: f64,
    pub mov_static_y_clamp: f64,
    pub mov_static_x: f64,
    pub mov_rotated_y: f64,
    pub mov_rotated_y_clamp: f64,
    pub mov_pusher_x: f64,
    pub clamp: f64,
    pub robot_state: RobotState,

}
impl Default for MachineRegisters {
    fn default() -> Self {
        Self {
            current_id: 0,
            carriage_pos: 0.0,
            rotation_y: 0.0,
            bend_angle: 0.0,
            prev_bend_angle: 0.0,
            t: 0.0,
            dt: 0.0,
            playback_speed: 0.0,
            up_dir: Vector3::new(0.0, 0.0, 1.0),
            bending_state: 0,
            current_bend_radius: 0.0,
            rot_angle: 0.0,
            rot_step: 0.0,
            dx: 0.0,
            last_dx: 0.0,
            tm: Mat4::default(),
            mov_y: 0.0,
            mov_y_clamp: 0.0,
            mov_z: 0.01,
            mov_to_z: 0.01,
            mov_static_y: -0.011,
            mov_static_y_clamp: 0.0,
            mov_static_x: 0.0,
            mov_rotated_y: -0.068,
            mov_rotated_y_clamp: 0.0,
            mov_pusher_x: 0.0,
            clamp: 0.0,
            robot_state: RobotState::Idle,
        }
    }
}


pub struct MachineRegistersPlugin;

impl Plugin for MachineRegistersPlugin {
    fn build(&self, app: &mut App) {
        //app.add_systems(Startup, (init_scene));
        //app.add_systems(OnEnter(AppMode::Loading), (crate::states::scene_control::loading_scene, crate::states::scene_control::init_scene));
        //app.add_systems(OnEnter(AppMode::Restarting), crate::states::scene_control::cleanup_scene);
        //app.add_systems(OnEnter(AppMode::StandBy), crate::states::scene_control::stand_by);
        //app.add_systems(Update, (cad_camera_controller, crate::states::scene_control::draw_gizmos));
        // app.add_systems(Update, tick.run_if(in_state(AppMode::Simulating).or(in_state(RobotState::Feeding))));
        //   app.add_systems(OnEnter(RobotState::ManualSetup), t_manual_set);
        app.add_systems(OnEnter(RobotState::PipeLoading), on_pipe_loading);
        app.add_systems(OnEnter(RobotState::Idle), on_idle);
        app.add_systems(OnEnter(RobotState::PipeLoaded), on_pipe_loaded);
        app.add_systems(OnEnter(RobotState::Feeding), on_pipe_feeding);
        app.add_systems(OnEnter(RobotState::Rotating), on_pipe_rotated);
        app.add_systems(OnEnter(RobotState::Bending), on_pipe_bending);
        app.add_systems(OnEnter(RobotState::Clamping), on_pipe_clamping);
        app.add_systems(OnEnter(RobotState::Unclamping), on_pipe_unclamping);
        //app.add_systems(OnEnter(RobotState::ManualSetup), t_manual_set);
        //app.add_systems(Update, t_manual_set.run_if(in_state(AppMode::Pause)));
        app.add_systems(Update, pause_tick.run_if(in_state(AppMode::Pause)));
        app.add_systems(Update, tick.run_if(in_state(AppMode::Simulating).and(in_state(RobotState::Feeding).or(in_state(RobotState::Bending)).or(in_state(RobotState::Bending)))));
        app.add_systems(Update, rotate_pipe.run_if(in_state(AppMode::Simulating).and(in_state(RobotState::Rotating))));
        app.add_systems(Update, unclamping.run_if(in_state(AppMode::Simulating).and(in_state(RobotState::Unclamping))));
        app.add_systems(Update, clamping.run_if(in_state(AppMode::Simulating).and(in_state(RobotState::Clamping))));
    }
}

fn pause_tick(time: Res<Time>,
              mut machine_registers: ResMut<MachineRegisters>,
              pipe_spec: Res<PipeSpecification>,
              curr_app_state: Res<State<AppMode>>,
              mut next_app_state: ResMut<NextState<AppMode>>,
              curr_robot_state: Res<State<RobotState>>,
              mut next_robot_state: ResMut<NextState<RobotState>>,
) {
    let (tm, dx, last_dx, is_rotated, curr_bend_angle, current_id) = calculate_pipe_matrix(&mut machine_registers, &pipe_spec.segments);
    machine_registers.current_id = current_id;
    machine_registers.dx = dx;
    machine_registers.last_dx = last_dx;
    machine_registers.tm = tm;
}

fn tick(time: Res<Time>,
        mut machine_registers: ResMut<MachineRegisters>,
        pipe_spec: Res<PipeSpecification>,
        curr_app_state: Res<State<AppMode>>,
        mut next_app_state: ResMut<NextState<AppMode>>,
        curr_robot_state: Res<State<RobotState>>,
        mut next_robot_state: ResMut<NextState<RobotState>>,
) {
    let speed = 0.05;
    machine_registers.t += time.delta_secs() * speed;

    let (tm, dx, last_dx, is_rotated, curr_bend_angle, current_id) = calculate_pipe_matrix(&mut machine_registers, &pipe_spec.segments);
    machine_registers.current_id = current_id;
    if (is_rotated) {
        if (curr_robot_state.get() != &RobotState::Rotating) {
            next_robot_state.set(RobotState::Unclamping);
            machine_registers.robot_state = RobotState::Rotating;
        }
    } else {
        if (curr_bend_angle != 0.0) {
            if (curr_robot_state.get() != &RobotState::Bending) {
                next_robot_state.set(RobotState::Clamping);
                machine_registers.robot_state = RobotState::Bending;
            }
            //next_robot_state.set(RobotState::Bending);

        } else {
            // next_robot_state.set(RobotState::Feeding);
            if (curr_robot_state.get() != &RobotState::Feeding) {
                next_robot_state.set(RobotState::Unclamping);
                machine_registers.robot_state = RobotState::Feeding;
            }
        }
        if (curr_bend_angle > 0.0) {
            machine_registers.prev_bend_angle = machine_registers.bend_angle;
            machine_registers.bend_angle = curr_bend_angle;
        }
        machine_registers.dx = dx;
        machine_registers.last_dx = last_dx;
        machine_registers.tm = tm;
    }
}

fn unclamping(time: Res<Time>,
              mut machine_registers: ResMut<MachineRegisters>,
              pipe_spec: Res<PipeSpecification>,
              curr_app_state: Res<State<AppMode>>,
              mut next_app_state: ResMut<NextState<AppMode>>,
              curr_robot_state: Res<State<RobotState>>,
              mut next_robot_state: ResMut<NextState<RobotState>>,
) {
    let mut is_angle_ok = false;
    let mut is_clamp0_ok = false;
    let mut is_clamp1_ok = false;
    let mut is_clamp2_ok = false;
    let ang_speed = time.delta_secs_f64() * 0.5;
    let clamp_speed = time.delta_secs_f64() * 0.1;


    if (machine_registers.prev_bend_angle > 0.0) {
        machine_registers.prev_bend_angle -= ang_speed;
        machine_registers.bend_angle = machine_registers.prev_bend_angle;
    } else {
        machine_registers.prev_bend_angle = 0.0;
        machine_registers.bend_angle = machine_registers.prev_bend_angle;
        is_angle_ok = true;
    }

    if (machine_registers.mov_y_clamp > -machine_registers.clamp * 2.0) {
        machine_registers.mov_y_clamp -= clamp_speed;
    } else {
        machine_registers.mov_y_clamp = -machine_registers.clamp * 2.0;
        is_clamp0_ok = true;
    }


    if (machine_registers.mov_rotated_y_clamp < machine_registers.clamp * 3.0) {
        machine_registers.mov_rotated_y_clamp += clamp_speed;
    } else {
        machine_registers.mov_rotated_y_clamp = machine_registers.clamp * 3.0;
        is_clamp1_ok = true;
    }

    if (machine_registers.mov_static_y_clamp < machine_registers.clamp * 3.0) {
        machine_registers.mov_static_y_clamp += clamp_speed;
    } else {
        machine_registers.mov_static_y_clamp = machine_registers.clamp * 3.0;
        is_clamp2_ok = true;
    }

    if (is_angle_ok && is_clamp0_ok && is_clamp1_ok && is_clamp2_ok) {
        next_robot_state.set(machine_registers.robot_state);
    }
}

fn clamping(time: Res<Time>,
            mut machine_registers: ResMut<MachineRegisters>,
            pipe_spec: Res<PipeSpecification>,
            curr_app_state: Res<State<AppMode>>,
            mut next_app_state: ResMut<NextState<AppMode>>,
            curr_robot_state: Res<State<RobotState>>,
            mut next_robot_state: ResMut<NextState<RobotState>>,
) {
    let clamp_speed = time.delta_secs_f64() * 0.1;

    let mut is_z_ok = false;

    let mut is_clamp0_ok = false;
    let mut is_clamp1_ok = false;
    let mut is_clamp2_ok = false;


    if (machine_registers.mov_to_z == machine_registers.mov_z) {
        is_z_ok = true;
    } else {
        if (machine_registers.mov_z < machine_registers.mov_to_z) {
            machine_registers.mov_z += clamp_speed;
            if (machine_registers.mov_z > machine_registers.mov_to_z) {
                machine_registers.mov_z = machine_registers.mov_to_z;
                is_z_ok = true;
            }
        }

        if (machine_registers.mov_z > machine_registers.mov_to_z) {
            machine_registers.mov_z -= clamp_speed;
            if (machine_registers.mov_z < machine_registers.mov_to_z) {
                machine_registers.mov_z = machine_registers.mov_to_z;
                is_z_ok = true;
            }
        }
    }


    if (is_z_ok) {
        if (machine_registers.mov_y_clamp < 0.0) {
            machine_registers.mov_y_clamp += clamp_speed;
        } else {
            machine_registers.mov_y_clamp = 0.0;
            is_clamp0_ok = true;
        }

        if (machine_registers.mov_rotated_y_clamp > 0.0) {
            machine_registers.mov_rotated_y_clamp -= clamp_speed;
        } else {
            machine_registers.mov_rotated_y_clamp = 0.0;
            is_clamp1_ok = true;
        }

        if (machine_registers.mov_static_y_clamp > 0.0) {
            machine_registers.mov_static_y_clamp -= clamp_speed;
        } else {
            machine_registers.mov_static_y_clamp = 0.0;
            is_clamp2_ok = true;
        }
    }


    if (is_z_ok && is_clamp0_ok && is_clamp1_ok && is_clamp2_ok) {
        next_robot_state.set(machine_registers.robot_state);
    }
}

pub fn on_pipe_loaded(mut machine_registers: ResMut<MachineRegisters>, pipe_spec: Res<PipeSpecification>, mut next_app_state: ResMut<NextState<AppMode>>) {
    machine_registers.robot_state = RobotState::PipeLoaded;
    next_app_state.set(AppMode::Pause);
}
pub fn on_pipe_loading(mut machine_registers: ResMut<MachineRegisters>, pipe_spec: Res<PipeSpecification>, mut next_robot_state: ResMut<NextState<RobotState>>) {
    machine_registers.robot_state = RobotState::PipeLoading;
    machine_registers.mov_pusher_x = 3.521 - pipe_spec.len / 1000.0;
    machine_registers.clamp = pipe_spec.diameter / 1000.0;
    machine_registers.mov_static_y_clamp = machine_registers.clamp * 3.0;
    machine_registers.mov_rotated_y_clamp = machine_registers.clamp * 3.0;
    machine_registers.mov_y_clamp = -machine_registers.clamp * 2.0;
    machine_registers.mov_y = 0.199 - pipe_spec.diameter / 1000.0;
    next_robot_state.set(RobotState::PipeLoaded);
}
fn on_idle(mut machine_registers: ResMut<MachineRegisters>) {
    machine_registers.robot_state = RobotState::Idle;
}
fn on_pipe_feeding(mut machine_registers: ResMut<MachineRegisters>) {
    machine_registers.robot_state = RobotState::Feeding;
}
fn on_pipe_bending(mut machine_registers: ResMut<MachineRegisters>) {
    machine_registers.robot_state = RobotState::Bending;
}

fn on_pipe_rotated(mut machine_registers: ResMut<MachineRegisters>, pipe_spec: Res<PipeSpecification>, mut next_robot_state: ResMut<NextState<RobotState>>) {
    machine_registers.robot_state = RobotState::Rotating;
}

fn on_pipe_clamping(mut machine_registers: ResMut<MachineRegisters>, pipe_spec: Res<PipeSpecification>, mut next_robot_state: ResMut<NextState<RobotState>>) {
    let (dz, state) = calculate_dz(&machine_registers, &pipe_spec);
    machine_registers.bending_state = state as u32;
    machine_registers.mov_to_z += dz;
    // next_robot_state.set(machine_registers.robot_state);
    println!("on_pipe_clamping current_bend_radius {:?}  dz {:?}", machine_registers.current_bend_radius, machine_registers.mov_to_z);
}
fn on_pipe_unclamping(mut machine_registers: ResMut<MachineRegisters>, pipe_spec: Res<PipeSpecification>, curr_robot_state: Res<State<RobotState>>, mut next_robot_state: ResMut<NextState<RobotState>>) {
    println!("on_pipe_unclamping {:?}  {:?}", machine_registers.bend_angle, machine_registers.prev_bend_angle);
}

fn rotate_pipe(mut machine_registers: ResMut<MachineRegisters>, pipe_spec: Res<PipeSpecification>, mut next_robot_state: ResMut<NextState<RobotState>>) {
    let (tm, dx, last_dx, is_rotated, curr_bend_angle, current_id) = calculate_pipe_matrix(&mut machine_registers, &pipe_spec.segments);

    if (is_rotated) {
        machine_registers.tm = tm;
    } else {
        next_robot_state.set(RobotState::Feeding);
        machine_registers.robot_state = RobotState::Feeding;
    }
}


fn calculate_pipe_matrix(machine_staus: &mut ResMut<MachineRegisters>, lraclr_arr: &Vec<LRACLR>) -> (Mat4, f64, f64, bool, f64, u64) {
    let t = machine_staus.t as f64;
    let delta_rot: f64 = 1.0;

    let (pt, xv, yv, zv, rot_deg, id, cp, l, theta, bend_radius) = byt(t, lraclr_arr, &machine_staus.up_dir);
    let current_id = id;
    machine_staus.current_bend_radius = bend_radius;
    let dx = t * l;
    let last_dx = l - t * l;
    let mut is_paused = false;
    let mut direction = 1.0;
    if (!id.is_odd()) {
        machine_staus.rot_angle = rot_deg;
        machine_staus.rot_step = -machine_staus.rot_angle;
    } else {
        if (machine_staus.rot_angle != 0.0) {
            is_paused = true;
            machine_staus.rot_step = machine_staus.rot_step + delta_rot * machine_staus.rot_angle.signum();
            if (machine_staus.rot_angle.signum() == machine_staus.rot_step.signum()) {
                is_paused = false;
                machine_staus.rot_step = 0.0;
            }
        }
    }

    if (t < 0.0 || t > 1.0) {
        direction = direction * -1.0;
        //bend_commands.is_paused=true;
    }

    let mut x_rotation = Mat3::from_cols(
        Vec3::new(1.0, 0.0, 0.0),
        Vec3::new(0.0, machine_staus.rot_step.to_radians().cos() as f32, -machine_staus.rot_step.to_radians().sin() as f32),
        Vec3::new(0.0, machine_staus.rot_step.to_radians().sin() as f32, machine_staus.rot_step.to_radians().cos() as f32),
    );

    let dest_pos = -Vec3::new(pt.x as f32, pt.y as f32, pt.z as f32);
    let source_x = Vec3::new(xv.x as f32, xv.y as f32, xv.z as f32);
    let source_y = Vec3::new(yv.x as f32, yv.y as f32, yv.z as f32);
    let source_z = Vec3::new(zv.x as f32, zv.y as f32, zv.z as f32);

    let dest_x = Vec3::X;  // X-axis now points where Y used to
    let dest_y = Vec3::Y; // Y-axis now points where -X used to
    let dest_z = Vec3::Z;  // Z-axis is unchanged

    let m_source = Mat3::from_cols(source_x, source_y, source_z);
    let m_dest = Mat3::from_cols(dest_x, dest_y, dest_z);

    let z_mirror = Mat3::from_cols(
        Vec3::new(-1.0, 0.0, 0.0),
        Vec3::new(0.0, -1.0, 0.0),
        Vec3::new(0.0, 0.0, 1.0),
    );

    let x_mirror = Mat3::from_cols(
        Vec3::new(1.0, 0.0, 0.0),
        Vec3::new(0.0, -1.0, 0.0),
        Vec3::new(0.0, 0.0, -1.0),
    );
    let rot_matrix: Mat3 = x_rotation * x_mirror * z_mirror * m_dest * m_source.transpose();
    let final_transform_matrix: Mat4 = Mat4::from_mat3(rot_matrix) * Mat4::from_translation(dest_pos);

    (final_transform_matrix, dx, last_dx, is_paused, theta, current_id)
}

fn calculate_dz(mr: &MachineRegisters, pipe_spec: &PipeSpecification) -> (f64, i32) {
    let mut dz: f64 = 0.0;
    let mut state = 0;
    match mr.bending_state {
        0 => {
            let roller = pipe_spec.roller_a;
            if (roller != mr.current_bend_radius) {
                state = 1;
                dz = 0.053;
            };
        }
        1 => {
            let roller = pipe_spec.roller_b;
            if (roller != mr.current_bend_radius) {
                state = 0;
                dz = -0.053;
            };
        }
        _ => {}
    }

    (dz, state)
}