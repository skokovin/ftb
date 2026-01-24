use bevy::prelude::*;
use bevy::input::mouse::{MouseMotion, MouseWheel};
use bevy::window::PrimaryWindow;

#[derive(Component)]
pub struct CadCamera {
    pub focus_point: Vec3,
    pub dist: f32,
    pub zoom_speed: f32,
    pub pan_speed: f32,
    pub orbit_speed: f32,
}

impl Default for CadCamera {
    fn default() -> Self {
        Self {
            focus_point: Vec3::new(0.0, 0.0, 0.0),
            dist: 3000.0,
            zoom_speed: 0.2,
            pan_speed: 1.5,
            orbit_speed: 0.005,
        }
    }
}

pub fn cad_camera_controller(
    mut mouse_wheel: MessageReader<MouseWheel>,
    mut mouse_motion: MessageReader<MouseMotion>,
    mouse_btn: Res<ButtonInput<MouseButton>>,
    keys: Res<ButtonInput<KeyCode>>,
    q_window: Query<&Window, With<PrimaryWindow>>,
    mut q_camera: Query<(&mut Transform, &mut CadCamera, &Camera, &GlobalTransform)>,
) {
    let Some(window) = q_window.iter().next() else { return };
    let Some((mut transform, mut settings, camera, global_transform)) = q_camera.iter_mut().next() else { return };
    
    for event in mouse_wheel.read() {
        let zoom_factor = 1.0 - event.y * settings.zoom_speed;
        let old_dist = settings.dist;
        let new_dist = (old_dist * zoom_factor).max(10.0).min(50000.0);

        if let Some(cursor_pos) = window.cursor_position() {
            if let Ok(ray) = camera.viewport_to_world(global_transform, cursor_pos) {
                let pivot_under_mouse = ray.origin + ray.direction * old_dist;
                let offset = pivot_under_mouse - settings.focus_point;
                settings.focus_point += offset * (1.0 - zoom_factor);
            }
        }
        settings.dist = new_dist;
    }
    
    let mut delta = Vec2::ZERO;
    for event in mouse_motion.read() {
        delta += event.delta;
    }

    let right_click = mouse_btn.pressed(MouseButton::Right);
    let middle_click = mouse_btn.pressed(MouseButton::Middle);
    let _shift_pressed = keys.pressed(KeyCode::ShiftLeft); // Shift больше не обязателен для панорамирования

    if middle_click && delta != Vec2::ZERO {
        let right = transform.right();
        let up = transform.up();
        let scale_factor = settings.dist * 0.0008;

        let pan_vec = (right * -delta.x + up * delta.y) * settings.pan_speed * scale_factor;
        settings.focus_point += pan_vec;
    }
    
    if right_click && delta != Vec2::ZERO {
        let sensitive_x = delta.x * settings.orbit_speed;
        let sensitive_y = delta.y * settings.orbit_speed;
        
        let q_yaw = Quat::from_rotation_z(-sensitive_x);
        transform.rotation = q_yaw * transform.rotation;

        let q_pitch = Quat::from_rotation_x(-sensitive_y);
        let new_rotation = transform.rotation * q_pitch;
        
       let new_forward = new_rotation * Vec3::NEG_Z;
       let angle_to_up = new_forward.angle_between(Vec3::Z);

        const PITCH_LIMIT: f32 = 0.1; 
        if angle_to_up > PITCH_LIMIT && angle_to_up < (std::f32::consts::PI - PITCH_LIMIT) {
            transform.rotation = new_rotation;
        }
    }

    
    let look_dir = transform.forward();
    transform.translation = settings.focus_point - look_dir * settings.dist;
}

