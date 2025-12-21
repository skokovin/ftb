use bevy::prelude::*;
use bevy::input::mouse::{MouseMotion, MouseWheel};
use bevy::window::PrimaryWindow;

// Компонент-маркер для нашей камеры
#[derive(Component)]
pub struct CadCamera {
    pub focus_point: Vec3, // Точка, вокруг которой вращаемся
    pub dist: f32,         // Расстояние от камеры до точки
    pub zoom_speed: f32,
    pub pan_speed: f32,
    pub orbit_speed: f32,
}

impl Default for CadCamera {
    fn default() -> Self {
        Self {
            focus_point: Vec3::new(0.0, 0.0, 0.0), // Центр станка
            dist: 3000.0,    // Начальное отдаление
            zoom_speed: 0.2,
            pan_speed: 1.5,
            orbit_speed: 0.005,
        }
    }
}

pub fn cad_camera_controller(
    // В Bevy 0.17 используем MessageReader вместо EventReader
    mut mouse_wheel: MessageReader<MouseWheel>,
    mut mouse_motion: MessageReader<MouseMotion>,

    mouse_btn: Res<ButtonInput<MouseButton>>,
    keys: Res<ButtonInput<KeyCode>>,

    // Запросы к ECS
    q_window: Query<&Window, With<PrimaryWindow>>,
    mut q_camera: Query<(&mut Transform, &mut CadCamera, &Camera, &GlobalTransform)>,
) {
    // 1. Безопасное получение окна (работает в любой версии)
    // Если get_single() ругается, iter().next() — это 100% рабочий аналог.
    let Some(window) = q_window.iter().next() else { return };

    // 2. Получение камеры
    // Используем тот же трюк с итератором, чтобы избежать ошибок API
    let Some((mut transform, mut settings, camera, global_transform)) = q_camera.iter_mut().next() else { return };

    // --- ЗУМ (Колесико) ---
    for event in mouse_wheel.read() {
        let zoom_factor = 1.0 - event.y * settings.zoom_speed;
        let old_dist = settings.dist;
        let new_dist = (old_dist * zoom_factor).max(10.0).min(50000.0);

        // Магия CAD-зума: приближаем к курсору
        if let Some(cursor_pos) = window.cursor_position() {
            if let Ok(ray) = camera.viewport_to_world(global_transform, cursor_pos) {
                // Точка в мире, куда указывает мышь (на текущем расстоянии фокуса)
                let pivot_under_mouse = ray.origin + ray.direction * old_dist;

                // Вектор смещения
                let offset = pivot_under_mouse - settings.focus_point;

                // Сдвигаем точку фокуса
                settings.focus_point += offset * (1.0 - zoom_factor);
            }
        }

        settings.dist = new_dist;
    }

    // Собираем движение мыши
    let mut delta = Vec2::ZERO;
    for event in mouse_motion.read() {
        delta += event.delta;
    }

    let shift_pressed = keys.pressed(KeyCode::ShiftLeft);
    let middle_click = mouse_btn.pressed(MouseButton::Middle);

    // --- ПАНОРАМИРОВАНИЕ (Shift + Middle) ---
    if middle_click && shift_pressed && delta != Vec2::ZERO {
        let right = transform.right();
        let up = transform.up();
        let scale_factor = settings.dist * 0.0008; // Чем дальше, тем быстрее

        let pan_vec = (right * -delta.x + up * delta.y) * settings.pan_speed * scale_factor;
        settings.focus_point += pan_vec;
    }

    // --- ВРАЩЕНИЕ (Middle) ---
    if middle_click && !shift_pressed && delta != Vec2::ZERO {
        // Вращаем вокруг Y (глобально)
        let yaw = Quat::from_rotation_y(-delta.x * settings.orbit_speed);
        // Вращаем вокруг X (локально)
        let pitch = Quat::from_rotation_x(-delta.y * settings.orbit_speed);

        transform.rotation = yaw * transform.rotation * pitch;

        // ВАЖНО: Убираем крен (Z-roll), чтобы горизонт не заваливался
        let (y, x, _z) = transform.rotation.to_euler(EulerRot::YXZ);
        transform.rotation = Quat::from_euler(EulerRot::YXZ, y, x, 0.0);
    }

    // --- ФИНАЛЬНАЯ ПОЗИЦИЯ ---
    // Ставим камеру на расстоянии dist от focus_point
    let look_dir = transform.forward();
    transform.translation = settings.focus_point - look_dir * settings.dist;
}