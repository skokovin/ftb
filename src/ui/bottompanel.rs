use bevy::app::{App, Plugin};
use bevy::prelude::*; // Res, ResMut, State и т.д.
use bevy_egui::{egui, EguiContexts, EguiPrimaryContextPass};
use crate::states::scene_control::AppMode;
use crate::ui::UiOrder;
use crate::states::state_machine::{MachineRegisters, RobotState};
// Твой Enum режимов

pub struct BottomUiPanelPlugin;

impl Plugin for BottomUiPanelPlugin {
    fn build(&self, app: &mut App) {
        app.add_systems(EguiPrimaryContextPass, ui_system.in_set(UiOrder::Bottom));
    }
}

fn ui_system(
    mut contexts: EguiContexts,
    mut status: ResMut<MachineRegisters>, // Читаем и пишем статус станка
    mut next_state: ResMut<NextState<AppMode>>, // Переключаем режимы
    current_state: Res<State<AppMode>>, // Узнаем текущий режим
    curr_robot_state: Res<State<RobotState>>,
    mut next_robot_state: ResMut<NextState<RobotState>>,
) {
    let Ok(ctx) = contexts.ctx_mut() else { return; };

    // Инициализация иконок (лучше вынести в main, но можно и тут)
    egui_material_icons::initialize(ctx);

    egui::TopBottomPanel::bottom("bottom_panel").show(ctx, |ui| {
        ui.horizontal(|ui| {

            // 1. КНОПКА PLAY / PAUSE
            let is_simulating = *current_state.get() == AppMode::Simulating;

            let icon = if is_simulating {
                egui_material_icons::icons::ICON_PAUSE
            } else {
                egui_material_icons::icons::ICON_PLAY_ARROW
            };

            // Кнопки управления
            if ui.button(egui_material_icons::icons::ICON_FAST_REWIND).clicked() {
                status.t = 0.0; // Сброс в начало
            }
            if ui.button(icon).clicked() {
                if is_simulating {
                    next_state.set(AppMode::Pause); // Пауза
                } else {
                    next_state.set(AppMode::Simulating); // Старт
                }
            }
            if ui.button(egui_material_icons::icons::ICON_FAST_FORWARD).clicked() {
                status.t = 0.0; // Сброс в начало
            }
            // 2. СЛАЙДЕР ПРОГРЕССА (0.0 - 1.0)
            // Мы растягиваем слайдер на всю ширину
            ui.style_mut().spacing.slider_width = ui.available_width() - 50.0;


            // Связываем слайдер напрямую с animation_time
            let slider = ui.add(egui::Slider::new(&mut status.t, 0.0..=1.0));
                //.show_value(false) // Скрываем цифры, если мешают
                //.text(format!("{:.0}%", status.animation_time * 100.0)));

            // Если юзер тянет слайдер руками — ставим на паузу, чтобы не бороться с системой анимации
            if slider.dragged() {
              //  next_robot_state.set(RobotState::ManualSetup);
                next_state.set(AppMode::Pause);
            }
        });
    });
}


/*use bevy::app::{App, Plugin};
use bevy::prelude::{NextState, State};
use bevy_ecs::prelude::*; // ResMut, Query и т.д.
use bevy_egui::{egui, EguiContexts, EguiPrimaryContextPass};
use crate::ui::UiOrder;
use crate::motion::bendmachine::MachineStatus; // Импортируем ресурс статуса
use crate::AppMode; // Импортируем состояния приложения

pub struct BottomUiPanelPlugin;

impl Plugin for BottomUiPanelPlugin {
    fn build(&self, app: &mut App) {
        app.add_systems(EguiPrimaryContextPass, ui_system.in_set(UiOrder::Bottom));
    }
}

fn ui_system(
    mut contexts: EguiContexts,
    mut status: ResMut<MachineStatus>, // Получаем доступ к состоянию станка
    mut next_state: ResMut<NextState<AppMode>>, // Для переключения режимов (Play/Edit)
    current_state: Res<State<AppMode>>,
) {
    let Ok(ctx) = contexts.ctx_mut() else { return; };

    // Иконки (если подключены)
     egui_material_icons::initialize(ctx);

    egui::TopBottomPanel::bottom("bottom_panel").show(ctx, |ui| {
        ui.horizontal(|ui| {
            // Кнопка PLAY / PAUSE
            let is_playing = *current_state.get() == AppMode::Simulating;
            let icon_text = if is_playing { "PAUSE" } else { "PLAY" }; // Или иконка

            if ui.button(icon_text).clicked() {
                if is_playing {
                    next_state.set(AppMode::Editing);
                } else {
                    next_state.set(AppMode::Simulating);
                }
            }

            // СЛАЙДЕР ВРЕМЕНИ
            ui.style_mut().spacing.slider_width = ui.available_width() - 20.0;

            // Привязываем слайдер к status.animation_time
            // Диапазон 0.0 ..= 1.0 (прогресс гибки)
            ui.add(egui::Slider::new(&mut status.animation_time, 0.0..=1.0).text("Progress"));
        });
    });
}
*/

/*use bevy::app::{App, Plugin};
use bevy::prelude::default;
use bevy_ecs::change_detection::ResMut;
use bevy_ecs::prelude::IntoScheduleConfigs;
use bevy_egui::{egui, EguiContexts, EguiPrimaryContextPass};
use crate::algo::cnc::LRACLR;
use crate::PipeSpecification;
use crate::ui::UiOrder;

pub struct BottomUiPanelPlugin;

impl Plugin for BottomUiPanelPlugin {
    fn build(&self, app: &mut App) {
        app
            // Добавляем системы в расписание Update
            .add_systems(EguiPrimaryContextPass, ui_system.in_set(UiOrder::Bottom));

        // Если вам нужно инициализировать ресурсы при старте:
        // .init_resource::<MyUiState>()
    }
}




fn ui_system(
    mut contexts: EguiContexts,
    mut pipe_spec: ResMut<PipeSpecification>,
) {
    let Ok(ctx) = contexts.ctx_mut() else {
        return;
    };
    egui_material_icons::initialize(ctx);
    egui::TopBottomPanel::bottom("bottom_panel").show(ctx, |ui| {
        let icon = if true {
            egui_material_icons::icons::ICON_PLAY_ARROW
        } else {
            egui_material_icons::icons::ICON_PAUSE
        };



        ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
            if ui.button(egui_material_icons::icons::ICON_FAST_FORWARD).clicked() {

            }
            if ui.button(icon).clicked() {

            }
            if ui.button(egui_material_icons::icons::ICON_FAST_REWIND).clicked() {

            }

            ui.style_mut().spacing.slider_width = ui.available_width() - 70.0;

            ui.add(
                egui::Slider::new(&mut 0.5, 0.0..=1.0).custom_formatter(|value, _range| {
                    format!("{:.0}", value )
                }),
            );
        });
    }).response.rect.width();

}*/