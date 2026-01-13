use bevy::prelude::SystemSet;

pub mod leftpanel;
pub mod toppanel;
pub mod bottompanel;
pub mod camera;
pub mod axes;

#[derive(SystemSet, Debug, Clone, PartialEq, Eq, Hash)]
pub enum UiOrder {
    Top,    // Рисуем первым
    Left,   // Рисуем вторым (чтобы занять высоту)
    Bottom, // Рисуем последним (в остатке)
}