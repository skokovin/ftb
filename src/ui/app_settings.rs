use std::sync::atomic::AtomicUsize;

pub static STRIGHT_SPEED: AtomicUsize = AtomicUsize::new(100);
pub static ROTATE_SPEED: AtomicUsize = AtomicUsize::new(10);
pub static ANGLE_SPEED: AtomicUsize = AtomicUsize::new(10);