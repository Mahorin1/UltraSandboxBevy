use bevy::prelude::*;
use bevy::window::PrimaryWindow;

use super::debug::RagdollDebug;

#[derive(Resource, Clone, Copy, Debug)]
pub struct PointerInput {
    pub world: Option<Vec2>,
    pub left: bool,
    pub right: bool,
}

impl Default for PointerInput {
    fn default() -> Self {
        Self {
            world: None,
            left: false,
            right: false,
        }
    }
}

#[derive(Resource, Clone, Copy, Debug, Default)]
pub struct RagdollInput {
    pub move_axis: f32,
    pub crouch: bool,
    pub brace: bool,
    pub active_request: Option<bool>,
    pub jump_requested: bool,
    pub recover_requested: bool,
    pub grip_requests: [bool; 2],
    pub spawn_requested: bool,
    pub foot_bias: f32,
}

#[derive(Component)]
pub struct Grabbable;

pub fn read_pointer(
    windows: Query<&Window, With<PrimaryWindow>>,
    cameras: Query<(&Camera, &GlobalTransform)>,
    buttons: Res<ButtonInput<MouseButton>>,
    mut pointer: ResMut<PointerInput>,
) {
    pointer.left = buttons.pressed(MouseButton::Left);
    pointer.right = buttons.pressed(MouseButton::Right);
    pointer.world = None;

    let (Ok(window), Ok((camera, camera_transform))) = (windows.single(), cameras.single()) else {
        return;
    };
    let Some(cursor) = window.cursor_position() else {
        return;
    };
    pointer.world = camera.viewport_to_world_2d(camera_transform, cursor).ok();
}

pub fn read_keyboard(
    keys: Res<ButtonInput<KeyCode>>,
    mut input: ResMut<RagdollInput>,
    mut debug: ResMut<RagdollDebug>,
) {
    let right = keys.pressed(KeyCode::KeyD) || keys.pressed(KeyCode::ArrowRight);
    let left = keys.pressed(KeyCode::KeyA) || keys.pressed(KeyCode::ArrowLeft);
    input.move_axis = (right as i8 - left as i8) as f32;
    input.crouch = keys.pressed(KeyCode::KeyC) || keys.pressed(KeyCode::ControlLeft);
    input.brace = keys.pressed(KeyCode::ShiftLeft) || keys.pressed(KeyCode::KeyT);

    if keys.just_pressed(KeyCode::ArrowUp) || keys.just_pressed(KeyCode::KeyW) {
        input.active_request = Some(true);
    }
    if keys.just_pressed(KeyCode::ArrowDown) {
        input.active_request = Some(false);
    }
    input.jump_requested |= keys.just_pressed(KeyCode::Space);
    input.recover_requested |= keys.just_pressed(KeyCode::KeyR);
    input.grip_requests[0] |= keys.just_pressed(KeyCode::KeyE);
    input.grip_requests[1] |= keys.just_pressed(KeyCode::KeyQ);
    input.spawn_requested |= keys.just_pressed(KeyCode::KeyH);

    if keys.just_pressed(KeyCode::F1) {
        debug.enabled = !debug.enabled;
    }
    if keys.just_pressed(KeyCode::KeyI) {
        input.foot_bias = (input.foot_bias + 2.0).min(20.0);
    }
    if keys.just_pressed(KeyCode::KeyK) {
        input.foot_bias = (input.foot_bias - 2.0).max(-20.0);
    }
}

pub fn clear_actions(mut input: ResMut<RagdollInput>) {
    input.active_request = None;
    input.jump_requested = false;
    input.recover_requested = false;
    input.grip_requests = [false; 2];
}
