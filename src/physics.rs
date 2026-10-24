use avian2d::prelude::*;
use bevy::prelude::*;

const GRAVITY: f32 = 981.0;

pub struct SandboxPhysicsPlugin;

impl Plugin for SandboxPhysicsPlugin {
    fn build(&self, app: &mut App) {
        app.insert_resource(Gravity(Vec2::NEG_Y * GRAVITY));
    }
}
