mod camera;
mod physics;
mod sandbox;

use avian2d::prelude::*;
use bevy::prelude::*;

fn main() {
    App::new()
        .add_plugins((
            DefaultPlugins.set(WindowPlugin {
                primary_window: Some(Window {
                    title: "Ultra Sandbox Bevy".into(),
                    resolution: (1280, 720).into(),
                    ..default()
                }),
                ..default()
            }),
            PhysicsPlugins::default(),
            camera::CameraPlugin,
            physics::SandboxPhysicsPlugin,
            sandbox::SandboxPlugin,
        ))
        .run();
}
