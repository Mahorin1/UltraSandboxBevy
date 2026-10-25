mod camera;
mod physics;
mod ragdoll;
mod sandbox;

use avian2d::prelude::*;
use bevy::prelude::*;

use ragdoll::RagdollCollisionHooks;

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
            PhysicsPlugins::default()
                .with_length_unit(100.0)
                .with_collision_hooks::<RagdollCollisionHooks>(),
            camera::CameraPlugin,
            physics::SandboxPhysicsPlugin,
            sandbox::SandboxPlugin,
            ragdoll::RagdollPlugin,
        ))
        .insert_resource(Time::<Fixed>::from_hz(80.0))
        .insert_resource(SubstepCount(4))
        .run();
}
