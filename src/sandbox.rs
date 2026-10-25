use avian2d::prelude::*;
use bevy::prelude::*;

use crate::ragdoll::Grabbable;

const GROUND_SIZE: Vec2 = Vec2::new(1600.0, 50.0);
const GROUND_POSITION: Vec2 = Vec2::new(0.0, -250.0);

pub struct SandboxPlugin;

impl Plugin for SandboxPlugin {
    fn build(&self, app: &mut App) {
        app.insert_resource(ClearColor(Color::srgb(0.12, 0.27, 0.44)))
            .add_systems(Startup, spawn_scene);
    }
}

fn spawn_scene(mut commands: Commands) {
    spawn_box(
        &mut commands,
        GROUND_POSITION,
        GROUND_SIZE,
        Color::srgb(0.25, 0.28, 0.32),
        RigidBody::Static,
        0.0,
    );

    let boxes = [
        (
            Vec2::new(-180.0, 180.0),
            Vec2::new(60.0, 60.0),
            Color::srgb(0.95, 0.45, 0.20),
            -0.15,
        ),
        (
            Vec2::new(-60.0, 260.0),
            Vec2::new(50.0, 80.0),
            Color::srgb(0.35, 0.80, 0.55),
            0.20,
        ),
        (
            Vec2::new(70.0, 200.0),
            Vec2::splat(55.0),
            Color::srgb(0.35, 0.65, 0.95),
            -0.25,
        ),
        (
            Vec2::new(190.0, 300.0),
            Vec2::new(75.0, 45.0),
            Color::srgb(0.90, 0.35, 0.45),
            0.10,
        ),
    ];

    for (position, size, color, angle) in boxes {
        spawn_box(
            &mut commands,
            position,
            size,
            color,
            RigidBody::Dynamic,
            angle,
        );
    }

    // Appuis fixes pour tester les prises, la suspension et les transferts de mains.
    for (position, angle) in [
        (Vec2::new(410.0, -145.0), 0.0),
        (Vec2::new(465.0, -65.0), 0.15),
        (Vec2::new(415.0, 20.0), -0.12),
    ] {
        spawn_box(
            &mut commands,
            position,
            Vec2::new(52.0, 14.0),
            Color::srgb(0.72, 0.58, 0.30),
            RigidBody::Static,
            angle,
        );
    }
}

fn spawn_box(
    commands: &mut Commands,
    position: Vec2,
    size: Vec2,
    color: Color,
    body: RigidBody,
    angle: f32,
) {
    commands.spawn((
        Sprite::from_color(color, size),
        Transform::from_translation(position.extend(0.0))
            .with_rotation(Quat::from_rotation_z(angle)),
        body,
        Collider::rectangle(size.x, size.y),
        ColliderDensity(1.0),
        Friction::new(0.8),
        Restitution::ZERO,
        Grabbable,
    ));
}
