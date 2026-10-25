use avian2d::prelude::*;
use bevy::{math::Isometry2d, prelude::*};

use super::{LocomotionMode, Ragdoll, RagdollControl, Side};

#[derive(Resource, Default)]
pub struct RagdollDebug {
    pub enabled: bool,
}

#[derive(Component)]
struct RagdollHud;

pub fn setup_hud(mut commands: Commands) {
    commands.spawn((
        Text::new(
            "A/D ou ←/→  marcher    C/Ctrl  s'accroupir    Espace  sauter\n\
             Clic gauche  bras arrière (IK)    Clic droit  bras avant (IK)\n\
             E/Q  saisir/relâcher avec la main correspondante    R  se relever\n\
             Haut/W  activer    Bas  ragdoll passif    Shift/T  se ramasser\n\
             I/K  biais des pieds    H  nouveau ragdoll    F1  diagnostic",
        ),
        TextFont {
            font_size: FontSize::Px(14.0),
            ..default()
        },
        TextColor(Color::srgb(0.92, 0.95, 1.0)),
        Node {
            position_type: PositionType::Absolute,
            top: px(12),
            left: px(12),
            ..default()
        },
    ));
    commands.spawn((
        RagdollHud,
        Text::new("Ragdoll : initialisation"),
        TextFont {
            font_size: FontSize::Px(14.0),
            ..default()
        },
        TextColor(Color::srgb(0.65, 0.95, 0.78)),
        Node {
            position_type: PositionType::Absolute,
            top: px(108),
            left: px(12),
            ..default()
        },
    ));
}

pub fn update_hud(
    controls: Query<&RagdollControl>,
    mut hud: Query<&mut Text, With<RagdollHud>>,
) {
    let Some(control) = controls.iter().next() else {
        return;
    };
    let Ok(mut text) = hud.single_mut() else {
        return;
    };
    let mode = match control.mode {
        LocomotionMode::Passive => "passif",
        LocomotionMode::Standing => "équilibre",
        LocomotionMode::Walking => "marche",
        LocomotionMode::Crouching => "accroupi",
        LocomotionMode::Jumping => "saut",
        LocomotionMode::Hanging => "suspension",
        LocomotionMode::Recovering => "relevé",
    };
    let recovery = control
        .recovery
        .map(|state| format!(" · {:?}", state.phase))
        .unwrap_or_default();
    text.0 = format!(
        "{mode}{recovery} · appuis [{}, {}] · prises [{}, {}] · vCoM {:5.1} px/s",
        if control.grounded[0] { "●" } else { "○" },
        if control.grounded[1] { "●" } else { "○" },
        control.grips[0].is_some(),
        control.grips[1].is_some(),
        control.com_velocity.x,
    );
}

pub fn draw_debug(
    debug: Res<RagdollDebug>,
    ragdolls: Query<(&Ragdoll, &RagdollControl)>,
    positions: Query<&Position>,
    mut gizmos: Gizmos,
) {
    if !debug.enabled {
        return;
    }
    for (ragdoll, control) in &ragdolls {
        gizmos.circle_2d(Isometry2d::from_translation(control.com), 4.0, Color::srgb(1.0, 0.85, 0.1));
        gizmos.line_2d(control.com, control.capture_point, Color::srgb(1.0, 0.35, 0.2));
        gizmos.cross_2d(control.capture_point, 7.0, Color::srgb(1.0, 0.35, 0.2));
        gizmos.cross_2d(control.support_center, 6.0, Color::srgb(0.2, 0.9, 1.0));

        for side in Side::ALL {
            let index = side.index();
            let foot_entity = ragdoll.bodies[if side == Side::Back { 8 } else { 11 }];
            if let Ok(position) = positions.get(foot_entity) {
                gizmos.line_2d(
                    position.0,
                    control.gait.feet[index],
                    Color::srgb(0.25, 0.95, 0.55),
                );
            }
            gizmos.circle_2d(
                Isometry2d::from_translation(control.gait.feet[index]),
                4.0,
                Color::srgb(0.25, 0.95, 0.55),
            );
            gizmos.cross_2d(control.arm_targets[index], 5.0, Color::srgb(0.95, 0.35, 0.85));
        }
    }
}
