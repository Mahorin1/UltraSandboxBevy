use avian2d::prelude::*;
use bevy::prelude::*;

use super::debug::RagdollDebug;
use super::input::{Grabbable, PointerInput, RagdollInput};
use super::math::*;
use super::{
    joint_limits, motor_settings, FootSensor, GaitState, Grip, LocomotionMode, Ragdoll,
    RagdollControl, RagdollJoint, RagdollMember, RecoveryPhase, RecoveryState, Side,
};

const GRAVITY: f32 = 981.0;
const GRAB_RADIUS: f32 = 52.0;
const ARM_REACH: f32 = UPPER_ARM_LENGTH + LOWER_ARM_LENGTH - 1.0;
const STEP_HEIGHT: f32 = 24.0;

#[derive(Clone, Copy, Default)]
struct BodyView {
    position: Vec2,
    rotation: f32,
    linear_velocity: Vec2,
    angular_velocity: f32,
    mass: f32,
}

#[allow(clippy::too_many_arguments)]
pub fn control_ragdolls(
    mut commands: Commands,
    time: Res<Time<Fixed>>,
    input: Res<RagdollInput>,
    pointer: Res<PointerInput>,
    debug: Res<RagdollDebug>,
    spatial: SpatialQuery,
    mut ragdolls: Query<(&Ragdoll, &mut RagdollControl)>,
    bodies: Query<(
        &Position,
        &Rotation,
        &LinearVelocity,
        &AngularVelocity,
        &ComputedMass,
    )>,
    mut joints: Query<(&mut RevoluteJoint, &RagdollJoint)>,
    members: Query<&RagdollMember>,
    grabbables: Query<(), With<Grabbable>>,
    rigid_bodies: Query<&RigidBody>,
    poses: Query<(&Position, &Rotation)>,
) {
    let dt = time.delta_secs().min(0.05);
    for (ragdoll, mut control) in &mut ragdolls {
        let Ok(views) = collect_views(ragdoll, &bodies) else {
            continue;
        };
        control.debug_enabled = debug.enabled;

        update_mass_center(&mut control, &views);
        update_grips(
            &mut commands,
            ragdoll,
            &mut control,
            &input,
            &views,
            &spatial,
            &grabbables,
            &rigid_bodies,
            &poses,
        );

        if let Some(active) = input.active_request {
            control.active = active;
            control.rest_time = 0.0;
            if active {
                control.recovery = Some(RecoveryState {
                    phase: RecoveryPhase::Settle,
                    timer: 0.0,
                    attempts: 0,
                });
            }
        }
        if input.recover_requested {
            control.active = true;
            control.recovery = Some(RecoveryState {
                phase: RecoveryPhase::Settle,
                timer: 0.0,
                attempts: 0,
            });
        }
        if input.jump_requested
            && control.active
            && control.recovery.is_none()
            && control.grounded.iter().any(|grounded| *grounded)
        {
            control.jump_timer = Some(0.0);
        }

        if input.move_axis.abs() > 0.01 {
            control.direction = input.move_axis.signum();
        }
        control.air_time = if control.grounded.iter().any(|grounded| *grounded) {
            0.0
        } else {
            control.air_time + dt
        };

        let chest = views[0];
        let ground_y = estimate_ground_y(&control, &views);
        control.gait.ground_y = ground_y;
        control.capture_point = capture_point(control.com, control.com_velocity, ground_y, GRAVITY);
        control.support_center = support_center(&control, &views);

        if control.active && control.recovery.is_none() && !has_fixed_grip(&control) {
            let tilted = wrap_angle(chest.rotation).abs() > 1.15;
            let low = control.com.y - ground_y < 105.0;
            let touching = control.grounded.iter().any(|grounded| *grounded);
            control.fallen_time = if tilted && low && touching {
                control.fallen_time + dt
            } else {
                (control.fallen_time - 2.0 * dt).max(0.0)
            };
            if control.fallen_time > 0.45 {
                control.recovery = Some(RecoveryState {
                    phase: RecoveryPhase::Settle,
                    timer: 0.0,
                    attempts: 0,
                });
                control.fallen_time = 0.0;
            }
        }

        if !control.active {
            control.mode = LocomotionMode::Passive;
            control.rest_time = if control.com_velocity.length() < 24.0
                && control.grounded.iter().any(|grounded| *grounded)
            {
                control.rest_time + dt
            } else {
                0.0
            };
            if control.rest_time >= 3.0 {
                control.active = true;
                control.rest_time = 0.0;
                control.recovery = Some(RecoveryState {
                    phase: RecoveryPhase::Settle,
                    timer: 0.0,
                    attempts: 0,
                });
            }
        }

        let mut targets = current_joint_targets(ragdoll, &views, &joints);
        let mut torque_scale = 1.0;
        let mut frequency_scale = 1.0;

        if control.active {
            if let Some(mut recovery) = control.recovery {
                control.mode = LocomotionMode::Recovering;
                recovery.timer += dt;
                apply_recovery(
                    &mut recovery,
                    &mut control,
                    &views,
                    &mut targets,
                    &input,
                    &pointer,
                    &spatial,
                    &members,
                    dt,
                );
                torque_scale = 0.8;
                if recovery.phase == RecoveryPhase::Settle {
                    frequency_scale = 0.25;
                }
                if control.recovery.is_some() {
                    control.recovery = Some(recovery);
                }
            } else if has_fixed_grip(&control) {
                control.mode = LocomotionMode::Hanging;
                apply_hanging(&mut control, &views, &mut targets);
                apply_arm_tasks(&mut control, &views, &mut targets, &input, &pointer, dt);
            } else if control.jump_timer.is_some() {
                control.mode = LocomotionMode::Jumping;
                apply_jump(&mut control, &views, &mut targets, dt);
                apply_arm_tasks(&mut control, &views, &mut targets, &input, &pointer, dt);
            } else if input.crouch {
                control.mode = LocomotionMode::Crouching;
                apply_crouch(&control, &views, &mut targets, 1.0);
                apply_arm_tasks(&mut control, &views, &mut targets, &input, &pointer, dt);
            } else {
                apply_locomotion(
                    &mut control,
                    &views,
                    &mut targets,
                    &input,
                    &spatial,
                    &members,
                    dt,
                );
                apply_arm_tasks(&mut control, &views, &mut targets, &input, &pointer, dt);
                if control.air_time > 0.12 && input.brace {
                    apply_air_brace(&control, &views, &mut targets);
                }
            }
        }

        apply_joint_drives(
            ragdoll,
            &control,
            &views,
            &mut joints,
            &targets,
            torque_scale,
            frequency_scale,
        );
    }
}

fn collect_views(
    ragdoll: &Ragdoll,
    bodies: &Query<(&Position, &Rotation, &LinearVelocity, &AngularVelocity, &ComputedMass)>,
) -> Result<[BodyView; 12], ()> {
    let mut views = [BodyView::default(); 12];
    for (index, entity) in ragdoll.bodies.iter().enumerate() {
        let (position, rotation, linear, angular, mass) = bodies.get(*entity).map_err(|_| ())?;
        views[index] = BodyView {
            position: position.0,
            rotation: rotation.as_radians(),
            linear_velocity: linear.0,
            angular_velocity: angular.0,
            mass: mass.value(),
        };
    }
    Ok(views)
}

fn update_mass_center(control: &mut RagdollControl, views: &[BodyView; 12]) {
    let mut mass = 0.0;
    let mut position = Vec2::ZERO;
    let mut velocity = Vec2::ZERO;
    for view in views {
        mass += view.mass;
        position += view.position * view.mass;
        velocity += view.linear_velocity * view.mass;
    }
    if mass > 1e-6 {
        control.com = position / mass;
        control.com_velocity = velocity / mass;
    }
}

pub fn update_foot_contacts(
    sensors: Query<(&FootSensor, &CollidingEntities)>,
    members: Query<&RagdollMember>,
    mut controls: Query<&mut RagdollControl>,
) {
    for (sensor, contacts) in &sensors {
        let Ok(mut control) = controls.get_mut(sensor.root) else {
            continue;
        };
        control.grounded[sensor.side.index()] = contacts.iter().any(|entity| {
            members
                .get(*entity)
                .is_none_or(|member| member.root != sensor.root)
        });
    }
}

#[allow(clippy::too_many_arguments)]
fn update_grips(
    commands: &mut Commands,
    ragdoll: &Ragdoll,
    control: &mut RagdollControl,
    input: &RagdollInput,
    views: &[BodyView; 12],
    spatial: &SpatialQuery,
    grabbables: &Query<(), With<Grabbable>>,
    rigid_bodies: &Query<&RigidBody>,
    poses: &Query<(&Position, &Rotation)>,
) {
    for side in Side::ALL {
        let slot = side.index();
        if let Some(mut grip) = control.grips[slot] {
            if let Ok((position, rotation)) = poses.get(grip.body) {
                grip.world_anchor = position.0 + *rotation * grip.object_local;
                control.grips[slot] = Some(grip);
            } else {
                commands.entity(grip.joint).despawn();
                control.grips[slot] = None;
            }
        }
        if !input.grip_requests[slot] {
            continue;
        }
        if let Some(grip) = control.grips[slot].take() {
            commands.entity(grip.joint).despawn();
            continue;
        }
        if !control.active {
            continue;
        }

        let lower_index = if side == Side::Back { 3 } else { 5 };
        let forearm = ragdoll.bodies[lower_index];
        let hand = hand_position(views, lower_index);
        let filter = SpatialQueryFilter::default();
        let projection = spatial.project_point_predicate(
            hand,
            true,
            &filter,
            &|entity| grabbables.contains(entity),
        );
        let Some(projection) = projection else {
            continue;
        };
        if projection.point.distance(hand) > GRAB_RADIUS {
            continue;
        }
        let body = projection.entity;
        let Ok(object_body) = rigid_bodies.get(body) else {
            continue;
        };
        let point = projection.point;
        let joint = commands
            .spawn((
                Name::new("Prise physique"),
                RevoluteJoint::new(forearm, body).with_anchor(point),
                JointCollisionDisabled,
            ))
            .id();
        let Ok((object_position, object_rotation)) = poses.get(body) else {
            commands.entity(joint).despawn();
            continue;
        };
        let object_local = Rotation::radians(-object_rotation.as_radians())
            * (point - object_position.0);
        control.grips[slot] = Some(Grip {
            joint,
            body,
            object_local,
            world_anchor: point,
            fixed_support: matches!(object_body, RigidBody::Static),
        });
    }
}

fn has_fixed_grip(control: &RagdollControl) -> bool {
    control
        .grips
        .iter()
        .flatten()
        .any(|grip| grip.fixed_support)
}

fn current_joint_targets(
    ragdoll: &Ragdoll,
    views: &[BodyView; 12],
    joints: &Query<(&mut RevoluteJoint, &RagdollJoint)>,
) -> [f32; 11] {
    let mut targets = [0.0; 11];
    for (index, entity) in ragdoll.joints.iter().enumerate() {
        if let Ok((_, definition)) = joints.get(*entity) {
            targets[index] = relative_angle(definition, views);
        }
    }
    targets
}

fn relative_angle(definition: &RagdollJoint, views: &[BodyView; 12]) -> f32 {
    let (first, second) = definition.body_indices;
    wrap_angle(views[second].rotation - views[first].rotation - definition.reference)
}

fn apply_joint_drives(
    ragdoll: &Ragdoll,
    control: &RagdollControl,
    views: &[BodyView; 12],
    joints: &mut Query<(&mut RevoluteJoint, &RagdollJoint)>,
    targets: &[f32; 11],
    torque_scale: f32,
    frequency_scale: f32,
) {
    for (index, entity) in ragdoll.joints.iter().enumerate() {
        let Ok((mut joint, definition)) = joints.get_mut(*entity) else {
            continue;
        };
        let (base_frequency, base_torque) = motor_settings(index);
        joint.motor.enabled = control.active;
        joint.motor.target_position = targets[index];
        joint.motor.target_velocity = 0.0;
        joint.motor.max_torque = base_torque * torque_scale;
        joint.motor.motor_model = MotorModel::SpringDamper {
            frequency: base_frequency * frequency_scale,
            damping_ratio: 1.0,
        };

        let shoulder = matches!(index, 1 | 3);
        joint.angle_limit = if control.active && shoulder {
            None
        } else if matches!(index, 7 | 10) && control.active {
            let (min, max) = joint_limits(index, control.direction);
            let current = wrap_angle(
                views[if index == 7 { 8 } else { 11 }].rotation
                    - views[if index == 7 { 7 } else { 10 }].rotation
                    - definition.reference,
            );
            if current >= min && current <= max {
                Some(AngleLimit::new(min, max))
            } else {
                None
            }
        } else {
            let (min, max) = joint_limits(index, control.direction);
            Some(AngleLimit::new(min, max))
        };
        if let Some(limit) = joint.angle_limit {
            joint.motor.target_position = joint.motor.target_position.clamp(limit.min, limit.max);
        }
    }
}

fn apply_locomotion(
    control: &mut RagdollControl,
    views: &[BodyView; 12],
    targets: &mut [f32; 11],
    input: &RagdollInput,
    spatial: &SpatialQuery,
    members: &Query<&RagdollMember>,
    dt: f32,
) {
    let moving = input.move_axis.abs() > 0.01;
    let capture_error = control.capture_point.x - control.support_center.x;
    let supported = control.grounded.iter().any(|grounded| *grounded);
    let needs_step = supported
        && (capture_error.abs() > 26.0 || control.com_velocity.x.abs() > 95.0);
    control.mode = if moving || control.gait.stepping || needs_step {
        LocomotionMode::Walking
    } else {
        LocomotionMode::Standing
    };

    if !control.gait.initialized {
        control.gait = GaitState::new([views[8].position, views[11].position]);
    }
    if !control.gait.stepping && supported && (moving || needs_step) {
        begin_step(control, views, input.move_axis, input.foot_bias, spatial, members);
    }

    if control.gait.stepping {
        control.gait.phase += dt / control.gait.duration;
        let t = control.gait.phase.clamp(0.0, 1.0);
        let blend = smootherstep(t);
        let swing = control.gait.swing.index();
        control.gait.feet[swing] = control
            .gait
            .start
            .lerp(control.gait.target, blend)
            + Vec2::Y * swing_height(t, STEP_HEIGHT);
        if t >= 1.0 {
            control.gait.feet[swing] = control.gait.target;
            control.gait.stepping = false;
            control.gait.swing = control.gait.swing.other();
        }
    }

    let desired_lean = -control.direction * input.move_axis.abs() * 0.08;
    let torso_error = wrap_angle(desired_lean - views[0].rotation);
    let torso_correction = (1.1 * torso_error - 0.07 * views[0].angular_velocity).clamp(-0.35, 0.35);
    let feedback = (0.010 * capture_error + 0.0025 * control.com_velocity.x).clamp(-0.40, 0.40);
    let foot_world = if control.direction > 0.0 { 0.0 } else { std::f32::consts::PI };

    for side in Side::ALL {
        let swinging = control.gait.stepping && side == control.gait.swing;
        let hip_adjust = torso_correction + if swinging { feedback } else { 0.0 };
        let ankle_adjust = if swinging { 0.0 } else { -0.30 * feedback };
        apply_leg_ik(
            control,
            views,
            targets,
            side,
            control.gait.feet[side.index()],
            foot_world + ankle_adjust,
            hip_adjust,
        );
    }
    apply_head(control, views, targets);
}

fn begin_step(
    control: &mut RagdollControl,
    views: &[BodyView; 12],
    move_axis: f32,
    foot_bias: f32,
    spatial: &SpatialQuery,
    members: &Query<&RagdollMember>,
) {
    let swing = if move_axis.abs() > 0.01 {
        if control.direction * control.com_velocity.x < 0.0 {
            control.gait.swing.other()
        } else {
            control.gait.swing
        }
    } else if control.capture_point.x > views[0].position.x {
        Side::Front
    } else {
        Side::Back
    };
    control.gait.swing = swing;
    control.gait.start = views[if swing == Side::Back { 8 } else { 11 }].position;
    let travel = if move_axis.abs() > 0.01 {
        move_axis.signum() * 34.0 + input_velocity_term(control)
    } else {
        0.0
    };
    let desired_x = control.capture_point.x + travel + control.direction * 10.0 + foot_bias;
    let hip_x = views[0].position.x;
    let target_x = desired_x.clamp(hip_x - 70.0, hip_x + 70.0);
    let proposed = Vec2::new(target_x, control.gait.ground_y + FOOT_HALF_HEIGHT);
    control.gait.target = project_foot_to_ground(
        spatial,
        members,
        proposed,
        control.gait.ground_y,
    );
    control.gait.phase = 0.0;
    control.gait.duration = (0.56 - control.com_velocity.length() * 0.0008).clamp(0.34, 0.56);
    control.gait.stepping = true;
}

fn input_velocity_term(control: &RagdollControl) -> f32 {
    (control.com_velocity.x * 0.12).clamp(-24.0, 24.0)
}

fn project_foot_to_ground(
    spatial: &SpatialQuery,
    members: &Query<&RagdollMember>,
    proposed: Vec2,
    fallback_ground: f32,
) -> Vec2 {
    let origin = proposed + Vec2::Y * 100.0;
    let filter = SpatialQueryFilter::default();
    let hit = spatial.cast_ray_predicate(
        origin,
        Dir2::NEG_Y,
        220.0,
        false,
        &filter,
        &|entity| !members.contains(entity),
    );
    if let Some(hit) = hit {
        Vec2::new(proposed.x, origin.y - hit.distance + FOOT_HALF_HEIGHT)
    } else {
        Vec2::new(proposed.x, fallback_ground + FOOT_HALF_HEIGHT)
    }
}

fn apply_arm_tasks(
    control: &mut RagdollControl,
    views: &[BodyView; 12],
    targets: &mut [f32; 11],
    input: &RagdollInput,
    pointer: &PointerInput,
    dt: f32,
) {
    for side in Side::ALL {
        let index = side.index();
        let shoulder_body = if side == Side::Back { 2 } else { 4 };
        let root = views[shoulder_body].position
            + rotate(views[shoulder_body].rotation, Vec2::new(0.0, 16.5));
        let desired = if let Some(grip) = control.grips[index] {
            grip.world_anchor
        } else {
            let manual = if side == Side::Back { pointer.left } else { pointer.right };
            if manual {
                pointer.world.unwrap_or(control.arm_targets[index])
            } else {
                let phase = if control.gait.stepping {
                    control.gait.phase * std::f32::consts::PI
                } else {
                    0.0
                };
                let opposition = if side == control.gait.swing { -1.0 } else { 1.0 };
                let swing = phase.sin() * opposition * 19.0;
                let crouch = if input.crouch { 18.0 } else { 0.0 };
                root + Vec2::new(
                    control.direction * (8.0 + 12.0 * index as f32 + swing),
                    -UPPER_ARM_LENGTH - LOWER_ARM_LENGTH + 10.0 + crouch,
                )
            }
        };
        let desired = clamp_target(root, desired, 4.0, ARM_REACH);
        let lambda = if control.grips[index].is_some() { 30.0 } else { 16.0 };
        control.arm_targets[index] = damp_vec2(control.arm_targets[index], desired, lambda, dt);
        apply_arm_ik(control, views, targets, side, control.arm_targets[index]);
    }
}

fn apply_arm_ik(
    control: &RagdollControl,
    views: &[BodyView; 12],
    targets: &mut [f32; 11],
    side: Side,
    target: Vec2,
) {
    let (upper, shoulder, elbow) = match side {
        Side::Back => (2, 1, 2),
        Side::Front => (4, 3, 4),
    };
    let root = views[upper].position + rotate(views[upper].rotation, Vec2::new(0.0, 16.5));
    let solution = two_bone_ik(
        root,
        target,
        UPPER_ARM_LENGTH,
        LOWER_ARM_LENGTH,
        -control.direction,
    );
    targets[shoulder] = wrap_angle(solution.upper_world - views[0].rotation);
    targets[elbow] = wrap_angle(solution.lower_world - solution.upper_world);
}

fn apply_leg_ik(
    control: &RagdollControl,
    views: &[BodyView; 12],
    targets: &mut [f32; 11],
    side: Side,
    foot_target: Vec2,
    foot_world: f32,
    hip_adjust: f32,
) {
    let (hip, knee, ankle) = match side {
        Side::Back => (5, 6, 7),
        Side::Front => (8, 9, 10),
    };
    let root = views[0].position
        + rotate(
            views[0].rotation,
            Vec2::new(0.0, -21.75),
        );
    let ankle_target = foot_target + rotate(foot_world, Vec2::new(-3.125, 0.0));
    let solution = two_bone_ik(
        root,
        ankle_target,
        THIGH_LENGTH,
        SHIN_LENGTH,
        control.direction,
    );
    targets[hip] = wrap_angle(solution.upper_world - views[0].rotation + hip_adjust);
    targets[knee] = wrap_angle(solution.lower_world - solution.upper_world);
    let reference = -std::f32::consts::FRAC_PI_2;
    targets[ankle] = wrap_angle(foot_world - solution.lower_world - reference);
}

fn apply_head(control: &RagdollControl, views: &[BodyView; 12], targets: &mut [f32; 11]) {
    let desired_world = -control.direction * 0.04;
    targets[0] = wrap_angle(desired_world - views[0].rotation);
}

fn apply_crouch(
    control: &RagdollControl,
    views: &[BodyView; 12],
    targets: &mut [f32; 11],
    amount: f32,
) {
    for side in Side::ALL {
        let (hip, knee, ankle, shin) = match side {
            Side::Back => (5, 6, 7, 7),
            Side::Front => (8, 9, 10, 10),
        };
        let hip_world = control.direction * 30.0_f32.to_radians() * amount;
        targets[hip] = wrap_angle(hip_world - views[0].rotation);
        targets[knee] = -control.direction * 82.0_f32.to_radians() * amount;
        let foot_world = if control.direction > 0.0 { 0.0 } else { std::f32::consts::PI };
        targets[ankle] = wrap_angle(
            foot_world - views[shin].rotation + std::f32::consts::FRAC_PI_2,
        );
    }
    apply_head(control, views, targets);
}

fn apply_air_brace(
    control: &RagdollControl,
    views: &[BodyView; 12],
    targets: &mut [f32; 11],
) {
    apply_crouch(control, views, targets, 0.35);
}

fn apply_jump(
    control: &mut RagdollControl,
    views: &[BodyView; 12],
    targets: &mut [f32; 11],
    dt: f32,
) {
    let timer = control.jump_timer.unwrap_or(0.0) + dt;
    control.jump_timer = Some(timer);
    if timer < 0.18 {
        apply_crouch(control, views, targets, timer / 0.18);
    } else {
        for side in Side::ALL {
            let (hip, knee, ankle, shin) = match side {
                Side::Back => (5, 6, 7, 7),
                Side::Front => (8, 9, 10, 10),
            };
            targets[hip] = wrap_angle(-views[0].rotation);
            targets[knee] = 0.0;
            let foot_world = if control.direction > 0.0 { 0.0 } else { std::f32::consts::PI };
            targets[ankle] = wrap_angle(
                foot_world - views[shin].rotation + std::f32::consts::FRAC_PI_2,
            );
        }
        apply_head(control, views, targets);
    }
    if timer > 0.85 {
        control.jump_timer = None;
    }
}

fn apply_hanging(
    control: &mut RagdollControl,
    views: &[BodyView; 12],
    targets: &mut [f32; 11],
) {
    apply_crouch(control, views, targets, 0.45);
}

#[allow(clippy::too_many_arguments)]
fn apply_recovery(
    recovery: &mut RecoveryState,
    control: &mut RagdollControl,
    views: &[BodyView; 12],
    targets: &mut [f32; 11],
    input: &RagdollInput,
    pointer: &PointerInput,
    spatial: &SpatialQuery,
    members: &Query<&RagdollMember>,
    dt: f32,
) {
    match recovery.phase {
        RecoveryPhase::Settle => {
            if recovery.timer > 0.45 {
                recovery.phase = RecoveryPhase::Roll;
                recovery.timer = 0.0;
            }
        }
        RecoveryPhase::Roll => {
            apply_tuck(control, views, targets);
            if recovery.timer > 0.85 {
                recovery.phase = RecoveryPhase::Crouch;
                recovery.timer = 0.0;
            }
        }
        RecoveryPhase::Crouch => {
            let center = views[0].position.x;
            for side in Side::ALL {
                let offset = if side == Side::Back { -18.0 } else { 18.0 };
                let proposed = Vec2::new(center + offset, control.gait.ground_y + FOOT_HALF_HEIGHT);
                let foot = project_foot_to_ground(spatial, members, proposed, control.gait.ground_y);
                control.gait.feet[side.index()] = foot;
                apply_leg_ik(control, views, targets, side, foot, 0.0, 0.0);
                let shoulder_root = views[0].position + Vec2::new(offset * 1.8, -10.0);
                let hand_target = project_foot_to_ground(
                    spatial,
                    members,
                    Vec2::new(shoulder_root.x, control.gait.ground_y + 4.0),
                    control.gait.ground_y,
                );
                apply_arm_ik(control, views, targets, side, hand_target);
            }
            apply_head(control, views, targets);
            if recovery.timer > 0.9 {
                recovery.phase = RecoveryPhase::Extend;
                recovery.timer = 0.0;
                control.gait.stepping = false;
            }
        }
        RecoveryPhase::Extend => {
            apply_locomotion(control, views, targets, input, spatial, members, dt);
            apply_arm_tasks(control, views, targets, input, pointer, dt);
            let upright = wrap_angle(views[0].rotation).abs() < 0.75;
            let grounded = control.grounded.iter().any(|grounded| *grounded);
            if recovery.timer > 1.25 && upright && grounded {
                control.recovery = None;
            } else if recovery.timer > 2.2 {
                recovery.attempts = recovery.attempts.saturating_add(1);
                recovery.phase = if recovery.attempts > 3 {
                    RecoveryPhase::Settle
                } else {
                    RecoveryPhase::Roll
                };
                recovery.timer = 0.0;
                if recovery.attempts > 3 {
                    recovery.attempts = 0;
                }
            }
        }
    }
}

fn apply_tuck(control: &RagdollControl, views: &[BodyView; 12], targets: &mut [f32; 11]) {
    apply_crouch(control, views, targets, 1.0);
    for side in Side::ALL {
        let (shoulder, elbow) = match side {
            Side::Back => (1, 2),
            Side::Front => (3, 4),
        };
        targets[shoulder] = control.direction * 55.0_f32.to_radians();
        targets[elbow] = -control.direction * 115.0_f32.to_radians();
    }
    let (hip_back, knee_back) = (5, 6);
    let (hip_front, knee_front) = (8, 9);
    targets[hip_back] = control.direction * 65.0_f32.to_radians() - views[0].rotation;
    targets[hip_front] = control.direction * 65.0_f32.to_radians() - views[0].rotation;
    targets[knee_back] = -control.direction * 110.0_f32.to_radians();
    targets[knee_front] = -control.direction * 110.0_f32.to_radians();
}

fn support_center(control: &RagdollControl, views: &[BodyView; 12]) -> Vec2 {
    let back = views[8].position;
    let front = views[11].position;
    match (control.grounded[0], control.grounded[1]) {
        (true, false) => back,
        (false, true) => front,
        _ => (back + front) * 0.5,
    }
}

fn estimate_ground_y(control: &RagdollControl, views: &[BodyView; 12]) -> f32 {
    let mut sum = 0.0;
    let mut count = 0.0;
    for side in Side::ALL {
        if control.grounded[side.index()] {
            let foot = views[if side == Side::Back { 8 } else { 11 }].position;
            sum += foot.y - FOOT_HALF_HEIGHT;
            count += 1.0;
        }
    }
    if count > 0.0 {
        sum / count
    } else {
        control.gait.ground_y
    }
}

fn hand_position(views: &[BodyView; 12], lower: usize) -> Vec2 {
    views[lower].position + rotate(views[lower].rotation, Vec2::new(0.0, -LOWER_ARM_LENGTH))
}

fn rotate(angle: f32, vector: Vec2) -> Vec2 {
    Vec2::from_angle(angle) * vector
}
