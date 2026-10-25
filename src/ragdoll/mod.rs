mod control;
mod debug;
mod input;
mod math;

use avian2d::prelude::*;
use bevy::ecs::system::SystemParam;
use bevy::prelude::*;

pub use input::Grabbable;

use control::{control_ragdolls, update_foot_contacts};
use debug::{draw_debug, setup_hud, update_hud, RagdollDebug};
use input::{clear_actions, read_keyboard, read_pointer, PointerInput, RagdollInput};

const INITIAL_POSITION: Vec2 = Vec2::new(-420.0, -120.0);
const FOOT_FRICTION: f32 = 20.0;

pub struct RagdollPlugin;

impl Plugin for RagdollPlugin {
    fn build(&self, app: &mut App) {
        app.init_resource::<RagdollInput>()
            .init_resource::<PointerInput>()
            .init_resource::<RagdollDebug>()
            .add_systems(Startup, (spawn_initial_ragdoll, setup_hud))
            .add_systems(
                Update,
                (
                    read_pointer,
                    read_keyboard,
                    spawn_requested_ragdoll,
                    update_hud,
                    draw_debug,
                )
                    .chain(),
            )
            .add_systems(
                FixedPreUpdate,
                (update_foot_contacts, control_ragdolls, clear_actions).chain(),
            );
    }
}

#[derive(Component, Debug)]
pub struct Ragdoll {
    pub bodies: [Entity; 12],
    pub joints: [Entity; 11],
    pub sensors: [Entity; 2],
}

#[derive(Component, Clone, Copy, Debug)]
pub struct RagdollMember {
    pub root: Entity,
    pub part: RagdollPart,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum RagdollPart {
    Chest,
    Head,
    BackUpperArm,
    BackLowerArm,
    FrontUpperArm,
    FrontLowerArm,
    BackThigh,
    BackShin,
    BackFoot,
    FrontThigh,
    FrontShin,
    FrontFoot,
    FootSensor(Side),
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum Side {
    Back = 0,
    Front = 1,
}

impl Side {
    pub const ALL: [Side; 2] = [Side::Back, Side::Front];

    pub fn index(self) -> usize {
        self as usize
    }

    pub fn other(self) -> Self {
        match self {
            Self::Back => Self::Front,
            Self::Front => Self::Back,
        }
    }
}

#[derive(Component)]
pub struct FootSensor {
    pub root: Entity,
    pub side: Side,
}

#[derive(Component, Clone, Copy)]
pub struct RagdollJoint {
    pub body_indices: (usize, usize),
    pub reference: f32,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum LocomotionMode {
    Passive,
    Standing,
    Walking,
    Crouching,
    Jumping,
    Hanging,
    Recovering,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum RecoveryPhase {
    Settle,
    Roll,
    Crouch,
    Extend,
}

#[derive(Clone, Copy, Debug)]
pub struct RecoveryState {
    pub phase: RecoveryPhase,
    pub timer: f32,
    pub attempts: u8,
}

#[derive(Clone, Copy, Debug)]
pub struct Grip {
    pub joint: Entity,
    pub body: Entity,
    pub object_local: Vec2,
    pub world_anchor: Vec2,
    pub fixed_support: bool,
}

#[derive(Clone, Copy, Debug)]
pub struct GaitState {
    pub initialized: bool,
    pub stepping: bool,
    pub swing: Side,
    pub phase: f32,
    pub duration: f32,
    pub start: Vec2,
    pub target: Vec2,
    pub feet: [Vec2; 2],
    pub ground_y: f32,
}

impl GaitState {
    fn new(feet: [Vec2; 2]) -> Self {
        let stance = [
            feet[0] + Vec2::new(-16.0, 0.0),
            feet[1] + Vec2::new(16.0, 0.0),
        ];
        Self {
            initialized: true,
            stepping: false,
            swing: Side::Back,
            phase: 0.0,
            duration: 0.52,
            start: stance[0],
            target: stance[0],
            feet: stance,
            ground_y: feet[0].y - math::FOOT_HALF_HEIGHT,
        }
    }
}

#[derive(Component, Debug)]
pub struct RagdollControl {
    pub active: bool,
    pub direction: f32,
    pub mode: LocomotionMode,
    pub grounded: [bool; 2],
    pub air_time: f32,
    pub rest_time: f32,
    pub fallen_time: f32,
    pub recovery: Option<RecoveryState>,
    pub jump_timer: Option<f32>,
    pub gait: GaitState,
    pub grips: [Option<Grip>; 2],
    pub arm_targets: [Vec2; 2],
    pub com: Vec2,
    pub com_velocity: Vec2,
    pub capture_point: Vec2,
    pub support_center: Vec2,
    pub debug_enabled: bool,
}

impl RagdollControl {
    fn new(position: Vec2) -> Self {
        let feet = [
            position + BODY_SPECS[8].position,
            position + BODY_SPECS[11].position,
        ];
        Self {
            active: true,
            direction: 1.0,
            mode: LocomotionMode::Standing,
            grounded: [false; 2],
            air_time: 0.0,
            rest_time: 0.0,
            fallen_time: 0.0,
            recovery: None,
            jump_timer: None,
            gait: GaitState::new(feet),
            grips: [None, None],
            arm_targets: feet,
            com: position,
            com_velocity: Vec2::ZERO,
            capture_point: position,
            support_center: feet[0],
            debug_enabled: false,
        }
    }
}

#[derive(SystemParam)]
pub struct RagdollCollisionHooks<'w, 's> {
    members: Query<'w, 's, &'static RagdollMember>,
}

impl CollisionHooks for RagdollCollisionHooks<'_, '_> {
    fn filter_pairs(
        &self,
        collider1: Entity,
        collider2: Entity,
        _commands: &mut Commands,
    ) -> bool {
        let (Ok(first), Ok(second)) = (
            self.members.get(collider1),
            self.members.get(collider2),
        ) else {
            return true;
        };
        first.root != second.root
    }
}

#[derive(Clone, Copy)]
enum BodyShape {
    Rectangle(Vec2),
    Circle(f32),
}

#[derive(Clone, Copy)]
struct BodySpec {
    name: &'static str,
    part: RagdollPart,
    position: Vec2,
    shape: BodyShape,
    friction: f32,
}

const BODY_SPECS: [BodySpec; 12] = [
    BodySpec { name: "Torse", part: RagdollPart::Chest, position: Vec2::new(0.0, 0.0), shape: BodyShape::Rectangle(Vec2::new(11.5, 43.5)), friction: 5.0 },
    BodySpec { name: "Tête", part: RagdollPart::Head, position: Vec2::new(0.0, 41.0), shape: BodyShape::Circle(13.25), friction: 5.0 },
    BodySpec { name: "Bras arrière supérieur", part: RagdollPart::BackUpperArm, position: Vec2::new(0.0, 3.25), shape: BodyShape::Rectangle(Vec2::new(6.0, 33.0)), friction: 5.0 },
    BodySpec { name: "Bras arrière inférieur", part: RagdollPart::BackLowerArm, position: Vec2::new(0.0, -29.75), shape: BodyShape::Rectangle(Vec2::new(4.8, 32.0)), friction: 5.0 },
    BodySpec { name: "Bras avant supérieur", part: RagdollPart::FrontUpperArm, position: Vec2::new(0.0, 3.25), shape: BodyShape::Rectangle(Vec2::new(6.0, 33.0)), friction: 5.0 },
    BodySpec { name: "Bras avant inférieur", part: RagdollPart::FrontLowerArm, position: Vec2::new(0.0, -29.75), shape: BodyShape::Rectangle(Vec2::new(4.8, 32.0)), friction: 5.0 },
    BodySpec { name: "Cuisse arrière", part: RagdollPart::BackThigh, position: Vec2::new(0.0, -41.25), shape: BodyShape::Rectangle(Vec2::new(8.85, 39.0)), friction: 5.0 },
    BodySpec { name: "Tibia arrière", part: RagdollPart::BackShin, position: Vec2::new(0.0, -78.5), shape: BodyShape::Rectangle(Vec2::new(6.75, 35.5)), friction: 5.0 },
    BodySpec { name: "Pied arrière", part: RagdollPart::BackFoot, position: Vec2::new(3.125, -100.915), shape: BodyShape::Rectangle(Vec2::new(15.0, 4.33)), friction: FOOT_FRICTION },
    BodySpec { name: "Cuisse avant", part: RagdollPart::FrontThigh, position: Vec2::new(0.0, -41.25), shape: BodyShape::Rectangle(Vec2::new(8.85, 39.0)), friction: 5.0 },
    BodySpec { name: "Tibia avant", part: RagdollPart::FrontShin, position: Vec2::new(0.0, -78.5), shape: BodyShape::Rectangle(Vec2::new(6.75, 35.5)), friction: 5.0 },
    BodySpec { name: "Pied avant", part: RagdollPart::FrontFoot, position: Vec2::new(3.125, -100.915), shape: BodyShape::Rectangle(Vec2::new(15.0, 4.33)), friction: FOOT_FRICTION },
];

struct JointSpec {
    bodies: (usize, usize),
    anchor1: Vec2,
    anchor2: Vec2,
    old_reference: f32,
}

fn joint_spec(index: usize) -> JointSpec {
    let (bodies, anchor1, anchor2, old_reference) = match index {
        0 => ((0, 1), Vec2::new(0.0, 23.75), Vec2::new(0.0, -17.25), 0.0),
        1 => ((0, 2), Vec2::new(0.0, 19.75), Vec2::new(0.0, 16.5), 0.0),
        2 => ((2, 3), Vec2::new(0.0, -17.0), Vec2::new(0.0, 16.0), 0.0),
        3 => ((0, 4), Vec2::new(0.0, 19.75), Vec2::new(0.0, 16.5), 0.0),
        4 => ((4, 5), Vec2::new(0.0, -17.0), Vec2::new(0.0, 16.0), 0.0),
        5 => ((0, 6), Vec2::new(0.0, -21.75), Vec2::new(0.0, 19.5), 0.0),
        6 => ((6, 7), Vec2::new(0.0, -19.5), Vec2::new(0.0, 17.75), 0.0),
        7 => ((7, 8), Vec2::new(0.0, -22.415), Vec2::new(-3.125, 0.0), 90.0),
        8 => ((0, 9), Vec2::new(0.0, -21.75), Vec2::new(0.0, 19.5), 0.0),
        9 => ((9, 10), Vec2::new(0.0, -19.5), Vec2::new(0.0, 17.75), 0.0),
        10 => ((10, 11), Vec2::new(0.0, -22.415), Vec2::new(-3.125, 0.0), 90.0),
        _ => unreachable!(),
    };
    JointSpec { bodies, anchor1, anchor2, old_reference }
}

pub fn joint_limits(index: usize, direction: f32) -> (f32, f32) {
    let right = match index {
        0 => (-70.0, 70.0),
        1 | 3 => (-65.0, 215.0),
        2 | 4 => (0.0, 150.0),
        5 | 8 => (-60.0, 140.0),
        6 | 9 => (-140.0, 0.0),
        7 | 10 => (50.0, 120.0),
        _ => unreachable!(),
    };
    if direction > 0.0 {
        (right.0.to_radians(), right.1.to_radians())
    } else {
        (-right.1.to_radians(), -right.0.to_radians())
    }
}

fn initial_target(index: usize) -> f32 {
    match index {
        1 | 3 => (-8.0_f32).to_radians(),
        2 | 4 => 28.0_f32.to_radians(),
        7 | 10 => std::f32::consts::FRAC_PI_2,
        _ => 0.0,
    }
}

fn motor_settings(index: usize) -> (f32, f32) {
    match index {
        0 => (4.0, 8_000.0),
        1 | 3 => (5.0, 120_000.0),
        2 | 4 => (6.0, 80_000.0),
        5 | 8 => (5.0, 220_000.0),
        6 | 9 => (6.0, 180_000.0),
        7 | 10 => (7.0, 150_000.0),
        _ => unreachable!(),
    }
}

fn render_z(index: usize) -> f32 {
    match index {
        2 | 3 => 0.0,
        6 | 7 | 8 => 1.0,
        0 | 1 => 2.0,
        9 | 10 | 11 => 3.0,
        4 | 5 => 4.0,
        _ => 0.0,
    }
}

fn body_color(index: usize) -> Color {
    match index {
        0 | 6 | 9 => Color::srgb(0.12, 0.20, 0.34),
        1 => Color::srgb(0.96, 0.78, 0.58),
        2 | 3 | 7 | 10 => Color::srgb(0.56, 0.68, 0.78),
        4 | 5 | 11 => Color::srgb(0.90, 0.88, 0.76),
        8 => Color::srgb(0.25, 0.30, 0.36),
        _ => Color::WHITE,
    }
}

fn spawn_initial_ragdoll(
    commands: Commands,
    meshes: ResMut<Assets<Mesh>>,
    materials: ResMut<Assets<ColorMaterial>>,
) {
    spawn_ragdoll(commands, meshes, materials, INITIAL_POSITION);
}

fn spawn_requested_ragdoll(
    commands: Commands,
    meshes: ResMut<Assets<Mesh>>,
    materials: ResMut<Assets<ColorMaterial>>,
    mut input: ResMut<RagdollInput>,
) {
    if input.spawn_requested {
        input.spawn_requested = false;
        spawn_ragdoll(commands, meshes, materials, INITIAL_POSITION + Vec2::new(0.0, 180.0));
    }
}

fn spawn_ragdoll(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<ColorMaterial>>,
    position: Vec2,
) {
    let root = commands
        .spawn((Name::new("Ragdoll actif"), RagdollControl::new(position)))
        .id();
    let mut bodies = [Entity::PLACEHOLDER; 12];
    let mut joints = [Entity::PLACEHOLDER; 11];
    let mut sensors = [Entity::PLACEHOLDER; 2];

    for (index, spec) in BODY_SPECS.iter().enumerate() {
        let color = body_color(index);
        let mut entity = commands.spawn((
            Name::new(spec.name),
            Transform::from_translation((position + spec.position).extend(render_z(index))),
            RigidBody::Dynamic,
            ColliderDensity(1.0),
            Friction::new(spec.friction),
            Restitution::ZERO,
            LinearDamping(0.01),
            AngularDamping(0.05),
            ActiveCollisionHooks::FILTER_PAIRS,
            RagdollMember { root, part: spec.part },
        ));
        match spec.shape {
            BodyShape::Rectangle(size) => {
                entity.insert((Sprite::from_color(color, size), Collider::rectangle(size.x, size.y)));
            }
            BodyShape::Circle(radius) => {
                entity.insert((
                    Mesh2d(meshes.add(Circle::new(radius))),
                    MeshMaterial2d(materials.add(color)),
                    Collider::circle(radius),
                ));
            }
        }
        bodies[index] = entity.id();
    }

    for side in Side::ALL {
        let foot = bodies[match side {
            Side::Back => 8,
            Side::Front => 11,
        }];
        sensors[side.index()] = commands
            .spawn((
                Name::new("Capteur de pied"),
                ChildOf(foot),
                Transform::IDENTITY,
                Collider::circle(30.0),
                NoAutoMass,
                NoAutoAngularInertia,
                NoAutoCenterOfMass,
                Sensor,
                CollisionEventsEnabled,
                CollidingEntities::default(),
                ActiveCollisionHooks::FILTER_PAIRS,
                RagdollMember { root, part: RagdollPart::FootSensor(side) },
                FootSensor { root, side },
            ))
            .id();
    }

    for index in 0..11 {
        let spec = joint_spec(index);
        let (frequency, max_torque) = motor_settings(index);
        let reference = -spec.old_reference.to_radians();
        let mut joint = RevoluteJoint::new(bodies[spec.bodies.0], bodies[spec.bodies.1])
            .with_local_anchor1(spec.anchor1)
            .with_local_anchor2(spec.anchor2)
            .with_local_basis2(Rotation::radians(spec.old_reference.to_radians()))
            .with_motor(
                AngularMotor::new(MotorModel::SpringDamper {
                    frequency,
                    damping_ratio: 1.0,
                })
                .with_target_position(initial_target(index))
                .with_max_torque(max_torque),
            );
        if !matches!(index, 1 | 3) {
            let (min, max) = joint_limits(index, 1.0);
            joint = joint.with_angle_limits(min, max);
        }
        joints[index] = commands
            .spawn((
                Name::new(format!("Articulation {index}")),
                joint,
                RagdollJoint {
                    body_indices: spec.bodies,
                    reference,
                },
            ))
            .id();
    }

    commands.entity(root).insert(Ragdoll { bodies, joints, sensors });
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn left_limits_are_the_mirror_of_right_limits() {
        for index in 0..11 {
            let right = joint_limits(index, 1.0);
            let left = joint_limits(index, -1.0);
            assert!((left.0 + right.1).abs() < 1e-5);
            assert!((left.1 + right.0).abs() < 1e-5);
        }
    }

    #[test]
    fn every_joint_anchor_pair_matches_in_the_initial_pose() {
        for index in 0..11 {
            let spec = joint_spec(index);
            let first = BODY_SPECS[spec.bodies.0].position + spec.anchor1;
            let second = BODY_SPECS[spec.bodies.1].position + spec.anchor2;
            assert!(
                first.distance(second) < 1e-4,
                "articulation {index}: {first:?} != {second:?}"
            );
        }
    }

    #[test]
    fn converted_ankle_starts_inside_its_limits() {
        let right = joint_limits(7, 1.0);
        let left = joint_limits(7, -1.0);
        assert!(std::f32::consts::FRAC_PI_2 > right.0 && std::f32::consts::FRAC_PI_2 < right.1);
        assert!(-std::f32::consts::FRAC_PI_2 > left.0 && -std::f32::consts::FRAC_PI_2 < left.1);
    }
}
