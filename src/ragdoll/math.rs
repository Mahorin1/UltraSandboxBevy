use bevy::prelude::*;

pub const FOOT_HALF_HEIGHT: f32 = 2.165;
pub const UPPER_ARM_LENGTH: f32 = 33.5;
pub const LOWER_ARM_LENGTH: f32 = 32.0;
pub const THIGH_LENGTH: f32 = 39.0;
pub const SHIN_LENGTH: f32 = 40.165;

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct IkSolution {
    pub upper_world: f32,
    pub lower_world: f32,
    pub elbow: Vec2,
    pub reachable: bool,
}

/// Intersection de deux cercles, adaptée d'Old `IntersectTwoCircles`.
/// Les angles décrivent la direction du vecteur local -Y dans le monde Bevy.
pub fn two_bone_ik(root: Vec2, target: Vec2, upper: f32, lower: f32, bend: f32) -> IkSolution {
    let delta = target - root;
    let raw_distance = delta.length();
    let direction = if raw_distance > 1e-4 {
        delta / raw_distance
    } else {
        Vec2::NEG_Y
    };
    let min_distance = (upper - lower).abs() + 1e-3;
    let max_distance = upper + lower - 1e-3;
    let distance = raw_distance.clamp(min_distance, max_distance);
    let end = root + direction * distance;
    let along = (upper * upper - lower * lower + distance * distance) / (2.0 * distance);
    let height = (upper * upper - along * along).max(0.0).sqrt();
    let perpendicular = Vec2::new(-direction.y, direction.x);
    let elbow = root + direction * along + perpendicular * height * bend.signum();
    let upper_world = angle_from_down(elbow - root);
    let lower_world = angle_from_down(end - elbow);

    IkSolution {
        upper_world,
        lower_world,
        elbow,
        reachable: raw_distance >= min_distance && raw_distance <= max_distance,
    }
}

pub fn angle_from_down(vector: Vec2) -> f32 {
    vector.y.atan2(vector.x) + std::f32::consts::FRAC_PI_2
}

pub fn wrap_angle(angle: f32) -> f32 {
    (angle + std::f32::consts::PI).rem_euclid(std::f32::consts::TAU)
        - std::f32::consts::PI
}

pub fn damp(current: f32, target: f32, lambda: f32, dt: f32) -> f32 {
    current + (target - current) * (1.0 - (-lambda * dt).exp())
}

pub fn damp_vec2(current: Vec2, target: Vec2, lambda: f32, dt: f32) -> Vec2 {
    current + (target - current) * (1.0 - (-lambda * dt).exp())
}

/// Courbe quintique à vitesse et accélération nulles aux deux extrémités.
pub fn smootherstep(t: f32) -> f32 {
    let t = t.clamp(0.0, 1.0);
    t * t * t * (10.0 + t * (-15.0 + 6.0 * t))
}

pub fn swing_height(t: f32, height: f32) -> f32 {
    let t = t.clamp(0.0, 1.0);
    4.0 * height * t * (1.0 - t)
}

/// Approximation du capture point dans le plan sagittal, en pixels.
pub fn capture_point(com: Vec2, com_velocity: Vec2, ground_y: f32, gravity: f32) -> Vec2 {
    let height = (com.y - ground_y).max(20.0);
    let omega = (gravity / height).sqrt();
    Vec2::new(com.x + com_velocity.x / omega, ground_y)
}

pub fn clamp_target(root: Vec2, target: Vec2, min: f32, max: f32) -> Vec2 {
    let delta = target - root;
    let distance = delta.length();
    if distance < 1e-4 {
        root + Vec2::NEG_Y * min
    } else {
        root + delta / distance * distance.clamp(min, max)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn wrap_angle_uses_the_shortest_arc() {
        assert!((wrap_angle(3.0 * std::f32::consts::PI).abs() - std::f32::consts::PI).abs() < 1e-4);
        assert!(wrap_angle(std::f32::consts::TAU).abs() < 1e-4);
    }

    #[test]
    fn quintic_has_zero_end_derivatives() {
        assert_eq!(smootherstep(0.0), 0.0);
        assert_eq!(smootherstep(1.0), 1.0);
        let epsilon = 1e-4;
        assert!(smootherstep(epsilon) < 1e-8);
        assert!((1.0 - smootherstep(1.0 - epsilon)) < 1e-8);
    }

    #[test]
    fn ik_reaches_a_target_inside_the_workspace() {
        let target = Vec2::new(15.0, -45.0);
        let solution = two_bone_ik(Vec2::ZERO, target, 33.5, 32.0, -1.0);
        assert!(solution.reachable);
        assert!((solution.elbow.distance(Vec2::ZERO) - 33.5).abs() < 1e-3);
    }

    #[test]
    fn ik_clamps_an_unreachable_target() {
        let solution = two_bone_ik(Vec2::ZERO, Vec2::new(200.0, 0.0), 33.5, 32.0, 1.0);
        assert!(!solution.reachable);
        assert!(solution.elbow.is_finite());
        assert!(solution.upper_world.is_finite());
        assert!(solution.lower_world.is_finite());
    }

    #[test]
    fn capture_point_moves_in_the_direction_of_velocity() {
        let com = Vec2::new(0.0, 100.0);
        let point = capture_point(com, Vec2::new(50.0, 0.0), 0.0, 981.0);
        assert!(point.x > com.x);
        assert_eq!(point.y, 0.0);
    }
}
