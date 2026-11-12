//! Pure 3D math for the WebGL view (no web-sys, host-testable): vec3 / column-major mat4,
//! damped orbit camera, screen rays, ray-plane / ray-sphere picking, frustum corners.
pub type V = [f32; 3];
/// Column-major 4×4 matrix (WebGL layout).
pub type M4 = [f32; 16];

pub fn add(a: V, b: V) -> V {
    [a[0] + b[0], a[1] + b[1], a[2] + b[2]]
}
pub fn sub(a: V, b: V) -> V {
    [a[0] - b[0], a[1] - b[1], a[2] - b[2]]
}
pub fn scale(a: V, s: f32) -> V {
    [a[0] * s, a[1] * s, a[2] * s]
}
pub fn dot(a: V, b: V) -> f32 {
    a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
}
pub fn cross(a: V, b: V) -> V {
    [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]]
}
pub fn norm(a: V) -> V {
    scale(a, 1.0 / dot(a, a).sqrt().max(1e-12))
}

pub const IDENTITY: M4 = [1., 0., 0., 0., 0., 1., 0., 0., 0., 0., 1., 0., 0., 0., 0., 1.];

pub fn mul(a: &M4, b: &M4) -> M4 {
    std::array::from_fn(|i| (0..4).map(|k| a[k * 4 + i % 4] * b[i / 4 * 4 + k]).sum())
}
pub fn perspective(fovy: f32, aspect: f32, near: f32, far: f32) -> M4 {
    let f = 1.0 / (fovy / 2.0).tan();
    let nf = 1.0 / (near - far);
    [f / aspect, 0., 0., 0., 0., f, 0., 0., 0., 0., (far + near) * nf, -1., 0., 0., 2. * far * near * nf, 0.]
}
pub fn look_at(eye: V, at: V, up: V) -> M4 {
    let f = norm(sub(at, eye));
    let s = norm(cross(f, up));
    let u = cross(s, f);
    [s[0], u[0], -f[0], 0., s[1], u[1], -f[1], 0., s[2], u[2], -f[2], 0., -dot(s, eye), -dot(u, eye), dot(f, eye), 1.]
}
/// General 4×4 inverse by cofactor expansion; `None` if singular.
pub fn invert(m: &M4) -> Option<M4> {
    // 3×3 minor determinant of the rows / columns not equal to (r, c), signed as a cofactor.
    let cof = |r: usize, c: usize| {
        let rs: Vec<usize> = (0..4).filter(|&i| i != r).collect();
        let cs: Vec<usize> = (0..4).filter(|&i| i != c).collect();
        let e = |i: usize, j: usize| m[cs[j] * 4 + rs[i]];
        let d = e(0, 0) * (e(1, 1) * e(2, 2) - e(1, 2) * e(2, 1)) - e(0, 1) * (e(1, 0) * e(2, 2) - e(1, 2) * e(2, 0))
            + e(0, 2) * (e(1, 0) * e(2, 1) - e(1, 1) * e(2, 0));
        if (r + c) % 2 == 0 { d } else { -d }
    };
    let det: f32 = (0..4).map(|c| m[c * 4] * cof(0, c)).sum();
    // inverse[r][c] = cof(c, r) / det; stored column-major at c * 4 + r.
    (det.abs() > 1e-12).then(|| std::array::from_fn(|i| cof(i / 4, i % 4) / det))
}
/// Transform a point (w = 1) and divide by w.
pub fn transform(m: &M4, p: V) -> V {
    let r = |i: usize| m[i] * p[0] + m[4 + i] * p[1] + m[8 + i] * p[2] + m[12 + i];
    let w = r(3);
    [r(0) / w, r(1) / w, r(2) / w]
}
pub fn translate(t: V) -> M4 {
    basis([1., 0., 0.], [0., 1., 0.], [0., 0., 1.], [1.; 3], t)
}
/// Model matrix from a 3×3 basis (columns x, y, z), per-axis scale `s` and translation `t`.
pub fn basis(x: V, y: V, z: V, s: V, t: V) -> M4 {
    let (x, y, z) = (scale(x, s[0]), scale(y, s[1]), scale(z, s[2]));
    [x[0], x[1], x[2], 0., y[0], y[1], y[2], 0., z[0], z[1], z[2], 0., t[0], t[1], t[2], 1.]
}
/// Flattens geometry onto the plane y = 0 along the direction `light` (pointing away from the light).
pub fn shadow_matrix(light: V) -> M4 {
    let (a, b) = (light[0] / light[1], light[2] / light[1]);
    [1., 0., 0., 0., -a, 0., -b, 0., 0., 0., 1., 0., 0., 0., 0., 1.]
}

/// Ray (origin, unit dir) through normalised device coordinates (x, y in -1..1, y up).
pub fn ndc_ray(inv_vp: &M4, x: f32, y: f32) -> (V, V) {
    let o = transform(inv_vp, [x, y, -1.0]);
    (o, norm(sub(transform(inv_vp, [x, y, 1.0]), o)))
}
/// Point where the ray (origin, unit dir) meets the plane through `p` with normal `n`, if in front.
pub fn ray_plane(o: V, d: V, p: V, n: V) -> Option<V> {
    let den = dot(d, n);
    if den.abs() < 1e-6 {
        return None;
    }
    let t = dot(sub(p, o), n) / den;
    (t > 0.0).then(|| add(o, scale(d, t)))
}
/// Distance along the ray to the first hit with the sphere, if any.
pub fn ray_sphere(o: V, d: V, c: V, r: f32) -> Option<f32> {
    let oc = sub(o, c);
    let b = dot(oc, d);
    let disc = b * b - (dot(oc, oc) - r * r);
    if disc < 0.0 {
        return None;
    }
    [-b - disc.sqrt(), -b + disc.sqrt()].into_iter().find(|t| *t >= 0.0)
}

/// The four corners (TL, TR, BR, BL) of a camera-frame frustum section at axis distance `d`,
/// for half-angle tangents `tx`, `ty` (camera frame: x right, y up, z forward).
pub fn frustum_corners(tx: f32, ty: f32, d: f32) -> [V; 4] {
    [(-tx, ty), (tx, ty), (tx, -ty), (-tx, -ty)].map(|(x, y)| [x * d, y * d, d])
}

/// Orbit pose: eye sits `dist` away from `target` at the given yaw / pitch (rad).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Orbit {
    pub target: V,
    pub yaw: f32,
    pub pitch: f32,
    pub dist: f32,
}

impl Orbit {
    pub fn eye(&self) -> V {
        let (sy, cy) = self.yaw.sin_cos();
        let (sp, cp) = self.pitch.sin_cos();
        add(self.target, scale([cp * sy, sp, cp * cy], self.dist))
    }
}

pub const PITCH_MIN: f32 = -0.1;
pub const PITCH_MAX: f32 = 1.55;

/// Orbit camera with smooth damping toward a goal pose and inertia after a drag is released.
#[derive(Clone, Copy, Debug)]
pub struct OrbitCam {
    pub cur: Orbit,
    pub goal: Orbit,
    /// Per-frame (1/60 s) velocity: yaw, pitch, pan x, pan y, pan z.
    pub vel: [f32; 5],
    pub held: bool,
}

impl OrbitCam {
    pub fn new(o: Orbit) -> Self {
        OrbitCam { cur: o, goal: o, vel: [0.0; 5], held: false }
    }
    /// Rotate by a drag delta (rad); remembered as velocity for inertia.
    pub fn rotate(&mut self, dyaw: f32, dpitch: f32) {
        self.goal.yaw += dyaw;
        self.goal.pitch = (self.goal.pitch + dpitch).clamp(PITCH_MIN, PITCH_MAX);
        self.vel = [dyaw, dpitch, 0.0, 0.0, 0.0];
    }
    /// Pan the target by a world-space offset; remembered as velocity for inertia.
    pub fn pan(&mut self, off: V) {
        self.goal.target = add(self.goal.target, off);
        self.vel = [0.0, 0.0, off[0], off[1], off[2]];
    }
    /// Multiply the goal distance (wheel / pinch), clamped to a sane range.
    pub fn zoom(&mut self, factor: f32) {
        self.goal.dist = (self.goal.dist * factor).clamp(0.05, 500.0);
    }
    /// Jump the goal (presets); the current pose glides there.
    pub fn set_goal(&mut self, o: Orbit) {
        self.goal = o;
        self.vel = [0.0; 5];
    }
    /// Advance by `dt` seconds. Returns true while still moving (keep rendering).
    pub fn step(&mut self, dt: f32) -> bool {
        let f = dt * 60.0;
        if self.held {
            // Holding still mid-drag kills the fling.
            self.vel = self.vel.map(|v| v * 0.5f32.powf(f));
        } else {
            let v = self.vel;
            self.goal.yaw += v[0] * f;
            self.goal.pitch = (self.goal.pitch + v[1] * f).clamp(PITCH_MIN, PITCH_MAX);
            self.goal.target = add(self.goal.target, scale([v[2], v[3], v[4]], f));
            self.vel = v.map(|v| v * 0.9f32.powf(f));
        }
        let k = 1.0 - (-dt * 14.0).exp();
        let (c, g) = (&mut self.cur, self.goal);
        c.yaw += (g.yaw - c.yaw) * k;
        c.pitch += (g.pitch - c.pitch) * k;
        c.dist += (g.dist - c.dist) * k;
        c.target = add(c.target, scale(sub(g.target, c.target), k));
        let eps = 1e-4 * g.dist;
        let off = sub(g.target, c.target);
        let moving = self.vel.iter().any(|v| v.abs() > 1e-5)
            || (g.yaw - c.yaw).abs() > 1e-4
            || (g.pitch - c.pitch).abs() > 1e-4
            || (g.dist - c.dist).abs() > eps
            || dot(off, off) > eps * eps;
        if !moving {
            self.vel = [0.0; 5];
            *c = g;
        }
        moving
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn close(a: f32, b: f32, tol: f32) {
        assert!((a - b).abs() < tol, "{a} vs {b}");
    }
    fn close_v(a: V, b: V, tol: f32) {
        (0..3).for_each(|i| close(a[i], b[i], tol));
    }
    fn vp() -> M4 {
        mul(&perspective(0.9, 1.5, 0.05, 100.0), &look_at([2.0, 3.0, -4.0], [0.0, 0.5, 1.0], [0.0, 1.0, 0.0]))
    }

    #[test]
    fn vector_basics() {
        assert_eq!(cross([1., 0., 0.], [0., 1., 0.]), [0., 0., 1.]);
        close(dot([1., 2., 3.], [4., 5., 6.]), 32.0, 1e-6);
        close_v(norm([3., 0., 4.]), [0.6, 0., 0.8], 1e-6);
        assert_eq!(sub(add([1., 2., 3.], scale([1., 1., 1.], 2.)), [3., 4., 5.]), [0., 0., 0.]);
    }

    #[test]
    fn invert_times_matrix_is_identity() {
        let m = vp();
        let p = mul(&invert(&m).unwrap(), &m);
        (0..16).for_each(|i| close(p[i], IDENTITY[i], 1e-4));
        let t = invert(&translate([1., 2., 3.])).unwrap();
        close_v(transform(&t, [1., 2., 3.]), [0.; 3], 1e-6);
        assert!(invert(&[0.0; 16]).is_none());
    }

    #[test]
    fn look_at_puts_target_in_screen_centre() {
        let c = transform(&vp(), [0.0, 0.5, 1.0]);
        close(c[0], 0.0, 1e-5);
        close(c[1], 0.0, 1e-5);
        assert!(c[2] > -1.0 && c[2] < 1.0);
        // Right-handed view: a point to the right of the view direction lands at x > 0.
        let v = look_at([0., 0., 0.], [0., 0., 1.], [0., 1., 0.]);
        assert!(transform(&mul(&perspective(1.0, 1.0, 0.1, 10.0), &v), [-1.0, 0.0, 3.0])[0] > 0.0);
    }

    #[test]
    fn screen_ray_passes_through_projected_point() {
        let m = vp();
        let p = [0.7, 0.2, 2.5];
        let s = transform(&m, p);
        let (o, d) = ndc_ray(&invert(&m).unwrap(), s[0], s[1]);
        close(dot(d, d), 1.0, 1e-5);
        let t = dot(sub(p, o), d);
        close_v(add(o, scale(d, t)), p, 1e-3);
    }

    #[test]
    fn ray_plane_hits_ground() {
        let d = norm([0., -1., 1.]);
        close_v(ray_plane([0., 1., 0.], d, [0.; 3], [0., 1., 0.]).unwrap(), [0., 0., 1.], 1e-6);
        assert!(ray_plane([0., 1., 0.], [0., 0., 1.], [0.; 3], [0., 1., 0.]).is_none(), "parallel");
        assert!(ray_plane([0., 1., 0.], norm([0., 1., 1.]), [0.; 3], [0., 1., 0.]).is_none(), "behind");
    }

    #[test]
    fn ray_sphere_hit_and_miss() {
        close(ray_sphere([0., 0., 0.], [0., 0., 1.], [0., 0., 5.], 1.0).unwrap(), 4.0, 1e-5);
        assert!(ray_sphere([0., 0., 0.], [0., 0., 1.], [0., 2., 5.], 1.0).is_none());
        assert!(ray_sphere([0., 0., 0.], [0., 0., 1.], [0., 0., -5.], 1.0).is_none(), "behind");
    }

    #[test]
    fn shadow_flattens_along_light() {
        let s = shadow_matrix([0., -1., 0.]);
        close_v(transform(&s, [1., 2., 3.]), [1., 0., 3.], 1e-6);
        let s = shadow_matrix(norm([1., -1., 0.]));
        close_v(transform(&s, [0., 2., 0.]), [2., 0., 0.], 1e-5);
    }

    #[test]
    fn orbit_glides_to_goal_and_inertia_stops() {
        let o = Orbit { target: [0.; 3], yaw: 0.0, pitch: 0.3, dist: 5.0 };
        close_v(o.eye(), [0.0, 5.0 * 0.3f32.sin(), 5.0 * 0.3f32.cos()], 1e-5);
        let mut c = OrbitCam::new(o);
        assert!(!c.step(1.0 / 60.0), "at rest");
        c.zoom(2.0);
        c.set_goal(Orbit { yaw: 1.0, ..c.goal });
        let mut frames = 0;
        while c.step(1.0 / 60.0) {
            frames += 1;
            assert!(frames < 600, "never settles");
        }
        close(c.cur.dist, 10.0, 1e-2);
        close(c.cur.yaw, 1.0, 1e-2);
        // Drag then release: keeps spinning for a while, then stops.
        c.held = true;
        c.rotate(0.05, 10.0);
        close(c.goal.pitch, PITCH_MAX, 1e-6);
        c.held = false;
        let before = c.goal.yaw;
        assert!(c.step(1.0 / 60.0));
        assert!(c.goal.yaw > before, "inertia");
        while c.step(1.0 / 60.0) {}
        assert!(c.goal.yaw < before + 1.0);
        c.zoom(1e-9);
        assert!(c.goal.dist > 0.0);
    }
}
