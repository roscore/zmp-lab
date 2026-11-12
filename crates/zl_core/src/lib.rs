//! Linear Inverted Pendulum (LIPM) walking with a capture-point (DCM) controller.
//!
//! Floor frame: x forward, y to the left, metres. Textbook equations only:
//! x'' = ω²(x − p), ω = √(g / z_c), capture point ξ = x + ẋ / ω, ξ' = ω(ξ − p).
pub mod geom;
use geom::{clamp, hull, lerp, rect, signed_dist, sub, P2};

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Params {
    /// CoM height z_c (m).
    pub z_c: f64,
    pub g: f64,
    pub foot_l: f64,
    pub foot_w: f64,
    pub step_len: f64,
    /// Lateral distance between the foot centres (m).
    pub step_w: f64,
    /// Single-support time (s).
    pub t_ss: f64,
    /// Double-support share of one step, T_ds / (T_ss + T_ds), 0…0.4.
    pub ds_ratio: f64,
    /// Number of swings (2…10); the last one brings the feet side by side.
    pub steps: usize,
    /// DCM feedback gain k (> 0): p = p_ref + (1 + k)(ξ − ξ_ref), error decays as e^(−kωt).
    pub k: f64,
}

impl Params {
    /// Adult-size humanoid, typical values (editable).
    pub const ADULT: Params =
        Params { z_c: 0.80, g: 9.81, foot_l: 0.22, foot_w: 0.12, step_len: 0.25, step_w: 0.20, t_ss: 0.70, ds_ratio: 0.2, steps: 6, k: 1.5 };
    /// Small kid-size humanoid, typical values (editable).
    pub const KID: Params =
        Params { z_c: 0.28, g: 9.81, foot_l: 0.10, foot_w: 0.06, step_len: 0.07, step_w: 0.08, t_ss: 0.35, ds_ratio: 0.2, steps: 6, k: 1.5 };

    pub fn omega(&self) -> f64 {
        omega(self.g, self.z_c)
    }
    pub fn t_ds(&self) -> f64 {
        let r = self.ds_ratio.clamp(0.0, 0.9);
        self.t_ss * r / (1.0 - r)
    }
    /// Swing-foot apex height (m), scaled with the robot.
    pub fn lift(&self) -> f64 {
        0.12 * self.z_c
    }
}

pub fn omega(g: f64, z_c: f64) -> f64 {
    (g / z_c).sqrt()
}

/// Capture point (DCM) ξ = x + v / ω.
pub fn capture_point(x: P2, v: P2, w: f64) -> P2 {
    [x[0] + v[0] / w, x[1] + v[1] / w]
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Foot {
    pub pos: P2,
    pub left: bool,
}

/// Footsteps: 0 = left, 1 = right standing side by side at x = 0, then `steps` swings alternating
/// left / right; the final swing lands beside the previous foot.
pub fn footsteps(p: &Params) -> Vec<Foot> {
    let n = p.steps.clamp(2, 10);
    (0..n + 2)
        .map(|k| {
            let i = if k < 2 { 0 } else { (k - 1).min(n - 1) };
            let left = k % 2 == 0;
            Foot { pos: [i as f64 * p.step_len, if left { p.step_w / 2.0 } else { -p.step_w / 2.0 }], left }
        })
        .collect()
}

/// One stretch of the ZMP reference: linear p0 → p1 over `dur`, with the feet that carry the weight.
#[derive(Clone, Debug, PartialEq)]
pub struct Seg {
    pub t0: f64,
    pub dur: f64,
    pub p0: P2,
    pub p1: P2,
    /// Indices of the feet on the ground (one = single support).
    pub support: Vec<usize>,
    /// Swinging foot: (from index, to index), single support only.
    pub swing: Option<(usize, usize)>,
    /// DCM reference at the end of the segment (filled by the backward recursion).
    pub xi_end: P2,
}

impl Seg {
    fn slope(&self) -> P2 {
        let d = sub(self.p1, self.p0);
        [d[0] / self.dur, d[1] / self.dur]
    }
    pub fn zmp(&self, t: f64) -> P2 {
        lerp(self.p0, self.p1, ((t - self.t0) / self.dur).clamp(0.0, 1.0))
    }
    /// Closed-form DCM for a linear ZMP: ξ = p + ṗ/ω + e^(ω(t − T))(ξ_T − p_T − ṗ/ω).
    pub fn xi(&self, t: f64, w: f64) -> P2 {
        let (p, d) = (self.zmp(t), self.slope());
        let e = (w * (t - self.t0 - self.dur)).min(0.0).exp();
        [0, 1].map(|i| p[i] + d[i] / w + e * (self.xi_end[i] - self.p1[i] - d[i] / w))
    }
}

pub struct Plan {
    pub feet: Vec<Foot>,
    pub segs: Vec<Seg>,
    pub omega: f64,
    pub foot_l: f64,
    pub foot_w: f64,
}

/// Which foot (or both) carries the weight.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Phase {
    Left,
    Right,
    Double,
}

impl Plan {
    pub fn new(p: &Params, feet: Vec<Foot>) -> Plan {
        let w = p.omega();
        let n = feet.len() - 1;
        let c = |i: usize| feet[i].pos;
        let mid = |i: usize, j: usize| lerp(c(i), c(j), 0.5);
        let (tss, tds) = (p.t_ss, p.t_ds());
        let shift = tds.max(0.5 * tss);
        let mut raw: Vec<(f64, P2, P2, Vec<usize>, Option<(usize, usize)>)> = vec![
            (1.0, mid(0, 1), mid(0, 1), vec![0, 1], None),
            (shift, mid(0, 1), c(1), vec![0, 1], None),
        ];
        for k in 2..=n {
            raw.push((tss, c(k - 1), c(k - 1), vec![k - 1], Some((k - 2, k))));
            if k < n {
                raw.push((tds, c(k - 1), c(k), vec![k - 1, k], None));
            } else {
                raw.push((shift, c(k - 1), mid(k - 1, k), vec![k - 1, k], None));
            }
        }
        raw.push((1.0 + 4.0 / w, mid(n - 1, n), mid(n - 1, n), vec![n - 1, n], None));
        let mut t = 0.0;
        let mut segs: Vec<Seg> = raw
            .into_iter()
            .filter(|r| r.0 > 1e-9)
            .map(|(dur, p0, p1, support, swing)| {
                let s = Seg { t0: t, dur, p0, p1, support, swing, xi_end: p1 };
                t += dur;
                s
            })
            .collect();
        // Backward recursion: at rest at the end, each segment ends where the next one starts.
        let mut xi = segs.last().map(|s| s.p1).unwrap_or([0.0; 2]);
        for s in segs.iter_mut().rev() {
            s.xi_end = xi;
            xi = s.xi(s.t0, w);
        }
        Plan { feet, segs, omega: w, foot_l: p.foot_l, foot_w: p.foot_w }
    }

    pub fn duration(&self) -> f64 {
        self.segs.last().map(|s| s.t0 + s.dur).unwrap_or(0.0)
    }
    pub fn seg_at(&self, t: f64) -> usize {
        self.segs.iter().rposition(|s| s.t0 <= t).unwrap_or(0)
    }
    pub fn polygon(&self, seg: usize) -> Vec<P2> {
        let pts: Vec<P2> = self.segs[seg].support.iter().flat_map(|&i| rect(self.feet[i].pos, self.foot_l, self.foot_w)).collect();
        hull(&pts)
    }
    pub fn phase(&self, seg: usize) -> Phase {
        match self.segs[seg].support[..] {
            [i] if self.feet[i].left => Phase::Left,
            [_] => Phase::Right,
            _ => Phase::Double,
        }
    }
    /// Left and right foot: floor position and height at time t (swing foot on a smooth arc).
    pub fn feet_at(&self, t: f64, lift: f64) -> [(P2, f64); 2] {
        let s = &self.segs[self.seg_at(t)];
        let mut out = [([0.0; 2], 0.0); 2];
        for &i in &s.support {
            out[!self.feet[i].left as usize] = (self.feet[i].pos, 0.0);
        }
        if let Some((a, b)) = s.swing {
            let u = ((t - s.t0) / s.dur).clamp(0.0, 1.0);
            let e = 0.5 - 0.5 * (std::f64::consts::PI * u).cos();
            let h = lift * (std::f64::consts::PI * u).sin();
            out[!self.feet[b].left as usize] = (lerp(self.feet[a].pos, self.feet[b].pos, e), h);
        }
        out
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Push {
    pub t: f64,
    /// Velocity impulse magnitude (m/s).
    pub dv: f64,
    /// Direction (deg): 0 = forward (+x), 90 = left (+y).
    pub angle_deg: f64,
}

impl Push {
    pub fn dir(&self) -> P2 {
        let a = self.angle_deg.to_radians();
        [a.cos(), a.sin()]
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Sample {
    pub t: f64,
    pub com: P2,
    pub vel: P2,
    pub zmp: P2,
    pub zmp_ref: P2,
    pub cp: P2,
    pub cp_ref: P2,
    /// Signed distance of the ZMP to the support polygon edge (m).
    pub margin: f64,
    pub clamped: bool,
    pub seg: usize,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct PushReport {
    /// Capture point right after the push lies outside the support polygon.
    pub step_needed: bool,
    /// Signed distance of that capture point to the polygon (m).
    pub cp_margin: f64,
    /// ω × (CoM → polygon edge distance along the push direction), standing still on that polygon.
    pub dv_max: f64,
    /// Same bound measured from the pre-push capture point (accounts for current motion).
    pub dv_max_cp: f64,
    /// Time the controller gave up (capture point far outside the polygon).
    pub fell_at: Option<f64>,
}

pub struct Sim {
    pub plan: Plan,
    pub samples: Vec<Sample>,
    pub dt: f64,
    /// Minimum ZMP stability margin over the run (m).
    pub min_margin: f64,
    pub saturated: bool,
    pub push: Option<PushReport>,
}

/// Max recoverable Δv along `dir` from `from` inside `poly`: ω × distance to the edge.
pub fn max_recoverable_dv(poly: &[P2], from: P2, dir: P2, w: f64) -> f64 {
    w * geom::ray_exit(poly, from, dir)
}

/// RK4 step of x'' = ω²(x − p) with p held constant over dt.
pub fn lipm_step(x: f64, v: f64, p: f64, w: f64, dt: f64) -> (f64, f64) {
    let f = |x: f64, v: f64| (v, w * w * (x - p));
    let (k1x, k1v) = f(x, v);
    let (k2x, k2v) = f(x + 0.5 * dt * k1x, v + 0.5 * dt * k1v);
    let (k3x, k3v) = f(x + 0.5 * dt * k2x, v + 0.5 * dt * k2v);
    let (k4x, k4v) = f(x + dt * k3x, v + dt * k3v);
    (x + dt / 6.0 * (k1x + 2.0 * k2x + 2.0 * k3x + k4x), v + dt / 6.0 * (k1v + 2.0 * k2v + 2.0 * k3v + k4v))
}

/// Capture point this far outside the polygon (m) counts as a fall; the CoM is frozen there.
pub const FALL_DIST: f64 = 0.3;

pub fn simulate(p: &Params, feet: Vec<Foot>, push: Option<Push>) -> Sim {
    const DT: f64 = 0.001;
    const EVERY: usize = 5;
    let plan = Plan::new(p, feet);
    let w = plan.omega;
    let end = plan.duration();
    // Start at rest on the reference capture point (ξ = x when v = 0).
    let mut x = plan.segs[0].xi(0.0, w);
    let mut v = [0.0; 2];
    let mut samples = Vec::new();
    let (mut min_margin, mut saturated) = (f64::INFINITY, false);
    let mut report: Option<PushReport> = None;
    let (mut pushed, mut fallen) = (false, false);
    let mut seg = 0;
    let mut poly = plan.polygon(0);
    for i in 0..=(end / DT).round() as usize {
        let t = i as f64 * DT;
        while seg + 1 < plan.segs.len() && plan.segs[seg + 1].t0 <= t + 1e-12 {
            seg += 1;
            poly = plan.polygon(seg);
        }
        if let Some(pu) = push.filter(|pu| !pushed && t >= pu.t - 1e-12) {
            pushed = true;
            let d = pu.dir();
            let xi0 = capture_point(x, v, w);
            v = [v[0] + pu.dv * d[0], v[1] + pu.dv * d[1]];
            let xi = capture_point(x, v, w);
            let m = signed_dist(&poly, xi);
            report = Some(PushReport {
                step_needed: m < 0.0,
                cp_margin: m,
                dv_max: max_recoverable_dv(&poly, x, d, w),
                dv_max_cp: max_recoverable_dv(&poly, xi0, d, w),
                fell_at: None,
            });
        }
        let s = &plan.segs[seg];
        let (zr, xr) = (s.zmp(t), s.xi(t, w));
        let xi = capture_point(x, v, w);
        if !fallen && pushed && signed_dist(&poly, xi) < -FALL_DIST {
            fallen = true;
            // The polygon could not hold the capture point: counts as saturation.
            saturated = true;
            v = [0.0; 2];
            if let Some(r) = report.as_mut() {
                r.fell_at = Some(t);
            }
        }
        let cmd = [0, 1].map(|j| zr[j] + (1.0 + p.k) * (xi[j] - xr[j]));
        let zmp = if fallen { clamp(&poly, x) } else { clamp(&poly, cmd) };
        let clamped = !fallen && geom::len(sub(zmp, cmd)) > 1e-9;
        if !fallen {
            saturated |= clamped;
            min_margin = min_margin.min(signed_dist(&poly, zmp));
        }
        if i % EVERY == 0 {
            let margin = signed_dist(&poly, zmp);
            samples.push(Sample { t, com: x, vel: v, zmp, zmp_ref: zr, cp: capture_point(x, v, w), cp_ref: xr, margin, clamped, seg });
        }
        if !fallen {
            for j in 0..2 {
                (x[j], v[j]) = lipm_step(x[j], v[j], zmp[j], w, DT);
            }
        }
    }
    Sim { plan, samples, dt: DT * EVERY as f64, min_margin, saturated, push: report }
}

impl Sim {
    pub fn sample_at(&self, t: f64) -> &Sample {
        let i = ((t / self.dt).round().max(0.0) as usize).min(self.samples.len() - 1);
        &self.samples[i]
    }
}

#[cfg(test)]
mod tests {
    use super::geom::*;
    use super::*;

    fn close(a: f64, b: f64, tol: f64) {
        assert!((a - b).abs() < tol, "{a} vs {b}");
    }

    #[test]
    fn omega_and_time_constant() {
        let w = omega(9.81, 0.981);
        close(w, 10f64.sqrt(), 1e-12);
        close(w, 3.162, 1e-3);
        close(1.0 / w, 0.3162, 1e-4);
    }

    #[test]
    fn capture_point_of_moving_com() {
        let xi = capture_point([0.0, 0.0], [0.5, 0.0], 10f64.sqrt());
        close(xi[0], 0.158, 1e-3);
        close(xi[1], 0.0, 1e-12);
    }

    #[test]
    fn lipm_rk4_matches_cosh() {
        let w = 10f64.sqrt();
        let (mut x, mut v) = (0.01, 0.0);
        for _ in 0..500 {
            (x, v) = lipm_step(x, v, 0.0, w, 0.001);
        }
        close(x, 0.01 * (w * 0.5).cosh(), 1e-4);
        close(x, 0.01 * (w * 0.5).cosh(), 1e-9);
    }

    #[test]
    fn hull_of_two_feet() {
        let mut pts = rect([0.0, 0.1], 0.2, 0.1);
        pts.extend(rect([0.2, -0.1], 0.2, 0.1));
        let h = hull(&pts);
        assert_eq!(h.len(), 6, "{h:?}");
        // Shoelace area: two 0.02 m² feet plus the parallelogram joining them.
        let area: f64 = (0..h.len()).map(|i| h[i][0] * h[(i + 1) % 6][1] - h[(i + 1) % 6][0] * h[i][1]).sum::<f64>() / 2.0;
        close(area, 0.08, 1e-12);
        assert!(contains(&h, [0.1, 0.0]));
        assert!(!contains(&h, [0.25, 0.1]));
        // Overlapping / duplicate feet still give a clean rectangle.
        assert_eq!(hull(&[rect([0.0; 2], 0.2, 0.1), rect([0.0; 2], 0.2, 0.1)].concat()).len(), 4);
    }

    #[test]
    fn signed_distance_in_unit_square() {
        let sq = rect([0.5, 0.5], 1.0, 1.0);
        close(signed_dist(&sq, [0.5, 0.5]), 0.5, 1e-12);
        close(signed_dist(&sq, [0.2, 0.7]), 0.2, 1e-12);
        close(signed_dist(&sq, [0.9, 0.1]), 0.1, 1e-12);
        close(signed_dist(&sq, [1.3, 0.5]), -0.3, 1e-12);
        close(signed_dist(&sq, [1.3, 1.4]), -0.5, 1e-12);
        assert_eq!(clamp(&sq, [1.3, 1.4]), [1.0, 1.0]);
        assert_eq!(clamp(&sq, [0.3, 0.4]), [0.3, 0.4]);
    }

    #[test]
    fn nominal_gait_balanced_for_presets() {
        for p in [Params::ADULT, Params::KID, Params { ds_ratio: 0.0, ..Params::ADULT }] {
            let sim = simulate(&p, footsteps(&p), None);
            assert!(!sim.saturated, "{p:?}");
            assert!(sim.min_margin > 0.0, "{p:?} {}", sim.min_margin);
            assert!(sim.samples.iter().all(|s| s.margin >= -1e-12));
            let f = &sim.plan.feet;
            let goal = lerp(f[f.len() - 2].pos, f[f.len() - 1].pos, 0.5);
            let last = sim.samples.last().unwrap();
            assert!(len(sub(last.com, goal)) < 0.01, "{:?} vs {goal:?}", last.com);
            assert!(len(last.vel) < 0.02);
            // The CoM really walked forward by the planned distance.
            close(goal[0], (p.steps - 1) as f64 * p.step_len, 1e-12);
        }
    }

    #[test]
    fn max_dv_is_omega_times_edge_distance() {
        let foot = rect([0.0, 0.0], 0.2, 0.1);
        let w = 10f64.sqrt();
        close(max_recoverable_dv(&foot, [0.0, 0.0], [1.0, 0.0], w), w * 0.1, 1e-12);
        close(max_recoverable_dv(&foot, [0.0, 0.0], [0.0, -1.0], w), w * 0.05, 1e-12);
        close(max_recoverable_dv(&foot, [0.02, 0.0], [-1.0, 0.0], w), w * 0.12, 1e-12);
        // Diagonal: hits the side edge first at 0.05 / sin 45°.
        let d = std::f64::consts::FRAC_1_SQRT_2;
        close(max_recoverable_dv(&foot, [0.0, 0.0], [d, d], w), w * 0.05 / d, 1e-12);
        assert_eq!(max_recoverable_dv(&foot, [0.5, 0.0], [1.0, 0.0], w), 0.0, "outside");
    }

    #[test]
    fn mid_single_support_capture_point_inside_stance_foot() {
        for p in [Params::ADULT, Params::KID] {
            let sim = simulate(&p, footsteps(&p), None);
            let s = sim.plan.segs.iter().position(|s| s.swing.is_some()).unwrap();
            let seg = &sim.plan.segs[s];
            let smp = sim.sample_at(seg.t0 + seg.dur / 2.0);
            assert_eq!(sim.plan.phase(smp.seg), Phase::Right);
            assert!(signed_dist(&sim.plan.polygon(s), smp.cp) > 0.0, "{p:?}");
        }
    }

    #[test]
    fn push_recovery_small_vs_big() {
        let p = Params::ADULT;
        // Push while standing in the initial double support.
        let small = simulate(&p, footsteps(&p), Some(Push { t: 0.3, dv: 0.05, angle_deg: 0.0 })).push.unwrap();
        assert!(!small.step_needed && small.fell_at.is_none(), "{small:?}");
        assert!(small.dv_max > 0.05);
        let big = simulate(&p, footsteps(&p), Some(Push { t: 0.3, dv: 1.5, angle_deg: 90.0 })).push.unwrap();
        assert!(big.step_needed && big.cp_margin < 0.0, "{big:?}");
        let sim = simulate(&p, footsteps(&p), Some(Push { t: 0.3, dv: 1.5, angle_deg: 90.0 }));
        assert!(sim.saturated && sim.push.unwrap().fell_at.is_some());
        assert!(big.dv_max < 1.5);
        // Nearly at rest, both bounds agree.
        close(big.dv_max, big.dv_max_cp, 0.02);
    }
}
