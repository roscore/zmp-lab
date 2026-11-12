//! 3D scene: floor, footsteps, support polygon, stick humanoid, ZMP / capture-point markers, trails.
//! Floor frame (x forward, y left, z up) maps to GL as (x, z, −y).
use crate::gl::{self, Mesh, Paint, Renderer};
use crate::math::{self, Orbit, OrbitCam, M4, V};
use web_sys::HtmlCanvasElement;
use zl_core::geom::{self, P2};
use zl_core::{Params, Sim};

const FOV: f32 = 0.8;

fn g(p: P2, h: f64) -> V {
    [p[0] as f32, h as f32, -p[1] as f32]
}

/// Which overlays are drawn.
#[derive(Clone, Copy)]
pub struct Show {
    pub trails: bool,
    pub poly: bool,
    pub cp: bool,
    pub body: bool,
}

#[derive(Clone, Copy, PartialEq)]
pub enum Preset {
    Persp,
    Top,
    Side,
    Front,
}

enum Drag {
    Orbit,
    Pan,
    Pinch(f64, [f64; 2]),
    Foot(usize, P2),
}

struct Meshes {
    cube: Mesh,
    cyl: Mesh,
    ball: Mesh,
    plane: Mesh,
    disc: Mesh,
    ring: Mesh,
    vline: Mesh,
}

pub struct View {
    pub r: Renderer,
    canvas: HtmlCanvasElement,
    pub cam: OrbitCam,
    home: Orbit,
    m: Meshes,
    trails: Vec<Mesh>,
    pointers: Vec<(i32, f64, f64)>,
    drag: Option<Drag>,
    pub hover: Option<usize>,
}

fn basis_seg(a: V, b: V, r: f32) -> M4 {
    // Cylinder (y-axis, height 1, radius 0.5) stretched from a to b with radius r.
    let d = math::sub(b, a);
    let len = math::dot(d, d).sqrt().max(1e-6);
    let y = math::scale(d, 1.0 / len);
    let x = math::norm(math::cross(y, if y[1].abs() < 0.9 { [0., 1., 0.] } else { [1., 0., 0.] }));
    let z = math::cross(x, y);
    math::basis(x, y, z, [2.0 * r, len, 2.0 * r], math::scale(math::add(a, b), 0.5))
}

fn at(t: V, s: V) -> M4 {
    math::basis([1., 0., 0.], [0., 1., 0.], [0., 0., 1.], s, t)
}

impl View {
    pub fn new(canvas: HtmlCanvasElement, dark: bool) -> Option<View> {
        let mut r = Renderer::new(&canvas)?;
        r.theme = if dark { gl::DARK } else { gl::LIGHT };
        let m = Meshes {
            cube: r.mesh(&gl::cube())?,
            cyl: r.mesh(&gl::cylinder(20))?,
            ball: r.mesh(&gl::sphere(16))?,
            plane: r.mesh(&gl::plane())?,
            disc: r.mesh(&gl::ring(0.0, 1.0, 32))?,
            ring: r.mesh(&gl::ring(0.72, 1.0, 40))?,
            vline: r.mesh(&gl::lines(&[[0., 0., 0.], [0., 1., 0.]]))?,
        };
        let home = Orbit { target: [0.0; 3], yaw: 0.9, pitch: 0.42, dist: 3.0 };
        Some(View { r, canvas, cam: OrbitCam::new(home), home, m, trails: vec![], pointers: vec![], drag: None, hover: None })
    }

    /// Frame the whole walk; `jump` also moves the camera there (otherwise only double-click does).
    pub fn frame(&mut self, sim: &Sim, p: &Params, jump: bool) {
        let xs = sim.plan.feet.iter().map(|f| f.pos[0]);
        let (lo, hi) = xs.fold((f64::MAX, f64::MIN), |(a, b), x| (a.min(x), b.max(x)));
        let span = ((hi - lo) as f32).max(p.step_w as f32 * 4.0).max(p.z_c as f32 * 1.5);
        self.home.target = [((lo + hi) / 2.0) as f32, (p.z_c * 0.45) as f32, 0.0];
        self.home.dist = span * 2.4;
        self.r.cell = if p.z_c < 0.5 { 0.1 } else { 0.25 };
        self.r.fog = [self.home.dist * 4.0, self.home.dist * 14.0];
        if jump {
            self.cam.set_goal(self.home);
        }
    }

    pub fn preset(&mut self, v: Preset) {
        let (yaw, pitch) = match v {
            Preset::Persp => (0.9, 0.42),
            Preset::Top => (0.0, math::PITCH_MAX),
            Preset::Side => (0.0, 0.05),
            Preset::Front => (std::f32::consts::FRAC_PI_2, 0.05),
        };
        self.cam.set_goal(Orbit { yaw, pitch, ..self.home });
    }

    pub fn reset(&mut self) {
        self.cam.set_goal(self.home);
    }

    /// Rebuild the trail line meshes after a new simulation.
    pub fn set_trails(&mut self, sim: &Sim) {
        for m in self.trails.drain(..) {
            self.r.free(m);
        }
        let pick: [fn(&zl_core::Sample) -> P2; 3] = [|s| s.com, |s| s.zmp, |s| s.cp];
        for (k, f) in pick.into_iter().enumerate() {
            let h = 0.008 + 0.002 * k as f64;
            let pts: Vec<V> = sim.samples.windows(2).flat_map(|w| [g(f(&w[0]), h), g(f(&w[1]), h)]).collect();
            if let Some(m) = self.r.mesh(&gl::lines(&pts)) {
                self.trails.push(m);
            }
        }
    }

    fn size(&self) -> (f64, f64) {
        (self.canvas.client_width().max(1) as f64, self.canvas.client_height().max(1) as f64)
    }

    fn inv_vp(&self) -> M4 {
        let (w, h) = self.size();
        let c = self.cam.cur;
        let vp = math::mul(&math::perspective(FOV, (w / h) as f32, 0.01, 400.0), &math::look_at(c.eye(), c.target, [0., 1., 0.]));
        math::invert(&vp).unwrap_or(math::IDENTITY)
    }

    /// Floor point under the canvas position (CSS px), in floor coordinates.
    fn floor_hit(&self, cx: f64, cy: f64) -> Option<P2> {
        let (w, h) = self.size();
        let (o, d) = math::ndc_ray(&self.inv_vp(), (2.0 * cx / w - 1.0) as f32, (1.0 - 2.0 * cy / h) as f32);
        math::ray_plane(o, d, [0.0; 3], [0., 1., 0.]).map(|p| [p[0] as f64, -p[2] as f64])
    }

    /// Draggable footstep (any but the first) under the cursor.
    fn pick(&self, sim: &Sim, cx: f64, cy: f64) -> Option<usize> {
        let p = self.floor_hit(cx, cy)?;
        let pl = &sim.plan;
        (1..pl.feet.len())
            .filter(|&i| geom::contains(&geom::rect(pl.feet[i].pos, pl.foot_l * 1.2, pl.foot_w * 1.3), p))
            .min_by(|&a, &b| geom::len(geom::sub(p, pl.feet[a].pos)).total_cmp(&geom::len(geom::sub(p, pl.feet[b].pos))))
    }

    fn centroid(&self) -> (f64, [f64; 2]) {
        let [a, b] = [self.pointers[0], self.pointers[1]];
        ((a.1 - b.1).hypot(a.2 - b.2), [(a.1 + b.1) / 2.0, (a.2 + b.2) / 2.0])
    }

    pub fn down(&mut self, sim: &Sim, id: i32, button: i16, cx: f64, cy: f64) {
        self.pointers.retain(|p| p.0 != id);
        self.pointers.push((id, cx, cy));
        self.cam.held = true;
        self.drag = if self.pointers.len() >= 2 {
            let (d, c) = self.centroid();
            Some(Drag::Pinch(d, c))
        } else if button == 2 {
            Some(Drag::Pan)
        } else if let Some(i) = self.pick(sim, cx, cy) {
            let hit = self.floor_hit(cx, cy).unwrap_or(sim.plan.feet[i].pos);
            Some(Drag::Foot(i, geom::sub(hit, sim.plan.feet[i].pos)))
        } else {
            Some(Drag::Orbit)
        };
    }

    fn pan_px(&mut self, dx: f64, dy: f64) {
        let c = self.cam.cur;
        let f = math::norm(math::sub(c.target, c.eye()));
        let right = math::norm(math::cross(f, [0., 1., 0.]));
        let up = math::cross(right, f);
        let k = c.dist * 2.0 * (FOV / 2.0).tan() / self.size().1 as f32;
        self.cam.pan(math::add(math::scale(right, -dx as f32 * k), math::scale(up, dy as f32 * k)));
    }

    /// Pointer moved; returns a footstep edit (index, new floor position) while dragging a foot.
    pub fn moved(&mut self, sim: &Sim, id: i32, cx: f64, cy: f64) -> Option<(usize, P2)> {
        let Some(slot) = self.pointers.iter_mut().find(|p| p.0 == id) else {
            self.hover = self.pick(sim, cx, cy);
            return None;
        };
        let (dx, dy) = (cx - slot.1, cy - slot.2);
        (slot.1, slot.2) = (cx, cy);
        match self.drag {
            Some(Drag::Orbit) => self.cam.rotate(-dx as f32 * 0.008, dy as f32 * 0.008),
            Some(Drag::Pan) => self.pan_px(dx, dy),
            Some(Drag::Pinch(d0, c0)) if self.pointers.len() >= 2 => {
                let (d, c) = self.centroid();
                self.cam.zoom((d0 / d.max(1.0)) as f32);
                self.pan_px(c[0] - c0[0], c[1] - c0[1]);
                self.drag = Some(Drag::Pinch(d, c));
            }
            Some(Drag::Foot(i, off)) => {
                let p = self.floor_hit(cx, cy)?;
                return Some((i, geom::sub(p, off)));
            }
            _ => {}
        }
        None
    }

    pub fn up(&mut self, id: i32) {
        self.pointers.retain(|p| p.0 != id);
        if self.pointers.is_empty() {
            self.drag = None;
            self.cam.held = false;
        } else {
            self.drag = None;
        }
    }

    pub fn cursor(&self) -> &'static str {
        match self.drag {
            Some(Drag::Foot(..)) => "grabbing",
            _ if self.hover.is_some() => "grab",
            _ => "default",
        }
    }

    pub fn dragging_foot(&self) -> Option<usize> {
        match self.drag {
            Some(Drag::Foot(i, _)) => Some(i),
            _ => None,
        }
    }

    pub fn render(&mut self, sim: &Sim, p: &Params, t: f64, show: Show, dt: f32) {
        self.cam.step(dt);
        let dpr = web_sys::window().map(|w| w.device_pixel_ratio()).unwrap_or(1.0);
        let (w, h) = self.size();
        let (pw, ph) = ((w * dpr).round() as u32, (h * dpr).round() as u32);
        if self.canvas.width() != pw || self.canvas.height() != ph {
            self.canvas.set_width(pw);
            self.canvas.set_height(ph);
        }
        let c = self.cam.cur;
        let eye = c.eye();
        let view = math::look_at(eye, c.target, [0., 1., 0.]);
        let proj = math::perspective(FOV, (w / h) as f32, 0.01, 400.0);
        let r = &self.r;
        r.begin(&view, &proj, eye, [0, 0, pw as i32, ph as i32]);

        let s = sim.sample_at(t);
        let pl = &sim.plan;
        let seg = &pl.segs[s.seg];
        let (zc, fl, fw) = (p.z_c, p.foot_l, p.foot_w);
        let thick = (0.03 * zc).max(0.006);
        let feet = pl.feet_at(t, p.lift());
        let blue = [0.25, 0.5, 0.95];
        let orange = [0.98, 0.55, 0.15];
        let col = |left: bool, a: f32| {
            let c = if left { blue } else { orange };
            [c[0], c[1], c[2], a]
        };

        // Opaque parts (with shadows): physical feet and the stick humanoid.
        let mut solid: Vec<(&Mesh, M4, Paint)> = vec![];
        for (k, &(pos, hgt)) in feet.iter().enumerate() {
            let left = k == 0;
            let stance = seg.support.iter().any(|&i| pl.feet[i].left == left);
            let mut paint = Paint::lit(col(left, 1.0));
            paint.glow = if stance { 0.3 } else { 0.0 };
            solid.push((&self.m.cube, at(g(pos, hgt + thick / 2.0), [fl as f32, thick as f32, fw as f32]), paint));
        }
        if show.body {
            let body = Paint::lit([0.78, 0.80, 0.84, 1.0]);
            let joint = Paint::lit([0.35, 0.38, 0.45, 1.0]);
            let pel = g(s.com, zc);
            let hip_w = (p.step_w * 0.4).max(0.06 * zc);
            let lr = (0.045 * zc) as f32;
            solid.push((&self.m.cube, at(pel, [(0.14 * zc) as f32, (0.1 * zc) as f32, (2.0 * hip_w + 0.08 * zc) as f32]), joint));
            for (k, &(pos, hgt)) in feet.iter().enumerate() {
                let side = if k == 0 { hip_w } else { -hip_w };
                let hip = g([s.com[0], s.com[1] + side], zc - 0.05 * zc);
                let ankle = g(pos, hgt + thick);
                solid.push((&self.m.cyl, basis_seg(hip, ankle, lr), body));
                for j in [hip, ankle] {
                    solid.push((&self.m.ball, at(j, [lr * 1.15; 3]), joint));
                }
            }
            let neck = g(s.com, zc * 1.45);
            solid.push((&self.m.cyl, basis_seg(pel, neck, (0.08 * zc) as f32), body));
            solid.push((&self.m.ball, at(g(s.com, zc * 1.6), [(0.1 * zc) as f32; 3]), body));
        }
        for (m, model, _) in &solid {
            r.shadow(m, model);
        }
        for (m, model, paint) in &solid {
            r.draw(m, model, *paint);
        }

        // Ground overlays (translucent, drawn last).
        for (i, f) in pl.feet.iter().enumerate() {
            let mut paint = Paint::flat(col(f.left, 0.28));
            if Some(i) == self.hover || Some(i) == self.dragging_foot() {
                paint.color[3] = 0.6;
                paint.glow = 0.35;
            }
            r.draw(&self.m.plane, &at(g(f.pos, 0.002), [fl as f32, 1.0, fw as f32]), paint);
        }
        if show.poly {
            let poly: Vec<V> = pl.polygon(s.seg).iter().map(|&q| g(q, 0.004)).collect();
            for geo in [gl::fan(&poly), gl::line_loop(&poly)] {
                let lines = geo.lines;
                if let Some(m) = r.mesh(&geo) {
                    r.draw(&m, &math::IDENTITY, Paint::flat(if lines { [0.2, 0.85, 0.4, 0.9] } else { [0.2, 0.85, 0.4, 0.3] }));
                    r.free(m);
                }
            }
        }
        if show.trails {
            let n = (2 * (t / sim.dt).round() as i32).max(0);
            for (k, (m, c)) in self.trails.iter().zip([[0.9, 0.9, 0.95, 0.9], [1.0, 0.3, 0.3, 0.9], [1.0, 0.85, 0.2, 0.9]]).enumerate() {
                if k == 2 && !show.cp {
                    continue;
                }
                r.draw_first(m, &math::IDENTITY, Paint::flat(c), n);
            }
        }
        let mk = (0.04 * zc).max(0.012) as f32;
        r.draw(&self.m.vline, &at(g(s.com, 0.0), [1.0, zc as f32, 1.0]), Paint::flat([0.95, 0.95, 1.0, 0.8]));
        r.draw(&self.m.ball, &at(g(s.com, zc), [mk * 0.7; 3]), Paint::lit([1.0, 1.0, 1.0, 1.0]));
        r.draw(&self.m.disc, &at(g(s.com, 0.006), [mk * 0.5; 3]), Paint::flat([0.95, 0.95, 1.0, 0.9]));
        let zcol = if s.clamped { [0.95, 0.2, 0.2, 0.95] } else { [0.2, 0.85, 0.35, 0.95] };
        r.draw(&self.m.disc, &at(g(s.zmp, 0.007), [mk; 3]), Paint::flat(zcol));
        if show.cp {
            r.draw(&self.m.ring, &at(g(s.cp, 0.009), [mk * 1.4; 3]), Paint::flat([1.0, 0.85, 0.2, 1.0]));
        }
    }
}
