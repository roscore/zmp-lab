//! ZMP & capture-point walking lab: DOM wiring, i18n, playback loop.
pub mod gl;
pub mod math;
mod plot;
mod view3d;

use std::cell::RefCell;
use std::rc::Rc;
use view3d::{Preset, Show, View};
use wasm_bindgen::prelude::*;
use web_sys::{Document, Element, HtmlCanvasElement, HtmlInputElement, HtmlSelectElement, PointerEvent};
use zl_core::{footsteps, geom, simulate, Foot, Params, Phase, Push, Sim};

/// (key, Korean, English)
const TEXT: &[(&str, &str, &str)] = &[
    ("title", "ZMP · 캡처 포인트 보행 랩", "ZMP & capture-point walking lab"),
    ("subtitle", "선형 역진자(LIPM) 기반 휴머노이드 보행 균형 계산기", "Humanoid gait balance with the Linear Inverted Pendulum Model"),
    ("lang", "English", "한국어"),
    ("presets", "프리셋 (일반적인 값, 수정 가능)", "Presets (typical values, editable)"),
    ("adult", "성인 크기 휴머노이드 (일반)", "Adult-size humanoid (typical)"),
    ("kid", "소형 키드 크기 휴머노이드 (일반)", "Small kid-size humanoid (typical)"),
    ("gait", "보행 파라미터", "Gait parameters"),
    ("zc", "CoM 높이 z_c (m)", "CoM height z_c (m)"),
    ("g", "중력 g (m/s²)", "Gravity g (m/s²)"),
    ("fl", "발 길이 (m)", "Foot length (m)"),
    ("fw", "발 폭 (m)", "Foot width (m)"),
    ("sl", "보폭 (m)", "Step length (m)"),
    ("sw", "양발 간격 (m)", "Step width (m)"),
    ("tss", "단일 지지 시간 T_ss (s)", "Single support T_ss (s)"),
    ("ds", "이중 지지 비율 (0–0.4)", "Double-support ratio (0–0.4)"),
    ("n", "걸음 수 (2–10)", "Number of steps (2–10)"),
    ("k", "DCM 게인 k", "DCM gain k"),
    ("push_h", "외란 (밀기)", "Push recovery"),
    ("pdv", "Δv (m/s)", "Δv (m/s)"),
    ("pang", "방향 (°, 0=앞, 90=왼쪽)", "Direction (°, 0=forward, 90=left)"),
    ("pt", "시각 (s)", "Time (s)"),
    ("push", "밀기!", "Push!"),
    ("clear", "외란 해제", "Clear push"),
    ("results", "결과", "Results"),
    ("trails", "궤적", "Trails"),
    ("poly", "지지 다각형", "Support polygon"),
    ("cp", "캡처 포인트", "Capture point"),
    ("body", "휴머노이드", "Humanoid"),
    ("persp", "원근", "Perspective"),
    ("top", "위", "Top"),
    ("side", "옆", "Side"),
    ("front", "앞", "Front"),
    ("play", "재생", "Play"),
    ("pause", "일시정지", "Pause"),
    ("speed", "속도", "Speed"),
    ("hint", "드래그: 회전 · 우클릭/두 손가락: 이동 · 휠/핀치: 확대 · 더블클릭: 초기화 · 발자국을 끌어서 위치 수정", "Drag: orbit · right-drag / two fingers: pan · wheel / pinch: zoom · double-click: reset · drag a footstep to move it"),
    ("nogl", "WebGL2를 사용할 수 없어 3D 뷰를 표시할 수 없습니다. 아래 2D 그래프는 계속 동작합니다.", "WebGL2 is unavailable, so the 3D view is off. The 2D plots below still work."),
    ("disclaimer", "교육용 추정치입니다 — 검증 없이 실제 로봇 제어기로 쓰지 마세요. 모든 계산은 브라우저 안에서만 수행됩니다. MIT 라이선스.", "Educational estimates — not a controller for a real robot without validation. Everything runs locally in your browser. MIT license."),
    ("plotx", "전후 x(t) (m)", "Forward x(t) (m)"),
    ("ploty", "좌우 y(t) (m)", "Lateral y(t) (m)"),
    ("com", "CoM", "CoM"),
    ("zmp", "ZMP", "ZMP"),
    ("zref", "ZMP 기준", "ZMP ref"),
    ("cpn", "캡처 포인트", "Capture pt"),
    ("phase", "지지", "phase"),
    ("ph_l", "왼발", "L"),
    ("ph_r", "오른발", "R"),
    ("ph_d", "양발", "double"),
    ("margin", "ZMP 여유", "ZMP margin"),
    ("clamped", "ZMP 포화(클램프)", "ZMP clamped"),
    ("omega", "ω", "ω"),
    ("tc", "시간 상수 1/ω", "Time constant 1/ω"),
    ("minm", "최소 안정 여유", "Minimum stability margin"),
    ("sat", "ZMP 포화 발생", "ZMP ever saturated"),
    ("yes", "예", "yes"),
    ("no", "아니오", "no"),
    ("dur", "총 시간", "Duration"),
    ("nopush", "밀기 없음 — Δv와 시각을 정하고 '밀기!'를 누르세요.", "No push — set Δv and time, then press Push!"),
    ("step", "⚠️ 캡처 포인트가 지지 다각형 밖 — 발을 내디뎌야 함", "⚠️ Capture point leaves the support polygon — step needed"),
    ("nostep", "캡처 포인트가 지지 다각형 안 — 발 내딛기 없이 회복 가능", "Capture point stays inside — recoverable without a step"),
    ("cpm", "밀기 직후 캡처 포인트 여유", "Capture-point margin right after push"),
    ("dvmax", "최대 회복 가능 Δv = ω × (CoM→가장자리 거리)", "Max recoverable Δv = ω × (CoM→edge distance)"),
    ("dvmaxcp", "현재 속도를 고려한 값 (캡처 포인트 기준)", "Accounting for current motion (from capture point)"),
    ("fell", "제어 한계 초과(넘어짐) 시각", "Controller gave up (fall) at"),
    ("drag", "발자국을 끌어서 수정됨", "footsteps edited by dragging"),
];

struct App {
    doc: Document,
    p: Params,
    feet: Vec<Foot>,
    push: Option<Push>,
    sim: Sim,
    t: f64,
    playing: bool,
    speed: f64,
    ko: bool,
    show: Show,
    dark: bool,
    edited: bool,
    view: Option<View>,
    last_ms: f64,
}

impl App {
    fn tr(&self, key: &str) -> &'static str {
        TEXT.iter().find(|e| e.0 == key).map(|e| if self.ko { e.1 } else { e.2 }).unwrap_or("?")
    }
    fn el(&self, id: &str) -> Element {
        self.doc.get_element_by_id(id).expect(id)
    }
    fn input(&self, id: &str) -> HtmlInputElement {
        self.el(id).unchecked_into()
    }
    fn num(&self, id: &str, lo: f64, hi: f64, old: f64) -> f64 {
        self.input(id).value().trim().parse::<f64>().ok().filter(|v| v.is_finite()).map(|v| v.clamp(lo, hi)).unwrap_or(old)
    }

    fn write_inputs(&self) {
        let p = &self.p;
        for (id, v) in [("zc", p.z_c), ("g", p.g), ("fl", p.foot_l), ("fw", p.foot_w), ("sl", p.step_len), ("sw", p.step_w), ("tss", p.t_ss), ("ds", p.ds_ratio), ("n", p.steps as f64), ("k", p.k)] {
            self.input(id).set_value(&format!("{}", (v * 1e4).round() / 1e4));
        }
    }

    fn read_inputs(&self) -> Params {
        let o = self.p;
        Params {
            z_c: self.num("zc", 0.05, 3.0, o.z_c),
            g: self.num("g", 0.1, 30.0, o.g),
            foot_l: self.num("fl", 0.01, 1.0, o.foot_l),
            foot_w: self.num("fw", 0.01, 1.0, o.foot_w),
            step_len: self.num("sl", -0.5, 1.0, o.step_len),
            step_w: self.num("sw", 0.0, 1.0, o.step_w),
            t_ss: self.num("tss", 0.1, 3.0, o.t_ss),
            ds_ratio: self.num("ds", 0.0, 0.4, o.ds_ratio),
            steps: self.num("n", 2.0, 10.0, o.steps as f64).round() as usize,
            k: self.num("k", 0.1, 10.0, o.k),
        }
    }

    fn resim(&mut self, reframe: bool) {
        self.sim = simulate(&self.p, self.feet.clone(), self.push);
        self.t = self.t.min(self.sim.plan.duration());
        let el = self.input("time");
        el.set_max(&format!("{}", self.sim.plan.duration()));
        if let Some(v) = self.view.as_mut() {
            v.set_trails(&self.sim);
            v.frame(&self.sim, &self.p, reframe);
        }
        self.results();
    }

    fn set_params(&mut self, p: Params, preset: bool) {
        self.p = p;
        self.feet = footsteps(&p);
        self.edited = false;
        if preset {
            self.write_inputs();
            // Default push: middle of the first single support, where the nominal capture point is inside.
            let plan = zl_core::Plan::new(&p, self.feet.clone());
            if let Some(s) = plan.segs.iter().find(|s| s.swing.is_some()) {
                self.input("pt").set_value(&format!("{:.2}", s.t0 + s.dur / 2.0));
            }
        }
        self.resim(true);
    }

    fn i18n(&self) {
        let list = self.doc.query_selector_all("[data-i18n]").expect("selector");
        for i in 0..list.length() {
            if let Some(e) = list.item(i).and_then(|n| n.dyn_into::<Element>().ok()) {
                e.set_text_content(Some(self.tr(&e.get_attribute("data-i18n").unwrap_or_default())));
            }
        }
        self.doc.document_element().map(|h| h.set_attribute("lang", if self.ko { "ko" } else { "en" }));
        self.el("playbtn").set_text_content(Some(self.tr(if self.playing { "pause" } else { "play" })));
        self.results();
    }

    fn results(&self) {
        let s = &self.sim;
        let w = s.plan.omega;
        let yn = |b: bool| self.tr(if b { "yes" } else { "no" });
        let row = |k: &str, v: String| format!("<tr><th>{}</th><td>{v}</td></tr>", self.tr(k));
        let mut h = String::from("<table>");
        h += &row("omega", format!("{w:.3} rad/s"));
        h += &row("tc", format!("{:.3} s", 1.0 / w));
        h += &row("minm", format!("{:.1} mm", s.min_margin * 1e3));
        h += &row("sat", format!("<span class=\"{}\">{}</span>", if s.saturated { "bad" } else { "good" }, yn(s.saturated)));
        h += &row("dur", format!("{:.2} s", s.plan.duration()));
        h += "</table>";
        if self.edited {
            h += &format!("<p class=\"note\">({})</p>", self.tr("drag"));
        }
        h += &match s.push {
            None => format!("<p class=\"note\">{}</p>", self.tr("nopush")),
            Some(r) => {
                let mut x = format!("<p class=\"{}\">{}</p><table>", if r.step_needed { "bad" } else { "good" }, self.tr(if r.step_needed { "step" } else { "nostep" }));
                x += &row("cpm", format!("{:.1} mm", r.cp_margin * 1e3));
                x += &row("dvmax", format!("{:.3} m/s", r.dv_max));
                x += &row("dvmaxcp", format!("{:.3} m/s", r.dv_max_cp));
                if let Some(f) = r.fell_at {
                    x += &row("fell", format!("{f:.2} s"));
                }
                x + "</table>"
            }
        };
        self.el("results").set_inner_html(&h);
    }

    fn frame(&mut self, now: f64) {
        let dt = if self.last_ms > 0.0 { ((now - self.last_ms) / 1000.0).clamp(0.0, 0.1) } else { 0.0 };
        self.last_ms = now;
        let end = self.sim.plan.duration();
        if self.playing {
            self.t += dt * self.speed;
            if self.t > end {
                self.t = 0.0;
            }
            self.input("time").set_value(&format!("{}", self.t));
        }
        let s = *self.sim.sample_at(self.t);
        let phase = match self.sim.plan.phase(s.seg) {
            Phase::Left => "ph_l",
            Phase::Right => "ph_r",
            Phase::Double => "ph_d",
        };
        let gap = geom::len(geom::sub(s.cp, s.zmp)) * 1e3;
        let mut hud = format!(
            "t = {:.2} s · {}: {} · {}: {:.1} mm · |ξ − ZMP| = {:.1} mm",
            self.t,
            self.tr("phase"),
            self.tr(phase),
            self.tr("margin"),
            s.margin * 1e3,
            gap
        );
        if s.clamped {
            hud += &format!(" · <b class=\"bad\">{}</b>", self.tr("clamped"));
        }
        self.el("hud").set_inner_html(&hud);
        self.el("tlabel").set_text_content(Some(&format!("{:.2} / {:.2} s", self.t, end)));
        if let Some(v) = self.view.as_mut() {
            v.render(&self.sim, &self.p, self.t, self.show, dt as f32);
        }
        self.plots();
    }

    fn plots(&self) {
        let ss = &self.sim.samples;
        for (id, axis) in [("plotx", 0), ("ploty", 1)] {
            let pick = |f: fn(&zl_core::Sample) -> [f64; 2]| ss.iter().map(|s| f(s)[axis]).collect::<Vec<f64>>();
            let series = [
                plot::Series { name: self.tr("zref"), color: "#9aa0aa", dashed: true, ys: pick(|s| s.zmp_ref) },
                plot::Series { name: self.tr("com"), color: "#4a8cf0", dashed: false, ys: pick(|s| s.com) },
                plot::Series { name: self.tr("zmp"), color: "#e5484d", dashed: false, ys: pick(|s| s.zmp) },
                plot::Series { name: self.tr("cpn"), color: "#e0a800", dashed: false, ys: pick(|s| s.cp) },
            ];
            let c: HtmlCanvasElement = self.el(id).unchecked_into();
            plot::draw(&c, self.tr(id), self.sim.dt, &series, self.t, self.dark);
        }
    }
}

type Shared = Rc<RefCell<App>>;

fn on<E: wasm_bindgen::convert::FromWasmAbi + 'static>(target: &Element, ev: &str, f: impl FnMut(E) + 'static) {
    let c = Closure::<dyn FnMut(E)>::new(f);
    let _ = target.add_event_listener_with_callback(ev, c.as_ref().unchecked_ref());
    c.forget();
}

fn click(app: &Shared, id: &str, f: impl Fn(&mut App) + 'static) {
    let a = app.clone();
    let el = app.borrow().el(id);
    on(&el, "click", move |_: web_sys::Event| f(&mut a.borrow_mut()));
}

#[wasm_bindgen(start)]
pub fn start() {
    let win = web_sys::window().expect("window");
    let doc = win.document().expect("document");
    let dark = win.match_media("(prefers-color-scheme: dark)").ok().flatten().map(|m| m.matches()).unwrap_or(false);
    let stored = win.local_storage().ok().flatten().and_then(|s| s.get_item("zmp-lab-lang").ok().flatten());
    let ko = match stored.as_deref() {
        Some(l) => l == "ko",
        None => win.navigator().language().unwrap_or_default().to_lowercase().starts_with("ko"),
    };
    let canvas: HtmlCanvasElement = doc.get_element_by_id("gl").expect("gl").unchecked_into();
    let view = View::new(canvas.clone(), dark);
    if view.is_none() {
        let _ = doc.get_element_by_id("nogl").map(|e| e.remove_attribute("hidden"));
        let _ = canvas.set_attribute("hidden", "");
    }
    let p = Params::ADULT;
    let app = Rc::new(RefCell::new(App {
        doc: doc.clone(),
        p,
        feet: footsteps(&p),
        push: None,
        sim: simulate(&p, footsteps(&p), None),
        t: 0.0,
        playing: true,
        speed: 0.25,
        ko,
        show: Show { trails: true, poly: true, cp: true, body: true },
        dark,
        edited: false,
        view,
        last_ms: 0.0,
    }));
    {
        let mut a = app.borrow_mut();
        a.set_params(p, true);
        a.i18n();
    }

    click(&app, "lang", |a| {
        a.ko = !a.ko;
        if let Some(s) = web_sys::window().and_then(|w| w.local_storage().ok().flatten()) {
            let _ = s.set_item("zmp-lab-lang", if a.ko { "ko" } else { "en" });
        }
        a.i18n();
    });
    click(&app, "p_adult", |a| a.set_params(Params::ADULT, true));
    click(&app, "p_kid", |a| a.set_params(Params::KID, true));
    for id in ["zc", "g", "fl", "fw", "sl", "sw", "tss", "ds", "n", "k"] {
        let a = app.clone();
        on(&app.borrow().el(id), "input", move |_: web_sys::Event| {
            let mut a = a.borrow_mut();
            let p = a.read_inputs();
            if p != a.p {
                a.set_params(p, false);
            }
        });
    }
    click(&app, "push", |a| {
        let push = Push { t: a.num("pt", 0.0, 60.0, 1.0), dv: a.num("pdv", 0.0, 10.0, 0.2), angle_deg: a.num("pang", -360.0, 360.0, 90.0) };
        a.push = Some(push);
        a.resim(false);
        a.t = (push.t - 0.4).max(0.0);
        a.playing = true;
        a.el("playbtn").set_text_content(Some(a.tr("pause")));
    });
    click(&app, "clear", |a| {
        a.push = None;
        a.resim(false);
    });
    click(&app, "playbtn", |a| {
        a.playing = !a.playing;
        a.el("playbtn").set_text_content(Some(a.tr(if a.playing { "pause" } else { "play" })));
    });
    {
        let a = app.clone();
        on(&app.borrow().el("speed"), "change", move |e: web_sys::Event| {
            let s: HtmlSelectElement = e.target().expect("target").unchecked_into();
            a.borrow_mut().speed = s.value().parse().unwrap_or(0.25);
        });
        let a = app.clone();
        on(&app.borrow().el("time"), "input", move |e: web_sys::Event| {
            let s: HtmlInputElement = e.target().expect("target").unchecked_into();
            let mut a = a.borrow_mut();
            a.t = s.value().parse().unwrap_or(0.0);
            a.playing = false;
            a.el("playbtn").set_text_content(Some(a.tr("play")));
        });
    }
    for (id, f) in [
        ("t_trails", (|s: &mut Show| &mut s.trails) as fn(&mut Show) -> &mut bool),
        ("t_poly", |s| &mut s.poly),
        ("t_cp", |s| &mut s.cp),
        ("t_body", |s| &mut s.body),
    ] {
        let id2 = id.to_string();
        click(&app, id, move |a| {
            let v = f(&mut a.show);
            *v = !*v;
            let on = *v;
            let _ = a.el(&id2).class_list().toggle_with_force("on", on);
        });
    }
    for (id, pr) in [("v_persp", Preset::Persp), ("v_top", Preset::Top), ("v_side", Preset::Side), ("v_front", Preset::Front)] {
        click(&app, id, move |a| {
            if let Some(v) = a.view.as_mut() {
                v.preset(pr);
            }
        });
    }
    for id in ["plotx", "ploty"] {
        let a = app.clone();
        on(&app.borrow().el(id), "pointerdown", move |e: PointerEvent| {
            let mut a = a.borrow_mut();
            let c: HtmlCanvasElement = a.el(id).unchecked_into();
            a.t = plot::time_at(&c, e.offset_x() as f64, a.sim.plan.duration());
            a.input("time").set_value(&format!("{}", a.t));
        });
    }
    wire_canvas(&app, &canvas);

    // Animation loop.
    let f: Rc<RefCell<Option<Closure<dyn FnMut(f64)>>>> = Rc::new(RefCell::new(None));
    let g = f.clone();
    let a = app.clone();
    *g.borrow_mut() = Some(Closure::new(move |now: f64| {
        a.borrow_mut().frame(now);
        if let (Some(w), Some(cb)) = (web_sys::window(), f.borrow().as_ref()) {
            let _ = w.request_animation_frame(cb.as_ref().unchecked_ref());
        }
    }));
    if let Some(cb) = g.borrow().as_ref() {
        let _ = win.request_animation_frame(cb.as_ref().unchecked_ref());
    };
}

fn wire_canvas(app: &Shared, canvas: &HtmlCanvasElement) {
    let el: &Element = canvas.as_ref();
    let a = app.clone();
    let cv = canvas.clone();
    on(el, "pointerdown", move |e: PointerEvent| {
        e.prevent_default();
        let _ = cv.set_pointer_capture(e.pointer_id());
        let mut a = a.borrow_mut();
        let App { view, sim, .. } = &mut *a;
        if let Some(v) = view.as_mut() {
            v.down(sim, e.pointer_id(), e.button(), e.offset_x() as f64, e.offset_y() as f64);
            let _ = cv.style().set_property("cursor", v.cursor());
        }
    });
    let a = app.clone();
    let cv = canvas.clone();
    on(el, "pointermove", move |e: PointerEvent| {
        let mut a = a.borrow_mut();
        let App { view, sim, .. } = &mut *a;
        let Some(v) = view.as_mut() else { return };
        let edit = v.moved(sim, e.pointer_id(), e.offset_x() as f64, e.offset_y() as f64);
        let _ = cv.style().set_property("cursor", v.cursor());
        if let Some((i, pos)) = edit {
            a.feet[i].pos = pos;
            a.edited = true;
            a.resim(false);
        }
    });
    for ev in ["pointerup", "pointercancel"] {
        let a = app.clone();
        let cv = canvas.clone();
        on(el, ev, move |e: PointerEvent| {
            if let Some(v) = a.borrow_mut().view.as_mut() {
                v.up(e.pointer_id());
                let _ = cv.style().set_property("cursor", v.cursor());
            }
        });
    }
    let a = app.clone();
    on(el, "wheel", move |e: web_sys::WheelEvent| {
        e.prevent_default();
        if let Some(v) = a.borrow_mut().view.as_mut() {
            v.cam.zoom((e.delta_y() * 0.0015).exp() as f32);
        }
    });
    let a = app.clone();
    on(el, "dblclick", move |_: web_sys::Event| {
        if let Some(v) = a.borrow_mut().view.as_mut() {
            v.reset();
        }
    });
    on(el, "contextmenu", |e: web_sys::Event| e.prevent_default());
}
