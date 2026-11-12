//! Tiny canvas-2D time plot: several series against t, with a vertical time cursor.
use wasm_bindgen::{JsCast, JsValue};
use web_sys::{CanvasRenderingContext2d as Ctx, HtmlCanvasElement};

pub struct Series<'a> {
    pub name: &'a str,
    pub color: &'a str,
    pub dashed: bool,
    pub ys: Vec<f64>,
}

/// Margins (CSS px) of the plot area: left, right, top, bottom.
const M: [f64; 4] = [46.0, 10.0, 22.0, 22.0];

/// Time (s) under a horizontal canvas position (CSS px), for click-to-seek.
pub fn time_at(canvas: &HtmlCanvasElement, cx: f64, t_end: f64) -> f64 {
    let w = canvas.client_width().max(1) as f64;
    ((cx - M[0]) / (w - M[0] - M[1]) * t_end).clamp(0.0, t_end)
}

pub fn draw(canvas: &HtmlCanvasElement, title: &str, dt: f64, series: &[Series], t: f64, dark: bool) {
    let dpr = web_sys::window().map(|w| w.device_pixel_ratio()).unwrap_or(1.0);
    let (w, h) = (canvas.client_width().max(1) as f64, canvas.client_height().max(1) as f64);
    let (pw, ph) = ((w * dpr).round() as u32, (h * dpr).round() as u32);
    if canvas.width() != pw || canvas.height() != ph {
        canvas.set_width(pw);
        canvas.set_height(ph);
    }
    let Some(c) = canvas.get_context("2d").ok().flatten().and_then(|c| c.dyn_into::<Ctx>().ok()) else { return };
    let _ = c.set_transform(dpr, 0.0, 0.0, dpr, 0.0, 0.0);
    let (bg, fg, grid) = if dark { ("#16191f", "#d8dbe2", "#2c313a") } else { ("#ffffff", "#2a2d33", "#e3e5ea") };
    c.set_fill_style_str(bg);
    c.fill_rect(0.0, 0.0, w, h);
    let n = series.iter().map(|s| s.ys.len()).max().unwrap_or(0);
    if n < 2 {
        return;
    }
    let t_end = (n - 1) as f64 * dt;
    let (mut lo, mut hi) = series.iter().flat_map(|s| s.ys.iter()).fold((f64::MAX, f64::MIN), |(a, b), &y| (a.min(y), b.max(y)));
    let pad = ((hi - lo) * 0.08).max(1e-3);
    (lo, hi) = (lo - pad, hi + pad);
    let (x0, x1, y0, y1) = (M[0], w - M[1], M[2], h - M[3]);
    let px = |t: f64| x0 + t / t_end * (x1 - x0);
    let py = |y: f64| y1 - (y - lo) / (hi - lo) * (y1 - y0);

    c.set_font("11px system-ui, sans-serif");
    c.set_line_width(1.0);
    c.set_stroke_style_str(grid);
    c.set_fill_style_str(fg);
    // Time grid every second, value grid at "nice" steps.
    let step = nice((hi - lo) / 4.0);
    c.begin_path();
    let mut ts = 0.0;
    while ts <= t_end + 1e-9 {
        c.move_to(px(ts), y0);
        c.line_to(px(ts), y1);
        let _ = c.fill_text(&format!("{ts:.0}"), px(ts) - 3.0, h - 6.0);
        ts += 1.0;
    }
    let mut v = (lo / step).ceil() * step;
    while v <= hi {
        c.move_to(x0, py(v));
        c.line_to(x1, py(v));
        let _ = c.fill_text(&format!("{v:.2}"), 4.0, py(v) + 4.0);
        v += step;
    }
    c.stroke();
    let _ = c.fill_text(title, x0, 14.0);
    let mut lx = x0 + 12.0 + c.measure_text(title).map(|m| m.width()).unwrap_or(90.0);
    c.set_font("10px system-ui, sans-serif");
    for s in series {
        c.set_stroke_style_str(s.color);
        c.set_line_width(if s.dashed { 1.0 } else { 1.6 });
        let dash = js_sys::Array::new();
        if s.dashed {
            dash.push(&JsValue::from(4.0));
            dash.push(&JsValue::from(3.0));
        }
        let _ = c.set_line_dash(&dash);
        c.begin_path();
        for (i, &y) in s.ys.iter().enumerate() {
            let (x, y) = (px(i as f64 * dt), py(y));
            if i == 0 {
                c.move_to(x, y);
            } else {
                c.line_to(x, y);
            }
        }
        c.stroke();
        c.begin_path();
        c.move_to(lx, 10.0);
        c.line_to(lx + 14.0, 10.0);
        c.stroke();
        let _ = c.fill_text(s.name, lx + 18.0, 14.0);
        lx += 24.0 + c.measure_text(s.name).map(|m| m.width()).unwrap_or(40.0);
    }
    let _ = c.set_line_dash(&js_sys::Array::new());
    c.set_stroke_style_str(fg);
    c.set_line_width(1.0);
    c.begin_path();
    c.move_to(px(t), y0);
    c.line_to(px(t), y1);
    c.stroke();
}

/// 1, 2 or 5 × 10^n just above `x`.
fn nice(x: f64) -> f64 {
    let e = 10f64.powf(x.max(1e-9).log10().floor());
    [1.0, 2.0, 5.0, 10.0].into_iter().map(|m| m * e).find(|&s| s >= x).unwrap_or(10.0 * e)
}
