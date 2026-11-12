//! 2D convex polygon helpers on the floor plane (x forward, y left). Polygons are CCW.
pub type P2 = [f64; 2];

pub fn sub(a: P2, b: P2) -> P2 {
    [a[0] - b[0], a[1] - b[1]]
}
pub fn lerp(a: P2, b: P2, s: f64) -> P2 {
    [a[0] + (b[0] - a[0]) * s, a[1] + (b[1] - a[1]) * s]
}
pub fn len(a: P2) -> f64 {
    a[0].hypot(a[1])
}
fn cross(o: P2, a: P2, b: P2) -> f64 {
    (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])
}

/// Axis-aligned foot rectangle (length along x, width along y) around `c`, CCW.
pub fn rect(c: P2, l: f64, w: f64) -> Vec<P2> {
    let (a, b) = (l / 2.0, w / 2.0);
    vec![[c[0] - a, c[1] - b], [c[0] + a, c[1] - b], [c[0] + a, c[1] + b], [c[0] - a, c[1] + b]]
}

/// Convex hull (Andrew's monotone chain), CCW, collinear points dropped.
pub fn hull(pts: &[P2]) -> Vec<P2> {
    let mut p = pts.to_vec();
    p.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
    p.dedup();
    if p.len() < 3 {
        return p;
    }
    let mut h: Vec<P2> = Vec::with_capacity(p.len() * 2);
    for _ in 0..2 {
        let start = h.len();
        for &q in &p {
            while h.len() >= start + 2 && cross(h[h.len() - 2], h[h.len() - 1], q) <= 1e-15 {
                h.pop();
            }
            h.push(q);
        }
        h.pop();
        p.reverse();
    }
    h
}

/// Edges as (start, end) pairs.
fn edges(poly: &[P2]) -> impl Iterator<Item = (P2, P2)> + '_ {
    (0..poly.len()).map(move |i| (poly[i], poly[(i + 1) % poly.len()]))
}

/// Closest point of segment ab to p.
fn seg_closest(a: P2, b: P2, p: P2) -> P2 {
    let d = sub(b, a);
    let dd = d[0] * d[0] + d[1] * d[1];
    let s = if dd > 0.0 { (((p[0] - a[0]) * d[0] + (p[1] - a[1]) * d[1]) / dd).clamp(0.0, 1.0) } else { 0.0 };
    lerp(a, b, s)
}

pub fn contains(poly: &[P2], p: P2) -> bool {
    poly.len() >= 3 && edges(poly).all(|(a, b)| cross(a, b, p) >= -1e-12)
}

/// Signed distance to the polygon boundary: + inside (distance to the nearest edge), − outside.
pub fn signed_dist(poly: &[P2], p: P2) -> f64 {
    let d = edges(poly).map(|(a, b)| len(sub(p, seg_closest(a, b, p)))).fold(f64::INFINITY, f64::min);
    if contains(poly, p) { d } else { -d }
}

/// Closest point of the polygon (p itself if inside).
pub fn clamp(poly: &[P2], p: P2) -> P2 {
    if contains(poly, p) || poly.is_empty() {
        return p;
    }
    edges(poly)
        .map(|(a, b)| seg_closest(a, b, p))
        .min_by(|x, y| len(sub(p, *x)).total_cmp(&len(sub(p, *y))))
        .unwrap_or(p)
}

/// Distance from an inside point p along unit direction `dir` until the boundary (0 if outside).
pub fn ray_exit(poly: &[P2], p: P2, dir: P2) -> f64 {
    if !contains(poly, p) {
        return 0.0;
    }
    edges(poly)
        .filter_map(|(a, b)| {
            // Outward normal of a CCW edge.
            let e = sub(b, a);
            let n = [e[1], -e[0]];
            let nd = n[0] * dir[0] + n[1] * dir[1];
            (nd > 1e-12).then(|| ((a[0] - p[0]) * n[0] + (a[1] - p[1]) * n[1]) / nd)
        })
        .fold(f64::INFINITY, f64::min)
        .max(0.0)
}
