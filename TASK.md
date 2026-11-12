# Task: Humanoid ZMP & capture-point walking lab (Rust → WebAssembly, static web app)

Original open-source tool. Write all code yourself; do not copy anyone's code, tables or assets.
Use textbook Linear Inverted Pendulum Model (LIPM) / ZMP / capture-point equations only.
Presets are generic, clearly labelled "typical" values that users can edit (no vendor names).

## Product (SMALL scope — core + UI about 800-1200 lines of new Rust; renderer/math are reused)
A single-page web app (GitHub Pages, sub-path `/zmp-lab/`): "My humanoid walks with this CoM height,
step length, step time and foot size — is the gait balanced, where does the ZMP / capture point go,
and how hard can I push it before it must take a step?"

1. **Inputs**: CoM height z_c (m), gravity (default 9.81), foot length × width (m), step length (m),
   step width (lateral foot distance, m), single-support time T_ss (s), double-support ratio (0-0.4),
   number of steps (2-10). Two presets: "adult-size humanoid (typical)" and "small kid-size humanoid
   (typical)" (editable).
2. **Gait model (core)**: footstep plan alternating left/right; ZMP reference = piecewise: constant at the
   stance-foot centre during single support, linear interpolation between feet during double support.
   CoM trajectory from the LIPM x'' = ω²(x − p), ω = √(g/z_c), integrated with a fixed small dt
   (semi-implicit or RK4) using a simple **capture-point (DCM) controller**: ξ = x + ẋ/ω, ZMP command
   p = ξ_ref,end-based closed form or p = ξ + k(ξ − ξ_ref) with gain k (default 1.5) and ZMP clamped to
   the current support polygon (stance foot rectangle, or convex hull of both feet in double support).
   Outputs: ω, time constant 1/ω, per-sample CoM, ZMP, capture point; minimum stability margin
   (signed distance of ZMP to support polygon edge, mm); whether ZMP ever saturated (clamped).
3. **Push recovery (extra)**: a "push" button + impulse Δv (m/s, direction angle) applied at a chosen time.
   Report: capture point leaves the support polygon? (⚠️ step needed) and max recoverable Δv while
   standing on one foot = ω × (distance from CoM to polygon edge along the push direction).
4. **3D view (WebGL2, game-grade — see below)**: checker floor, foot rectangles (left blue / right orange,
   current stance highlighted), support polygon as a translucent green slab, a simple stick humanoid
   (pelvis box at CoM height, capsule legs from hip to feet with swing foot following a smooth arc,
   torso capsule, head sphere), a vertical line from CoM to floor, ZMP marker (red disc), capture point
   marker (yellow ring), and the trails of CoM-ground projection / ZMP / capture point.
5. **2D plots** (canvas 2D, own small module): x(t) and y(t) for CoM, ZMP, capture point, with the
   current time as a vertical cursor.
6. Korean + English toggle (browser default). Mobile friendly. CSS prefers-color-scheme only.
Out of scope (do NOT build): full-body inverse kinematics, preview control (Kajita), MPC, uneven terrain,
URL share, JSON import/export.

## 3D view requirements (game-like, reuse the existing renderer)
- `crates/zl_web/src/gl.rs` and `crates/zl_web/src/math.rs` already exist: a tool-independent WebGL2
  renderer (lit meshes, checker/grid ground with fog, sky gradient, stencil planar shadows, translucent
  volumes, lines) and pure math (mat4, damped orbit camera with inertia, ray-plane/ray-sphere picking)
  with unit tests. Reuse them; extend only if needed and keep them tool-independent and tested.
- Scene code in `view3d.rs`. Orbit (left drag) / pan (right drag, two fingers) / zoom (wheel, pinch)
  with inertia, double-click reset, `touch-action:none`. View presets: Perspective / Top / Side / Front.
- Direct manipulation: drag a **footstep** (any non-first foot) on the floor plane (ray-plane) to edit
  that step's position; the gait recomputes live. Hover highlight + cursor change on draggable feet.
- Toggle buttons over the view: trails, support polygon, capture point, humanoid.
- Playback: play/pause, speed (0.1× / 0.25× / 1×, default 0.25×), time slider; same instant shown as a
  cursor on the 2D plots. HUD over the canvas: t, phase (L/R/double), ZMP margin (mm), |ξ − ZMP|.
- Colour the ZMP marker red while it is clamped (saturated), green otherwise.
- If WebGL2 is unavailable, show a message and keep the 2D plots working (2D top view fallback ok).
- Release wasm ≲ 350 KB. No crates besides wasm-bindgen, web-sys, js-sys.

## Core crate (pure Rust, no web deps, unit tested) — `crates/zl_core`
- Footstep plan, support polygon (convex hull + point-in-polygon + signed edge distance), ZMP
  reference, LIPM simulation with DCM controller, push recovery, max recoverable Δv.
- Tests (hand-checkable, 6-10): ω for z_c = 0.981 m, g = 9.81 → √10 ≈ 3.162; capture point of x=0, v=0.5,
  ω=√10 → 0.158 m; LIPM with ZMP fixed at 0 from x0=0.01, v0=0 → x(t) = 0.01·cosh(ωt) (integrator within
  1e-4 at t=0.5 s); convex hull of two 0.2×0.1 feet; signed distance of a point inside a unit square =
  distance to nearest edge (negative outside); simulated nominal gait keeps ZMP inside the support
  polygon at all samples and ends near the last footstep midpoint; max recoverable Δv = ω × edge distance;
  a big push makes the capture point leave the polygon.
- `crates/zl_web`: UI, plain web-sys.

## Repo hygiene
- README (Korean first, then English): what it is, model and assumptions (LIPM: constant CoM height,
  point mass, no angular momentum, flat floor, rigid feet), controls, local build
  (`cargo test`, `trunk build --release`), MIT license, "educational estimates — not a controller for a
  real robot without validation".
- LICENSE: MIT, copyright "zmp-lab contributors". `.gitignore` for target/ dist/.
- `index.html` with a CSP meta that blocks external requests:
  `default-src 'self'; script-src 'self' 'wasm-unsafe-eval'; style-src 'self' 'unsafe-inline'; img-src 'self' data: blob:; connect-src 'self'; object-src 'none'; base-uri 'self'`.
  In `Trunk.toml` set `filehash = false`, `public_url = "/zmp-lab/"` and
  `pattern_script = "<script type=\"module\" src=\"{base}boot.js\"></script>"` with an external
  `web/boot.js` (copied via `<link data-trunk rel="copy-file">`) that imports `./zl_web.js` and calls init.
- `.github/workflows/pages.yml`: on push to main + workflow_dispatch; checkout, rust stable + wasm32
  target, install trunk 0.22.1 binary release, `sh tools/privacy_check.sh`, `cargo test --release`,
  `trunk build --release`, configure-pages@v5, upload-pages-artifact@v3 (dist), deploy job with
  actions/deploy-pages@v4 (permissions pages: write, id-token: write).
- `tools/privacy_check.sh` already exists; keep it and keep it clean.

## Done when
- `cargo test` green (workspace); `trunk build --release` succeeds with no warnings;
  `sh tools/privacy_check.sh` clean.
- Commit author `zmp-lab contributors <59640857+roscore@users.noreply.github.com>`.
- Commit everything with clear commits; open a PR. Do NOT create new GitHub repos or change repo settings.
