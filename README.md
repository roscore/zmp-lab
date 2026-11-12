# zmp-lab — ZMP · 캡처 포인트 보행 랩

브라우저에서 바로 실행되는 휴머노이드 보행 균형 계산기입니다 (Rust → WebAssembly, WebGL2).
"우리 휴머노이드가 이 CoM 높이, 보폭, 스텝 시간, 발 크기로 걸으면 균형이 맞을까? ZMP와 캡처 포인트는
어디로 가고, 발을 내딛지 않고 버틸 수 있는 외란은 얼마까지일까?"에 답합니다.

> **교육용 추정치입니다 — 검증 없이 실제 로봇의 제어기로 사용하지 마세요.**
> 모든 계산은 브라우저 안에서만 수행됩니다 (서버, 분석 도구, 외부 요청 없음).

## 한국어

### 기능
- **입력**: CoM 높이 z_c, 중력, 발 길이 × 폭, 보폭, 양발 간격, 단일 지지 시간 T_ss, 이중 지지 비율(0–0.4),
  걸음 수(2–10), DCM 게인 k. 프리셋 2개("성인 크기 휴머노이드 (일반)", "소형 키드 크기 휴머노이드 (일반)") — 모두 수정 가능한 일반적인 값입니다.
- **보행 모델**: 왼발/오른발이 번갈아 나오는 발자국 계획. ZMP 기준은 단일 지지 동안 지지발 중심에 고정,
  이중 지지 동안 두 발 사이를 선형 보간합니다. 마지막 걸음은 발을 모으고 두 발의 중점에서 멈춥니다.
- **출력**: ω, 시간 상수 1/ω, 시간별 CoM · ZMP · 캡처 포인트, 최소 안정 여유(ZMP와 지지 다각형 가장자리 사이의 부호 있는 거리, mm), ZMP 포화(클램프) 여부.
- **외란(밀기)**: 지정한 시각에 속도 충격 Δv(크기, 방향)를 가합니다. 밀기 직후 캡처 포인트가 지지 다각형을 벗어나면
  "⚠️ 발을 내디뎌야 함"으로 표시하고, 최대 회복 가능 Δv = ω × (CoM에서 밀기 방향으로 다각형 가장자리까지의 거리)를 보여 줍니다.
  현재 속도까지 고려한 값(캡처 포인트 기준)도 함께 표시합니다.
- **3D 뷰**: 체커 바닥, 발(왼쪽 파랑 / 오른쪽 주황, 지지발 강조), 반투명 초록 지지 다각형, 막대 휴머노이드,
  CoM 수직선, ZMP 원판(클램프 중 빨강, 평소 초록), 캡처 포인트 링(노랑), 궤적.
- **2D 그래프**: x(t), y(t)의 CoM · ZMP · ZMP 기준 · 캡처 포인트, 현재 시각 커서 (그래프를 클릭하면 그 시각으로 이동).

### 모델과 가정
- 선형 역진자 모델(LIPM): **CoM 높이 일정, 점질량, 각운동량 없음, 평평한 바닥, 강체 발**.
- 운동 방정식: ẍ = ω²(x − p), ω = √(g / z_c). 캡처 포인트(DCM): ξ = x + ẋ / ω, ξ̇ = ω(ξ − p).
- DCM 기준 궤적 ξ_ref는 구간별 선형 ZMP 기준에 대해 마지막 정지 상태부터 거꾸로 닫힌 형태로 계산합니다:
  ξ(t) = p(t) + ṗ/ω + e^{ω(t−T)}(ξ_T − p_T − ṗ/ω).
- 제어 법칙: p = p_ref + (1 + k)(ξ − ξ_ref). 정지 상태에서는 p = ξ + k(ξ − ξ_ref)와 같고, 오차는 e^{−kωt}로 줄어듭니다.
  명령 ZMP는 현재 지지 다각형(단일 지지: 지지발 사각형, 이중 지지: 두 발의 볼록 껍질)으로 클램프됩니다.
- 적분: RK4, dt = 1 ms (구간 안에서 ZMP 일정), 5 ms마다 기록.
- 밀기 후 캡처 포인트가 다각형 밖으로 0.3 m 이상 벗어나면 "넘어짐"으로 보고 시뮬레이션을 멈춥니다. 발 위치를 바꾸는 회복 걸음은 모델링하지 않습니다.
- 발은 항상 진행 방향(x)으로 정렬된 사각형입니다. 역기구학, 프리뷰 제어, MPC, 고르지 않은 지면은 다루지 않습니다.

### 조작
- 왼쪽 드래그: 회전 · 오른쪽 드래그 / 두 손가락: 이동 · 휠 / 핀치: 확대 · 더블클릭: 시점 초기화 (관성 포함).
- 첫 번째 발을 제외한 **발자국을 끌어서** 위치를 바꾸면 보행이 즉시 다시 계산됩니다.
- 뷰 프리셋: 원근 / 위 / 옆 / 앞. 표시 토글: 궤적, 지지 다각형, 캡처 포인트, 휴머노이드.
- 재생/일시정지, 속도 0.1× / 0.25× / 1×, 시간 슬라이더. 언어는 브라우저 기본값을 따르며 버튼으로 전환합니다.

### 로컬 빌드
```sh
rustup target add wasm32-unknown-unknown
cargo test                 # 코어 단위 테스트
trunk build --release      # dist/ 생성 (public_url = /zmp-lab/)
sh tools/privacy_check.sh  # 개인정보 패턴 검사
```
`trunk serve`로 로컬에서 띄운 뒤 `/zmp-lab/` 경로로 접속하세요.

### 라이선스
MIT — `LICENSE` 참고.

## English

### What it is
A static, fully client-side web app (Rust → WebAssembly, WebGL2) that answers: "My humanoid walks with this CoM
height, step length, step time and foot size — is the gait balanced, where do the ZMP and capture point go, and how
hard can I push it before it must take a step?"

> **Educational estimates — not a controller for a real robot without validation.**
> Everything runs in your browser: no server, no analytics, no external requests.

### Features
- **Inputs**: CoM height z_c, gravity, foot length × width, step length, step width, single-support time T_ss,
  double-support ratio (0–0.4), number of steps (2–10), DCM gain k. Two editable presets: "adult-size humanoid (typical)"
  and "small kid-size humanoid (typical)".
- **Gait model**: alternating left/right footsteps; ZMP reference constant at the stance-foot centre during single
  support and linearly interpolated between the feet during double support; the last step closes the feet and the robot
  comes to rest at their midpoint.
- **Outputs**: ω, time constant 1/ω, per-sample CoM / ZMP / capture point, minimum stability margin (signed distance of
  the ZMP to the support-polygon edge, mm) and whether the ZMP was ever saturated (clamped).
- **Push recovery**: a velocity impulse Δv (magnitude, direction) at a chosen time. Reports whether the capture point
  leaves the support polygon right after the push (⚠️ step needed) and the max recoverable Δv = ω × (distance from the
  CoM to the polygon edge along the push direction), plus the same bound measured from the current capture point.
- **3D view**: checker floor, feet (left blue / right orange, stance highlighted), translucent green support polygon,
  stick humanoid, CoM plumb line, ZMP disc (red while clamped, green otherwise), capture-point ring (yellow) and trails.
- **2D plots**: x(t) and y(t) of CoM, ZMP, ZMP reference and capture point with a time cursor (click to seek).

### Model and assumptions
- Linear Inverted Pendulum Model: **constant CoM height, point mass, no angular momentum, flat floor, rigid feet**.
- ẍ = ω²(x − p), ω = √(g / z_c); capture point (DCM) ξ = x + ẋ / ω with ξ̇ = ω(ξ − p).
- The DCM reference is computed in closed form backwards from the final rest state over the piecewise-linear ZMP
  reference: ξ(t) = p(t) + ṗ/ω + e^{ω(t−T)}(ξ_T − p_T − ṗ/ω).
- Control law: p = p_ref + (1 + k)(ξ − ξ_ref) (equal to p = ξ + k(ξ − ξ_ref) when standing); the DCM error decays as
  e^{−kωt}. The commanded ZMP is clamped to the current support polygon (stance-foot rectangle in single support,
  convex hull of both feet in double support).
- Integration: RK4 with dt = 1 ms (ZMP held over each step), sampled every 5 ms.
- After a push, a capture point more than 0.3 m outside the polygon counts as a fall and the simulation stops there;
  recovery steps (changing footholds) are not modelled.
- Feet are rectangles aligned with the walking direction. Out of scope: inverse kinematics, preview control, MPC,
  uneven terrain.

### Controls
- Left-drag: orbit · right-drag / two fingers: pan · wheel / pinch: zoom · double-click: reset (with inertia).
- **Drag any footstep except the first** to move it; the gait is recomputed live.
- View presets: Perspective / Top / Side / Front. Toggles: trails, support polygon, capture point, humanoid.
- Play/pause, speed 0.1× / 0.25× / 1×, time slider. Language follows the browser; the header button switches it.

### Local build
```sh
rustup target add wasm32-unknown-unknown
cargo test                 # core unit tests
trunk build --release      # writes dist/ (public_url = /zmp-lab/)
sh tools/privacy_check.sh  # personal-data pattern check
```
Use `trunk serve` and open the `/zmp-lab/` path locally.

### License
MIT — see `LICENSE`.
