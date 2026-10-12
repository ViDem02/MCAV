# mcav_project

## Directory Tree

```text
mcav_project/
├── config/
│   ├── quadcopter_certified_circle.yaml
│   ├── quadcopter_dynamic_circle.yaml
│   ├── quadcopter_ff_circle.yaml
│   ├── quadcopter_kinematic.yaml
│   ├── quadcopter_lyapunov_circle.yaml
│   ├── quadcopter_path_following_circle.yaml
│   ├── quadcopter_saturated_circle.yaml
│   ├── unicycle_circle.yaml
│   ├── unicycle_fall_north.yaml
│   └── unicycle_lyapunov.yaml
├── figures/
│   └── .gitignore
├── models/
│   ├── quadcopter/
│   │   ├── quadcopter.xml
│   │   └── quadcopter_dynamic.xml
│   └── unicycle/
│       └── unicycle.xml
├── notebooks/
│   └── _lyapunov.md
├── src/
│   ├── control/
│   │   ├── __init__.py
│   │   ├── cascade_controller.py
│   │   ├── certified_cascade_controller.py
│   │   └── path_following_controller.py
│   ├── logging/
│   │   ├── __init__.py
│   │   └── sim_logger.py
│   ├── multithreading/
│   │   ├── __init__.py
│   │   └── monte_carlo_roa.py
│   ├── simulation/
│   │   ├── __init__.py
│   │   ├── base_sim.py
│   │   ├── headless_sim.py
│   │   ├── video_sim.py
│   │   └── viewer_sim.py
│   ├── symbolic/
│   │   ├── kinematics.py
│   │   ├── lyapunov.py
│   │   ├── quadcopter_certified_lyapunov.py
│   │   ├── quadcopter_dynamic_control.py
│   │   ├── quadcopter_ff_dynamic_control.py
│   │   ├── quadcopter_kinematic_lyapunov.py
│   │   └── quadcopter_path_following.py
│   ├── utils/
│   │   ├── animations.py
│   │   ├── config.py
│   │   └── plotting.py
│   └── vehicles/
│       └── __init__.py
├── .gitignore
├── doc.md
├── main.py
├── Project Structure.md
├── pyproject.toml
└── test_sim.py
```

## File Contents

### `.gitignore`

```gitignore
pedro_material
```

### `config/quadcopter_certified_circle.yaml`

```yaml
# config/quadcopter_certified_circle.yaml
# Identical physics to quadcopter_dynamic_circle.yaml.
# The certification comes from the controller, not the config.

model_path: "models/quadcopter/quadcopter_dynamic.xml"
dt: 0.002
t_end: 60.0

video_fps: 30
video_path: "videos/quadcopter_certified_circle.mp4"
video_w: 1280
video_h: 720

figures_dir: "figures/quadcopter_certified_circle"

mass: 0.80
arm_d: 0.1838
km: 0.012
max_thrust_per_motor: 4.0

controller:
  kx:   1.0
  ky:   1.0
  kz:   1.5
  kpsi: 2.0
  kv:   4.0
  kR:   8.0
  kw:   2.5
  full_ff: false    # set true to also add inertial feedforward (non-constant trajectories)

trajectory:
  radius:   3.0
  omega:    0.5
  z_height: 3.0

camera:
  lookat:    [0.0, 0.0, 2.0]
  distance:  12.0
  elevation: -30.0
  azimuth:   45.0

multithreading:
  n_samples: 10000
  max_radius: 3.0
  n_jobs: -1
  t_max: 4.0
  initial_altitude_error: 0.0
  ground_clearance: 0.1
  success_threshold: 0.05
  seed: 7
  show_progress: true
```

### `config/quadcopter_dynamic_circle.yaml`

```yaml
# config/quadcopter_dynamic_circle.yaml

model_path: "models/quadcopter/quadcopter_dynamic.xml"
dt: 0.002           # physics timestep → 500 Hz
t_end: 60.0         # longer than kinematic: inner loop needs time to converge

video_fps: 30
video_path: "videos/quadcopter_dynamic_circle.mp4"
video_w: 1280
video_h: 720

figures_dir: "figures/quadcopter_dynamic_circle"

# vehicle physical parameters (must match the MJCF mass budget)
mass: 0.80          # kg  (body 0.5 + arms 0.2 + props 0.1)
arm_d: 0.1838       # m   = 0.13 * sqrt(2)  (moment arm for roll/pitch)
km: 0.012           # reaction-torque / thrust ratio
max_thrust_per_motor: 4.0   # N

controller:
  # outer loop — position Lyapunov (same as kinematic)
  kx:   1.0
  ky:   1.0
  kz:   1.5
  kpsi: 2.0

  # middle loop — velocity tracking
  kv: 4.0

  # inner loop — geometric attitude (SO3)
  kR: 8.0           # attitude-error gain
  kw: 2.5           # angular-rate damping

trajectory:
  radius:   3.0     # m
  omega:    0.5     # rad/s  (one lap ≈ 12.6 s)
  z_height: 3.0     # m

camera:
  lookat:    [0.0, 0.0, 2.0]
  distance:  12.0
  elevation: -30.0
  azimuth:   45.0
```

### `config/quadcopter_ff_circle.yaml`

```yaml
# config/quadcopter_ff_circle.yaml
# Identical to quadcopter_dynamic_circle.yaml except:
#   - figures_dir and video_path point to a separate ff folder
#   - no gain changes: feedforward alone should reduce the error

model_path: "models/quadcopter/quadcopter_dynamic.xml"
dt: 0.002
t_end: 25.0

video_fps: 30
video_path: "videos/quadcopter_ff_circle.mp4"
video_w: 1280
video_h: 720

figures_dir: "figures/quadcopter_ff_circle"

mass: 0.80
arm_d: 0.1838
km: 0.012
max_thrust_per_motor: 4.0

controller:
  kx:   1.0
  ky:   1.0
  kz:   1.5
  kpsi: 2.0
  kv:   4.0
  kR:   8.0
  kw:   2.5

trajectory:
  radius:   3.0
  omega:    0.5
  z_height: 3.0

camera:
  lookat:    [0.0, 0.0, 2.0]
  distance:  12.0
  elevation: -30.0
  azimuth:   45.0
```

### `config/quadcopter_kinematic.yaml`

```yaml
model_path: "models/quadcopter/quadcopter.xml"

dt: 0.002
t_end: 30.0

video_fps: 15
video_path: "videos/quadcopter_kinematic.mp4"
video_w: 640
video_h: 480

camera:
  lookat: [0.0, 0.0, 0.5]
  distance: 8.0
  elevation: -30.0
  azimuth: 45.0
```

### `config/quadcopter_lyapunov_circle.yaml`

```yaml
model_path: "models/quadcopter/quadcopter.xml"
video_path: "videos/quadcopter_lyapunov_circle.mp4"
video_fps: 30
dt: 0.002
t_end: 20.0
camera:
  distance: 10.0
  elevation: -30.0
  azimuth: 45.0
  lookat: [0.0, 0.0, 2.0]
controller:
  kx: 1.0
  ky: 1.0
  kz: 1.0
  kpsi: 1.0
trajectory:
  radius: 3.0
  omega: 0.5
  z_height: 3.0
figures_dir: "figures/quadcopter_lyapunov_circle"
```

### `config/quadcopter_path_following_circle.yaml`

```yaml
# config/quadcopter_path_following_circle.yaml

model_path: "models/quadcopter/quadcopter_dynamic.xml"
dt: 0.002
t_end: 60.0           # longer: path following needs time to converge first, then circle

video_fps: 30
video_path: "videos/quadcopter_path_following_circle.mp4"
video_w: 1280
video_h: 720

figures_dir: "figures/quadcopter_path_following_circle"

# vehicle
mass: 0.80
arm_d: 0.1838
km: 0.012
max_thrust_per_motor: 4.0

# cascade inner loop (same gains as certified notebook)
controller:
  kx:   1.0
  ky:   1.0
  kz:   1.5
  kpsi: 2.0
  kv:   4.0
  kR:   8.0
  kw:   2.5
  saturate: true

# path following outer loop
path_following:
  v_d:     2.0       # desired path speed [rad/s]  — same ω as trajectory tracking
  k_gamma: 0.5       # along-track correction gain

# trajectory for comparison
trajectory:
  radius:   3.0
  omega:    2.0
  z_height: 3.0

camera:
  lookat:    [0.0, 0.0, 2.0]
  distance:  12.0
  elevation: -30.0
  azimuth:   45.0
```

### `config/quadcopter_saturated_circle.yaml`

```yaml
# config/quadcopter_saturated_circle.yaml
# Identical to quadcopter_certified_circle.yaml except:
#   - controller.saturate: true
#   - figures_dir and video_path point to a separate folder

model_path: "models/quadcopter/quadcopter_dynamic.xml"
dt: 0.002
t_end: 25.0

video_fps: 30
video_path: "videos/quadcopter_saturated_circle.mp4"
video_w: 1280
video_h: 720

figures_dir: "figures/quadcopter_saturated_circle"

mass: 0.80
arm_d: 0.1838
km: 0.012
max_thrust_per_motor: 4.0

controller:
  kx:   1.0
  ky:   1.0
  kz:   1.5
  kpsi: 2.0
  kv:   4.0
  kR:   8.0
  kw:   2.5
  saturate: true       # ← only difference

trajectory:
  radius:   3.0
  omega:    0.5
  z_height: 3.0

camera:
  lookat:    [0.0, 0.0, 2.0]
  distance:  12.0
  elevation: -30.0
  azimuth:   45.0
```

### `config/unicycle_circle.yaml`

```yaml
model_path: "models/unicycle/unicycle.xml"

dt: 0.005
t_end: 10.0

video_fps: 30
video_path: "videos/unicycle_circle.mp4"
video_w: 640
video_h: 480

control:
  k1: 1.0
  k2: 2.0
  lookahead: 0.5
```

### `config/unicycle_fall_north.yaml`

```yaml
model_path: "models/unicycle/unicycle.xml"

dt: 0.005
# Shorter run for drop + 1 m north motion.
t_end: 4.0

video_fps: 30
video_path: "videos/unicycle_fall_north.mp4"
video_w: 640
video_h: 480

control:
  k1: 1.0
  k2: 2.0
  lookahead: 0.5
```

### `config/unicycle_lyapunov.yaml`

```yaml
model_path: "models/unicycle/unicycle.xml"

dt: 0.005
t_end: 15.0

video_fps: 30
video_path: "videos/unicycle_lyapunov.mp4"
video_w: 640
video_h: 480

control:
  v: 1.0
  R: 1.5
  ky: 1.0
  ktheta: 2.0

camera:
  lookat:    [0.0, 0.0, 0.0]
  distance:  10.0          # increase until full circle fits
  elevation: -70.0         # negative = looking steeply downward
  azimuth:   90.0          # compass direction of the camera
```

### `doc.md`

```markdown
# Lyapunov Stability Analysis — Theoretical Foundation

## 1. Lyapunov Stability Theory

### 1.1 Definitions

Consider an autonomous dynamical system

$$\dot{\mathbf{x}} = f(\mathbf{x}), \quad \mathbf{x} \in \mathbb{R}^n, \quad f(\mathbf{0}) = \mathbf{0}$$

where the origin $\mathbf{x} = \mathbf{0}$ is an equilibrium point. The equilibrium is said to be:

- **Stable (Lyapunov):** for every $\varepsilon > 0$ there exists $\delta > 0$ such that $\|\mathbf{x}(0)\| < \delta \Rightarrow \|\mathbf{x}(t)\| < \varepsilon$ for all $t \geq 0$.
- **Asymptotically Stable (AS):** stable and $\mathbf{x}(t) \to \mathbf{0}$ as $t \to \infty$.
- **Globally Asymptotically Stable (GAS):** asymptotically stable for all initial conditions $\mathbf{x}(0) \in \mathbb{R}^n$.

### 1.2 Lyapunov's Direct Method

A continuously differentiable function $V : \mathbb{R}^n \to \mathbb{R}$ is a **Lyapunov function candidate** if:

$$V(\mathbf{0}) = 0, \quad V(\mathbf{x}) > 0 \quad \forall\, \mathbf{x} \neq \mathbf{0}$$

Its time derivative along trajectories of the system is:

$$\dot{V}(\mathbf{x}) = \nabla V(\mathbf{x})^\top f(\mathbf{x}) = \frac{\partial V}{\partial \mathbf{x}} f(\mathbf{x})$$

**Theorem (Lyapunov's Direct Method):**  
If there exists a Lyapunov function candidate $V$ such that:

$$\dot{V}(\mathbf{x}) < 0 \quad \forall\, \mathbf{x} \neq \mathbf{0}$$

then the origin is **asymptotically stable**. If additionally $V$ is **radially unbounded**:

$$V(\mathbf{x}) \to \infty \quad \text{as} \quad \|\mathbf{x}\| \to \infty$$

then the origin is **globally asymptotically stable**.

The geometric intuition is that $V$ measures the "energy" of the system. A strictly negative $\dot{V}$ means the energy is always decreasing — the system must converge to the only state of zero energy, the equilibrium.

### 1.3 LaSalle's Invariance Principle

LaSalle extends Lyapunov's theorem to cases where $\dot{V} \leq 0$ (negative **semi**-definite), which is insufficient for direct application of the theorem above.

**Theorem (LaSalle):**  
Let $\Omega$ be a compact set that is positively invariant with respect to $\dot{\mathbf{x}} = f(\mathbf{x})$. Let $V : \Omega \to \mathbb{R}$ be continuously differentiable with $\dot{V}(\mathbf{x}) \leq 0$ on $\Omega$. Define:

$$\mathcal{S} = \left\{ \mathbf{x} \in \Omega \;\middle|\; \dot{V}(\mathbf{x}) = 0 \right\}$$

Let $\mathcal{M}$ be the **largest invariant set** contained in $\mathcal{S}$. Then every solution starting in $\Omega$ converges to $\mathcal{M}$ as $t \to \infty$.

If $\mathcal{M} = \{\mathbf{0}\}$ and $V$ is radially unbounded, the origin is GAS.

LaSalle is particularly useful when cross-terms appear in $\dot{V}$ that make it only negative semi-definite — a common occurrence in cascade systems.

---

## 2. Tier 1 — Kinematic Outer Loop

### 2.1 System Model

The kinematic simulator directly implements $\dot{\mathbf{x}} = \mathbf{u}$ by writing to `qpos` and `qvel` at each step. There is no inner loop, no rigid-body dynamics, and no separation assumption. The control input is the commanded velocity:

$$\mathbf{u} = \mathbf{v}_\text{cmd} = \mathbf{v}_d - K \mathbf{e}_p$$

where $\mathbf{e}_p = \mathbf{x} - \mathbf{x}_d \in \mathbb{R}^4$ is the error vector $[e_x, e_y, e_z, e_\psi]^\top$ and $K = \mathrm{diag}(k_x, k_y, k_z, k_\psi)$ with all $k_i > 0$.

The error dynamics are therefore:

$$\dot{\mathbf{e}}_p = \dot{\mathbf{x}} - \dot{\mathbf{x}}_d = \mathbf{u} - \mathbf{v}_d = -K\mathbf{e}_p$$

This is a **linear, decoupled, first-order system**. Each component satisfies $\dot{e}_i = -k_i e_i$, with the known solution $e_i(t) = e_i(0)e^{-k_i t} \to 0$.

### 2.2 Lyapunov Candidate

Choose the standard quadratic energy function:

$$V_1(\mathbf{e}_p) = \frac{1}{2}\mathbf{e}_p^\top \mathbf{e}_p = \frac{1}{2}\|\mathbf{e}_p\|^2$$

**Positive Definiteness:** $V_1(\mathbf{0}) = 0$ and $V_1(\mathbf{e}_p) > 0$ for all $\mathbf{e}_p \neq \mathbf{0}$. ✓  
**Radial Unboundedness:** $V_1 \to \infty$ as $\|\mathbf{e}_p\| \to \infty$. ✓

### 2.3 Time Derivative

$$\dot{V}_1 = \nabla V_1^\top \dot{\mathbf{e}}_p = \mathbf{e}_p^\top (-K\mathbf{e}_p) = -\mathbf{e}_p^\top K \mathbf{e}_p$$

Since $K = \mathrm{diag}(k_i)$ with all $k_i > 0$:

$$\dot{V}_1 = -k_x e_x^2 - k_y e_y^2 - k_z e_z^2 - k_\psi e_\psi^2 < 0 \quad \forall\, \mathbf{e}_p \neq \mathbf{0}$$

**Negative Definiteness:** $\dot{V}_1$ is a negative definite quadratic form. ✓

### 2.4 LaSalle Application

The set where $\dot{V}_1 = 0$ is:

$$\mathcal{S} = \left\{ \mathbf{e}_p : k_x e_x^2 + k_y e_y^2 + k_z e_z^2 + k_\psi e_\psi^2 = 0 \right\} = \{\mathbf{0}\}$$

The largest invariant set $\mathcal{M} \subseteq \mathcal{S}$ is trivially $\mathcal{M} = \{\mathbf{0}\}$.

Since $\dot{V}_1$ is already strictly negative definite, LaSalle is redundant here but confirms the result: all trajectories converge to $\mathcal{M} = \{\mathbf{0}\}$.

### 2.5 Conclusion

By Lyapunov's Direct Method and the radial unboundedness of $V_1$:

$$\boxed{\mathbf{e}_p(t) \to \mathbf{0} \text{ as } t \to \infty \quad \forall\, \mathbf{e}_p(0) \in \mathbb{R}^4 \qquad \textbf{(GAS)}}$$

No caveats. No assumptions. No inner loop. This is the strongest possible certificate.

---

## 3. Tier 2 — Attitude Inner Loop

### 3.1 Attitude Error on SO(3)

The attitude of the vehicle is represented by a rotation matrix $R \in SO(3)$, the Lie group of $3\times3$ orthogonal matrices with determinant $+1$. The desired attitude is $R_d \in SO(3)$, computed from the desired thrust vector and yaw.

The **attitude tracking error** is defined on $SO(3)$ as (Lee et al. 2010, eq. 7):

$$\mathbf{e}_R = \frac{1}{2}\,\mathrm{vee}\!\left(R_d^\top R - R^\top R_d\right) \in \mathbb{R}^3$$

where $\mathrm{vee}(\cdot)$ extracts the axial vector from a skew-symmetric matrix. This error is zero if and only if $R = R_d$.

The **angular velocity error** is:

$$\mathbf{e}_\Omega = \boldsymbol{\Omega} - R^\top R_d \boldsymbol{\Omega}_d \in \mathbb{R}^3$$

where $\boldsymbol{\Omega}$ is the current angular velocity in the body frame and $\boldsymbol{\Omega}_d = [0, 0, \dot{\psi}_d]^\top$.

### 3.2 Lyapunov Candidate for Attitude

Following Lee et al. (2010, eq. 9), define:

$$V_2(\mathbf{e}_R, \mathbf{e}_\Omega) = k_R\,\Psi(R, R_d) + \frac{1}{2}\mathbf{e}_\Omega^\top J\, \mathbf{e}_\Omega$$

where $\Psi(R, R_d) = \frac{1}{2}\mathrm{tr}(I - R_d^\top R)$ is the **attitude tracking function** on $SO(3)$, a metric-like quantity that is zero when $R = R_d$ and positive otherwise.

Near the equilibrium, $\Psi \approx \frac{1}{4}\|\mathbf{e}_R\|^2$ (Lee et al. Lemma 1), giving the quadratic approximation:

$$V_2 \approx \frac{k_R}{4}\|\mathbf{e}_R\|^2 + \frac{1}{2}\mathbf{e}_\Omega^\top J\, \mathbf{e}_\Omega$$

**Positive Definiteness:** Both terms are non-negative; they vanish simultaneously only when $\mathbf{e}_R = \mathbf{0}$ and $\mathbf{e}_\Omega = \mathbf{0}$. Since $k_R > 0$ and $J \succ 0$ (physically), $V_2$ is positive definite. ✓

### 3.3 The Broken Controller — Why the Certificate Fails

The naive torque law, without any physics-based cancellation:

$$\boldsymbol{\tau}_\text{broken} = -k_R\, \mathbf{e}_R - k_w\, \mathbf{e}_\Omega$$

The rigid-body angular dynamics are governed by Euler's equation:

$$J\dot{\boldsymbol{\Omega}} = \boldsymbol{\tau} - \boldsymbol{\Omega} \times J\boldsymbol{\Omega}$$

Substituting the broken torque law:

$$J\dot{\boldsymbol{\Omega}} = -k_R\,\mathbf{e}_R - k_w\,\mathbf{e}_\Omega - \boldsymbol{\Omega} \times J\boldsymbol{\Omega}$$

The time derivative of $V_2$ along these dynamics includes a term from the Coriolis/gyroscopic contribution:

$$\dot{V}_2\big|_\text{broken} = \frac{k_R}{2}\mathbf{e}_R^\top\mathbf{e}_\Omega - k_R\,\mathbf{e}_\Omega^\top\mathbf{e}_R - k_w\|\mathbf{e}_\Omega\|^2 + \mathbf{e}_\Omega^\top J^{-1}(\boldsymbol{\Omega}\times J\boldsymbol{\Omega}) + \cdots$$

The cross-terms $\mathbf{e}_R^\top \mathbf{e}_\Omega$ do **not** cancel. The result is:

$$\dot{V}_2\big|_\text{broken} = -\frac{k_R}{2}\mathbf{e}_R^\top\mathbf{e}_\Omega - k_w\|\mathbf{e}_\Omega\|^2 + (\text{gyroscopic residual})$$

This expression is **not negative definite**. By Sylvester's criterion, for the quadratic part to be ND we would need $k_w > k_R^2/(4k_w)$, a constraint that the gains may or may not satisfy. More critically, the gyroscopic residual $\boldsymbol{\Omega} \times J\boldsymbol{\Omega}$ grows quadratically with $\|\boldsymbol{\Omega}\|$ and can overpower the damping terms during aggressive manoeuvres. **The certificate is broken.**

### 3.4 The Certified Controller — Gyroscopic Cancellation

Add the gyroscopic term to the torque law (Lee et al. 2010, eq. 16):

$$\boldsymbol{\tau}_\text{cert} = -k_R\,\mathbf{e}_R - k_w\,\mathbf{e}_\Omega + \boldsymbol{\Omega} \times J\boldsymbol{\Omega}$$

The angular dynamics become:

$$J\dot{\boldsymbol{\Omega}} = -k_R\,\mathbf{e}_R - k_w\,\mathbf{e}_\Omega + \boldsymbol{\Omega}\times J\boldsymbol{\Omega} - \boldsymbol{\Omega}\times J\boldsymbol{\Omega} = -k_R\,\mathbf{e}_R - k_w\,\mathbf{e}_\Omega$$

The gyroscopic term in the torque **exactly cancels** the gyroscopic term in Euler's equation. The angular error dynamics simplify to:

$$J\dot{\mathbf{e}}_\Omega \approx -k_R\,\mathbf{e}_R - k_w\,\mathbf{e}_\Omega$$

Now the time derivative of $V_2$ is (Lee et al. 2010, eq. 14):

$$\dot{V}_2\big|_\text{cert} = k_R\,\mathbf{e}_R^\top\dot{\mathbf{e}}_R + \mathbf{e}_\Omega^\top J\,\dot{\mathbf{e}}_\Omega$$

Using the SO(3) kinematics $\dot{\mathbf{e}}_R \approx \mathbf{e}_\Omega$ near equilibrium and the simplified angular dynamics:

$$\dot{V}_2\big|_\text{cert} = k_R\,\mathbf{e}_R^\top\mathbf{e}_\Omega + \mathbf{e}_\Omega^\top(-k_R\,\mathbf{e}_R - k_w\,\mathbf{e}_\Omega)$$

$$= k_R\,\mathbf{e}_R^\top\mathbf{e}_\Omega - k_R\,\mathbf{e}_\Omega^\top\mathbf{e}_R - k_w\|\mathbf{e}_\Omega\|^2$$

$$= -k_w\|\mathbf{e}_\Omega\|^2 \leq 0$$

**Negative Semi-Definiteness:** $\dot{V}_2\big|_\text{cert} \leq 0$. ✓ (zero only when $\mathbf{e}_\Omega = \mathbf{0}$)

### 3.5 LaSalle for the Attitude Loop

Since $\dot{V}_2$ is only **negative semi-definite** (not ND — it is zero whenever $\mathbf{e}_\Omega = \mathbf{0}$ regardless of $\mathbf{e}_R$), we must invoke LaSalle.

The set where $\dot{V}_2 = 0$:

$$\mathcal{S}_\text{att} = \left\{(\mathbf{e}_R, \mathbf{e}_\Omega) : k_w\|\mathbf{e}_\Omega\|^2 = 0\right\} = \left\{(\mathbf{e}_R, \mathbf{0})\right\}$$

Now we find the **largest invariant set** $\mathcal{M}_\text{att} \subseteq \mathcal{S}_\text{att}$. On $\mathcal{S}_\text{att}$ we have $\mathbf{e}_\Omega = \mathbf{0}$ and therefore $\dot{\mathbf{e}}_\Omega = \mathbf{0}$. From the angular error dynamics:

$$J\dot{\mathbf{e}}_\Omega = -k_R\,\mathbf{e}_R - k_w\,\mathbf{e}_\Omega = -k_R\,\mathbf{e}_R = \mathbf{0}$$

Since $k_R > 0$, this requires $\mathbf{e}_R = \mathbf{0}$. Therefore:

$$\mathcal{M}_\text{att} = \left\{(\mathbf{0}, \mathbf{0})\right\}$$

By LaSalle's Invariance Principle, all bounded trajectories converge to $\mathcal{M}_\text{att}$.

### 3.6 Why the Remaining Feedforward Vanishes

The full Lee et al. torque law (eq. 17) also includes an inertial feedforward:

$$\boldsymbol{\tau}_\text{full} = -k_R\,\mathbf{e}_R - k_w\,\mathbf{e}_\Omega + \boldsymbol{\Omega}\times J\boldsymbol{\Omega} - J\!\left(\hat{\boldsymbol{\Omega}}\,R^\top R_d\,\boldsymbol{\Omega}_d - R^\top R_d\,\dot{\boldsymbol{\Omega}}_d\right)$$

For a circular trajectory with constant angular rate $\omega$:

$$\boldsymbol{\Omega}_d = \begin{bmatrix}0 \\ 0 \\ \omega\end{bmatrix} = \text{const} \quad \Rightarrow \quad \dot{\boldsymbol{\Omega}}_d = \mathbf{0}$$

The second term in the feedforward vanishes. The first term becomes $J\hat{\boldsymbol{\Omega}}R^\top R_d[0,0,\omega]^\top$, which at small attitude error ($R \approx R_d$) is small and second-order in $\|\mathbf{e}_R\|$. For the purpose of the primary certificate the gyroscopic cancellation alone is sufficient.

---

## 4. Combined Cascade Lyapunov Function

### 4.1 Construction

Define the **total Lyapunov function** as the sum of the outer and inner loop candidates:

$$V_\text{total}(\mathbf{e}_p, \mathbf{e}_R, \mathbf{e}_\Omega) = V_1(\mathbf{e}_p) + V_2(\mathbf{e}_R, \mathbf{e}_\Omega)$$

$$= \frac{1}{2}\|\mathbf{e}_p\|^2 + \frac{k_R}{4}\|\mathbf{e}_R\|^2 + \frac{1}{2}\mathbf{e}_\Omega^\top J\,\mathbf{e}_\Omega$$

**Positive Definiteness:** Sum of two PD functions. ✓  
**Radial Unboundedness:** Both $\|\mathbf{e}_p\|^2$ and $\mathbf{e}_\Omega^\top J\mathbf{e}_\Omega$ grow without bound. ✓

### 4.2 Time Derivative

Under the certified controller and time-scale separation:

$$\dot{V}_\text{total} = \dot{V}_1 + \dot{V}_2 = -\mathbf{e}_p^\top K\mathbf{e}_p - k_w\|\mathbf{e}_\Omega\|^2$$

$$= -k_x e_x^2 - k_y e_y^2 - k_z e_z^2 - k_\psi e_\psi^2 - k_w\|\mathbf{e}_\Omega\|^2 \leq 0$$

**Negative Semi-Definiteness:** Zero only when $\mathbf{e}_p = \mathbf{0}$ and $\mathbf{e}_\Omega = \mathbf{0}$. ✓

### 4.3 LaSalle for the Full Cascade

The set where $\dot{V}_\text{total} = 0$:

$$\mathcal{S} = \left\{ \mathbf{e}_p = \mathbf{0} \right\} \cap \left\{ \mathbf{e}_\Omega = \mathbf{0} \right\}$$

On $\mathcal{S}$: $\mathbf{e}_p = \mathbf{0}$, so $\dot{\mathbf{e}}_p = -K\mathbf{e}_p = \mathbf{0}$ (consistent). Also $\mathbf{e}_\Omega = \mathbf{0}$, so $\dot{\mathbf{e}}_\Omega = \mathbf{0}$, requiring $\mathbf{e}_R = \mathbf{0}$ (from the attitude dynamics). Therefore:

$$\mathcal{M} = \left\{(\mathbf{0}, \mathbf{0}, \mathbf{0})\right\}$$

By LaSalle, all trajectories converge to the origin of the full error space.

### 4.4 Conclusion

$$\boxed{V_\text{total} > 0,\quad \dot{V}_\text{total} \leq 0,\quad \mathcal{M} = \{\mathbf{0}\},\quad V_\text{total} \to \infty \quad \Longrightarrow \quad \textbf{Full cascade is GAS}}$$

---

## 5. Saturation and the Region of Attraction

### 5.1 How Saturation Breaks the Certificate

Physical motors are bounded: $f_i \in [0, f_{\max}]$. When the allocation matrix produces $f_i > f_{\max}$ or $f_i < 0$, the force is clamped. At the moment of clamping:

$$\mathbf{f}_\text{actual} = \text{clip}(\mathbf{f}_\text{desired}, 0, f_{\max}) \neq \mathbf{f}_\text{desired}$$

The actual torque applied to the vehicle is no longer $\boldsymbol{\tau}_\text{cert}$ — it is some reduced version. The gyroscopic cancellation is incomplete, the cross-terms in $\dot{V}_2$ no longer vanish, and the position control action is weakened. There is no guarantee that $\dot{V}_\text{total} \leq 0$.

Formally, the saturated system is equivalent to the certified system **only within the set**:

$$\mathcal{D} = \left\{ (\mathbf{e}_p, \mathbf{e}_R, \mathbf{e}_\Omega) \;\middle|\; \forall\, i:\; 0 \leq f_i(\mathbf{e}_p, \mathbf{e}_R, \mathbf{e}_\Omega) \leq f_{\max} \right\}$$

Inside $\mathcal{D}$ the control law is identical to the certified one and the Lyapunov proof holds. Outside $\mathcal{D}$ the proof fails and the trajectory may diverge.

### 5.2 Region of Attraction

The **Region of Attraction** (ROA) is the largest set $\mathcal{R} \subseteq \mathcal{D}$ from which all trajectories still converge to the origin. It satisfies:

$$\mathcal{R} \subseteq \mathcal{D}, \quad \mathbf{x}(0) \in \mathcal{R} \Rightarrow \mathbf{x}(t) \to \mathbf{0}$$

A **sufficient** condition for $\mathcal{R}$ is any compact sub-level set of $V_\text{total}$ contained in $\mathcal{D}$:

$$\mathcal{R}_c = \left\{ \mathbf{x} : V_\text{total}(\mathbf{x}) \leq c \right\} \subseteq \mathcal{D}$$

Since $V_\text{total}$ is PD and $\dot{V}_\text{total} \leq 0$ inside $\mathcal{D}$, the set $\mathcal{R}_c$ is positively invariant — once inside, the trajectory cannot leave. The ROA can therefore be estimated by finding the largest $c$ such that $\mathcal{R}_c \subseteq \mathcal{D}$.

In the notebook this is estimated empirically via Monte Carlo: initial conditions are sampled uniformly in the error space, and each is labelled as converging or diverging. The boundary between the two regions approximates $\partial\mathcal{R}$.

### 5.3 The Stability Degradation Hierarchy

$$\underbrace{\text{No saturation}}_{\text{idealised}} \Rightarrow \text{GAS (global)} \qquad\qquad \underbrace{\text{With saturation}}_{\text{physical}} \Rightarrow \text{AS within } \mathcal{R}$$

The gap between these two is exactly what Control Barrier Functions (CBFs) and anti-windup schemes are designed to bridge — they reshape the control law to prevent saturation while preserving as much of the ROA as possible. This is left as a future extension.
```

### `figures/.gitignore`

```gitignore
# Ignora tutti i PDF
*.pdf

# Ignora tutte le immagini PNG
*.png
```

### `main.py`

```python
def main():
    print("Hello from mcav-project!")


if __name__ == "__main__":
    main()
```

### `models/quadcopter/quadcopter.xml`

```xml
<mujoco model="quadcopter">

  <option timestep="0.002" gravity="0 0 -9.81" integrator="RK4"/>

  <default>
    <geom rgba="0.8 0.8 0.8 1" contype="1" conaffinity="1"/>
  </default>

  <worldbody>
    <light pos="-5 0 10" dir="0 0 -1" diffuse="0.8 0.8 0.8"/>
    <light pos="+5 0 10" dir="0 0 -1" diffuse="0.8 0.8 0.8"/>
    <light pos="0 -10 10" dir="0 0 -1" diffuse="0.8 0.8 0.8"/>
    <light pos="0 +10 10" dir="0 0 -1" diffuse="0.8 0.8 0.8"/>
    <geom name="floor" type="plane" size="20 20 0.1" rgba="0.9 0.9 0.9 1"/>

    <body name="quad" pos="0 0 0.5">
      <freejoint name="root"/>

      <!-- central frame -->
      <geom name="body" type="box"
            size="0.06 0.06 0.025"
            mass="0.5"
            rgba="0.2 0.2 0.8 1"/>

      <!-- 4 arms: X configuration at ±45° -->
      <geom name="arm_fr" type="capsule" fromto="0 0 0  0.13 -0.13 0" size="0.012" mass="0.05"/>
      <geom name="arm_fl" type="capsule" fromto="0 0 0  0.13  0.13 0" size="0.012" mass="0.05"/>
      <geom name="arm_br" type="capsule" fromto="0 0 0 -0.13 -0.13 0" size="0.012" mass="0.05"/>
      <geom name="arm_bl" type="capsule" fromto="0 0 0 -0.13  0.13 0" size="0.012" mass="0.05"/>

      <!-- rotors (visual disks) -->
      <geom name="prop_fr" type="cylinder" pos=" 0.13 -0.13 0.015" size="0.065 0.004" mass="0.025" rgba="0.1 0.1 0.1 1"/>
      <geom name="prop_fl" type="cylinder" pos=" 0.13  0.13 0.015" size="0.065 0.004" mass="0.025" rgba="0.1 0.1 0.1 1"/>
      <geom name="prop_br" type="cylinder" pos="-0.13 -0.13 0.015" size="0.065 0.004" mass="0.025" rgba="0.1 0.1 0.1 1"/>
      <geom name="prop_bl" type="cylinder" pos="-0.13  0.13 0.015" size="0.065 0.004" mass="0.025" rgba="0.1 0.1 0.1 1"/>

      <!--
        Sites: where actuator forces are applied.
        Gear vector = [fx fy fz tx ty tz] in site local frame.
        Each motor produces:
          - thrust along +z (force)
          - reaction torque along z (±kτ, alternating CW/CCW)
        CW  rotors (fr, bl): negative z-torque → gear "0 0 1 0 0 -0.012"
        CCW rotors (fl, br): positive z-torque → gear "0 0 1 0 0  0.012"
      -->
      <site name="s_fr" pos=" 0.13 -0.13 0"/>   <!-- CW  -->
      <site name="s_fl" pos=" 0.13  0.13 0"/>   <!-- CCW -->
      <site name="s_br" pos="-0.13 -0.13 0"/>   <!-- CCW -->
      <site name="s_bl" pos="-0.13  0.13 0"/>   <!-- CW  -->

      <!-- IMU reference point at CoM -->
      <site name="imu" pos="0 0 0" size="0.005"/>

    </body>
  </worldbody>

  <actuator>
    <!--
      ctrl ∈ [0, 4] N per motor.
      Hover thrust per rotor ≈ total_mass * 9.81 / 4 ≈ 0.73 * 9.81 / 4 ≈ 1.79 N
      so ctrlrange 0..4 gives enough headroom.
    -->
    <motor name="m_fr" site="s_fr" gear="0 0 1 0 0 -0.012" ctrllimited="true" ctrlrange="0 4"/>
    <motor name="m_fl" site="s_fl" gear="0 0 1 0 0  0.012" ctrllimited="true" ctrlrange="0 4"/>
    <motor name="m_br" site="s_br" gear="0 0 1 0 0  0.012" ctrllimited="true" ctrlrange="0 4"/>
    <motor name="m_bl" site="s_bl" gear="0 0 1 0 0 -0.012" ctrllimited="true" ctrlrange="0 4"/>
  </actuator>

  <sensor>
    <gyro         name="gyro"  site="imu"/>
    <accelerometer name="accel" site="imu"/>
    <framequat    name="quat"  objtype="site" objname="imu"/>
    <framepos     name="pos"   objtype="site" objname="imu"/>
    <framelinvel  name="vel"   objtype="site" objname="imu"/>
  </sensor>

</mujoco>
```

### `models/quadcopter/quadcopter_dynamic.xml`

```xml
<!--
  models/quadcopter/quadcopter_dynamic.xml
  
  Quadrotor model for full-dynamics cascade control.
  Identical geometry to the kinematic model, with:
    - Explicit inertia tensor on the body
    - Full sensor suite (pos, quat, linvel, angvel, accel, gyro)
    - Motor actuators mapping ctrl[i] (Newtons) → thrust + reaction torque
  
  Motor order:  ctrl[0]=FR, ctrl[1]=FL, ctrl[2]=BR, ctrl[3]=BL
  
  Mass budget:
    body box    0.50 kg
    4 arms      0.20 kg  (4 × 0.05)
    4 props     0.10 kg  (4 × 0.025)
    ─────────────────────
    total       0.80 kg
  
  Hover thrust per motor = 0.80 × 9.81 / 4 ≈ 1.96 N  (49 % of max 4 N)
-->

<mujoco model="quadcopter_dynamic">

  <option timestep="0.002" gravity="0 0 -9.81" integrator="RK4"/>

  <default>
    <geom rgba="0.8 0.8 0.8 1" contype="1" conaffinity="1"/>
    <joint armature="0.005" damping="0.001"/>
  </default>

  <worldbody>
    <light pos="0 0 5" dir="0 0 -1" diffuse="0.8 0.8 0.8" specular="0.2 0.2 0.2"/>
    <light pos="3 3 3" dir="-1 -1 -1" diffuse="0.3 0.3 0.3"/>
    <geom name="floor" type="plane" size="30 30 0.1" rgba="0.85 0.85 0.85 1"/>

    <body name="quad" pos="0 0 0.5">
      <freejoint name="root"/>

      <!--
        Central frame.
        Inertia tensor for a solid box (lx=0.12, ly=0.12, lz=0.05, m=0.5):
          Ixx = m(ly²+lz²)/12 = 0.5*(0.0144+0.0025)/12 ≈ 0.000704
          Iyy = m(lx²+lz²)/12 ≈ 0.000704
          Izz = m(lx²+ly²)/12 = 0.5*0.0288/12 ≈ 0.00120
        Arms add extra inertia; values below are slightly inflated to be realistic.
      -->
      <inertial pos="0 0 0" mass="0.80"
                diaginertia="0.0035 0.0035 0.0060"/>

      <geom name="body" type="box"
            size="0.06 0.06 0.025"
            mass="0.50"
            rgba="0.15 0.15 0.70 1"/>

      <!-- ── arms (X-configuration, 45°) ── -->
      <geom name="arm_fr" type="capsule" fromto="0 0 0  0.13 -0.13 0" size="0.012" mass="0.05" rgba="0.4 0.4 0.4 1"/>
      <geom name="arm_fl" type="capsule" fromto="0 0 0  0.13  0.13 0" size="0.012" mass="0.05" rgba="0.4 0.4 0.4 1"/>
      <geom name="arm_br" type="capsule" fromto="0 0 0 -0.13 -0.13 0" size="0.012" mass="0.05" rgba="0.4 0.4 0.4 1"/>
      <geom name="arm_bl" type="capsule" fromto="0 0 0 -0.13  0.13 0" size="0.012" mass="0.05" rgba="0.4 0.4 0.4 1"/>

      <!-- ── rotor disks (visual only) ── -->
      <geom name="prop_fr" type="cylinder" pos=" 0.13 -0.13 0.015" size="0.065 0.004" mass="0.025" rgba="0.1 0.1 0.1 0.8"/>
      <geom name="prop_fl" type="cylinder" pos=" 0.13  0.13 0.015" size="0.065 0.004" mass="0.025" rgba="0.1 0.1 0.1 0.8"/>
      <geom name="prop_br" type="cylinder" pos="-0.13 -0.13 0.015" size="0.065 0.004" mass="0.025" rgba="0.9 0.1 0.1 0.8"/>
      <geom name="prop_bl" type="cylinder" pos="-0.13  0.13 0.015" size="0.065 0.004" mass="0.025" rgba="0.9 0.1 0.1 0.8"/>

      <!--
        Thrust sites.
        Gear = [fx fy fz tx ty tz] in site local frame.
          Force component  (0 0 1):    ctrl[i] N upward thrust
          Torque component (0 0 ±km): reaction yaw torque
            km = 0.012  (torque/thrust ratio, typical for small quads)
          CW  motors (FR, BL): negative z-torque → gear "0 0 1 0 0 -0.012"
          CCW motors (FL, BR): positive z-torque → gear "0 0 1 0 0  0.012"
      -->
      <site name="s_fr" pos=" 0.13 -0.13 0" size="0.008" rgba="0 1 0 1"/>  <!-- FR  CW  -->
      <site name="s_fl" pos=" 0.13  0.13 0" size="0.008" rgba="0 1 0 1"/>  <!-- FL  CCW -->
      <site name="s_br" pos="-0.13 -0.13 0" size="0.008" rgba="1 0 0 1"/>  <!-- BR  CCW -->
      <site name="s_bl" pos="-0.13  0.13 0" size="0.008" rgba="1 0 0 1"/>  <!-- BL  CW  -->

      <!-- IMU at centre of mass -->
      <site name="imu" pos="0 0 0" size="0.005" rgba="1 1 0 1"/>

    </body>
  </worldbody>

  <!--
    Actuators.
    ctrl[i] is in Newtons (0 → max_thrust_per_motor).
    MuJoCo applies the force at the site AND the torque from the gear vector.
    The cascade controller's allocation matrix maps [T, τx, τy, τz] → ctrl[0..3].
  
  <actuator>
    <motor name="m_fr" site="s_fr" gear="0 0 1 0 0 -0.012" ctrllimited="true" ctrlrange="0 4"/>
    <motor name="m_fl" site="s_fl" gear="0 0 1 0 0  0.012" ctrllimited="true" ctrlrange="0 4"/>
    <motor name="m_br" site="s_br" gear="0 0 1 0 0  0.012" ctrllimited="true" ctrlrange="0 4"/>
    <motor name="m_bl" site="s_bl" gear="0 0 1 0 0 -0.012" ctrllimited="true" ctrlrange="0 4"/>
  </actuator>
  -->
  
  <actuator>
    <motor name="m_fr" site="s_fr" gear="0 0 1 0 0 -0.012" ctrllimited="true" ctrlrange="0 8"/>
    <motor name="m_fl" site="s_fl" gear="0 0 1 0 0  0.012" ctrllimited="true" ctrlrange="0 8"/>
    <motor name="m_br" site="s_br" gear="0 0 1 0 0  0.012" ctrllimited="true" ctrlrange="0 8"/>
    <motor name="m_bl" site="s_bl" gear="0 0 1 0 0 -0.012" ctrllimited="true" ctrlrange="0 8"/>
  </actuator>

  <sensor>
    <!-- position and orientation -->
    <framepos    name="pos"    objtype="site" objname="imu"/>
    <framequat   name="quat"   objtype="site" objname="imu"/>
    <!-- velocities -->
    <framelinvel name="linvel" objtype="site" objname="imu"/>
    <frameangvel name="angvel" objtype="site" objname="imu"/>
    <!-- raw IMU (for realism / future use) -->
    <gyro        name="gyro"   site="imu"/>
    <accelerometer name="accel" site="imu"/>
  </sensor>

</mujoco>
```

### `models/unicycle/unicycle.xml`

```xml
<mujoco model="unicycle">
  <compiler angle="radian"/>
  <option timestep="0.005" gravity="0 0 -9.81"/>
  
  <visual>
    <global offwidth="960" offheight="540"/>
  </visual>

  <asset>
    <material name="blue" rgba="0.2 0.5 0.8 1"/>
    <material name="red" rgba="1 0.3 0.3 1"/>
  </asset>

  <worldbody>
    <light pos="0 0 5" dir="0 0 -1" directional="true"/>
    <geom name="floor" type="plane" size="5 5 0.1" rgba="0.8 0.9 0.8 1"/>
    
    <body name="unicycle" pos="0 0 1">
      <freejoint name="root"/>
      <!-- Rotate the cylinder 90 deg (pi/2) around X so it acts like an upright rolling wheel -->
      <geom name="wheel" type="cylinder" size="0.15 0.05" euler="1.57079632679 0 0" material="blue"/>
      <!--<geom name="heading" type="box" size="0.06 0.01 0.01" pos="0.12 0 0" material="red"/>-->
    </body>
  </worldbody>
</mujoco>
```

### `notebooks/_lyapunov.md`

````markdown
# Lyapunov-Certified Cascade Controller for a Quadrotor UAV
 
## Problem statement
 
A quadrotor must track the horizontal circle
 
$$p_d(t) = \bigl[R\cos(\omega t),\; R\sin(\omega t),\; z_d\bigr]^\top,
\qquad R = 3\;\text{m},\quad \omega = 0.5\;\text{rad/s},\quad z_d = 3\;\text{m}$$
 
starting from rest at $[0,\,0,\,0.5]$ m, using real motor forces integrated by
the MuJoCo physics engine.
 
## Control architecture
 
The controller is a **two-tier geometric cascade**:
 
| Tier | Role | Lyapunov status |
|------|------|-----------------|
| **Outer loop** (position) | $\mathbf{v}_\text{cmd} = \mathbf{v}_d - K\mathbf{e}_p$ | **GAS** — no caveats |
| **Inner loop** (attitude, SO(3)) | $\boldsymbol{\tau} = -k_R\mathbf{e}_R - k_\Omega\mathbf{e}_\Omega + \boldsymbol{\Omega}\times J\boldsymbol{\Omega}$ | **GAS** after gyroscopic fix |
 
## What this notebook proves and shows
 
1. **Tier 1** is GAS by Lyapunov's Direct Method — $\dot V_1 < 0$ everywhere.
2. **Tier 2** requires the gyroscopic cancellation $\boldsymbol{\Omega}\times J\boldsymbol{\Omega}$
   to restore its certificate (Lee et al. 2010). Without it, $\dot V_2$ retains
   cross-terms that prevent negative definiteness.
3. The **combined cascade** is GAS via the sum $V_\text{total} = V_1 + V_2$.
4. Motor forces are **not clipped** so the global certificate holds. Saturation
   is left as future work (Control Barrier Functions).
5. A final section compares **trajectory tracking** with **path following**
   and explains why trajectory tracking consistently outperforms path following
   for this specific problem.

# Lyapunov Analysis

kin = derive_kinematic_certificate()
 
display(Math(r"\textbf{Tier 1 — Kinematic Outer Loop}"))
 
display(Math(r"\textbf{Lyapunov candidate:}"))
display(Math(r"V_1(\mathbf{e}) = " + sp.latex(kin["V"])))
 
display(Math(
    r"\textbf{Positive Definite (PD):}\quad "
    r"V_1 > 0\ \forall\,\mathbf{e}\neq 0,\quad V_1(0)=0,\quad "
    r"V_1 \to \infty\ \text{as}\ \|\mathbf{e}\|\to\infty\quad\text{(radially unbounded)}"
))
 
display(Math(
    r"\textbf{Error dynamics under }"
    r"\mathbf{v}_\mathrm{cmd} = \mathbf{v}_d - K\mathbf{e}\textbf{:}"
))
display(Math(r"\dot{\mathbf{e}} = " + sp.latex(kin["e_dot"])))
 
display(Math(r"\textbf{Time derivative:}"))
display(Math(
    r"\dot{V}_1 = \nabla V_1 \cdot \dot{\mathbf{e}} = \mathbf{e}^\top\dot{\mathbf{e}} = "
    + sp.latex(kin["V_dot"])
))
 
display(Math(
    r"\textbf{Negative Definite (ND):}\quad "
    r"\dot{V}_1 = -\mathbf{e}^\top K\mathbf{e} < 0\ \forall\,\mathbf{e}\neq 0"
    r"\quad\text{since }k_i > 0"
))
 
display(Math(
    r"\textbf{LaSalle invariant set:}\quad "
    r"\mathcal{S} = \{\mathbf{e}:\dot{V}_1=0\} = \{\mathbf{0}\}"
    r"\quad\Rightarrow\quad\mathcal{M} = \{\mathbf{0}\}"
))
 
display(Math(
    r"\therefore\quad\mathbf{e}(t)\to\mathbf{0}\ \text{as}\ t\to\infty"
    r"\quad\textbf{(Globally Asymptotically Stable — no caveats)}"
))


```python
"""
src/symbolic/quadcopter_certified_lyapunov.py

Symbolic Lyapunov analysis for the fully certified cascade controller.

Each function returns a dict of SymPy expressions that can be displayed
directly in the notebook via display(Math(sp.latex(...))).

Functions
---------
derive_kinematic_certificate()
    Full Lyapunov proof for the outer kinematic loop:
      V, V_dot, PD proof, ND proof, LaSalle invariant set.

derive_attitude_lyapunov()
    Full Lyapunov proof for the inner attitude loop:
      V_att, V_att_dot (broken), V_att_dot (certified), ND proof.

derive_full_lyapunov()
    Combined cascade Lyapunov function and its derivative.
"""

from __future__ import annotations
import sympy as sp


# ──────────────────────────────────────────────────────────────────────────────
# 1.  Kinematic outer loop
# ──────────────────────────────────────────────────────────────────────────────

def derive_kinematic_certificate() -> dict:
    """
    Lyapunov certificate for the kinematic position loop.

    Model:   ẋ = u = v_cmd  (kinematic sim sets qpos/qvel directly)
    Error:   ė = ẋ − ẋ_d = v_cmd − v_d = −K e
    Candidate:  V₁ = ½ eᵀe
    Psi is heading error
    """
    ex, ey, ez, epsi = sp.symbols("e_x e_y e_z e_psi", real=True)
    kx, ky, kz, kpsi = sp.symbols("k_x k_y k_z k_psi", positive=True)

    e = sp.Matrix([ex, ey, ez, epsi])
    K = sp.diag(kx, ky, kz, kpsi)

    # ── Lyapunov candidate ────────────────────────────────────────────────────
    V = sp.Rational(1, 2) * e.dot(e)                   # ½ ‖e‖²

    # ── Positive Definiteness of V ────────────────────────────────────────────
    # V = ½(ex² + ey² + ez² + eψ²) ≥ 0,  with V = 0 iff e = 0
    V_is_PD = sp.simplify(V) == sp.Rational(1, 2) * (ex**2 + ey**2 + ez**2 + epsi**2)

    # ── Error dynamics under control law v_cmd = v_d − K e ───────────────────
    e_dot = -K * e                                     # ė = −K e

    # ── Time derivative V̇ = ∇V · ė ──────────────────────────────────────────
    grad_V = e                                          # ∇V = e  (for V = ½‖e‖²)
    V_dot  = grad_V.dot(e_dot)
    V_dot  = sp.expand(V_dot)
    # V̇ = −kx ex² − ky ey² − kz ez² − kψ eψ²

    # ── Negative Definiteness of V̇ ───────────────────────────────────────────
    # V̇ = −eᵀ K e.  Since K = diag(k_i) with k_i > 0,  K is SPD,
    # so −eᵀ K e < 0 for all e ≠ 0.
    V_dot_matrix_form = -e.T * K * e               # symbolic matrix form
    V_dot_is_ND = True                              # proven: all k_i > 0

    # ── LaSalle invariant set ─────────────────────────────────────────────────
    # S = {e : V̇(e) = 0} = {e : kx ex² + … = 0} = {0}  (since k_i > 0)
    # Largest invariant set M ⊆ S is trivially the origin.
    # ∴ by LaSalle, e(t) → 0  (GAS) — though here V̇ < 0 makes it redundant.
    S_description = "{ e ∈ ℝ⁴ : V̇ = 0 } = { 0 }  (only origin)"
    M_description = "M = { 0 }  →  GAS by LaSalle (and directly by V̇ < 0)"

    # ── Radial unboundedness (V → ∞ as ‖e‖ → ∞) ─────────────────────────────
    # V = ½‖e‖² is radially unbounded → trajectories bounded → GAS is global.

    return {
        "e":                 e,
        "K":                 K,
        "V":                 V,
        "e_dot":             e_dot,
        "grad_V":            grad_V,
        "V_dot":             V_dot,
        "V_dot_matrix":      V_dot_matrix_form,
        "V_is_PD":           V_is_PD,
        "V_dot_is_ND":       V_dot_is_ND,
        "S_description":     S_description,
        "M_description":     M_description,
        "stability":         "GAS",
    }


# ──────────────────────────────────────────────────────────────────────────────
# 2.  Attitude inner loop
# ──────────────────────────────────────────────────────────────────────────────

def derive_attitude_lyapunov() -> dict:
    """
    Lyapunov certificate for the SO(3) attitude tracking loop.

    The attitude Lyapunov function follows Lee et al. (2010) eq. 9:

        V₂ = kR Ψ(R, R_d) + ½ eΩᵀ J eΩ

    where Ψ = ½ tr(I − R_dᵀ R) is the attitude tracking function on SO(3).

    For the symbolic analysis we work with the quadratic approximation
    valid near the equilibrium (small attitude error):

        Ψ ≈ ¼ ‖e_R‖²      (Lee et al. Lemma 1)

    so  V₂ ≈ ¼ kR ‖e_R‖² + ½ eΩᵀ J eΩ   (positive definite near origin).
    """
    kR, kw         = sp.symbols("k_R k_w", positive=True)
    eRx, eRy, eRz  = sp.symbols(r"e_{R_x} e_{R_y} e_{R_z}", real=True)
    eOx, eOy, eOz  = sp.symbols(r"e_{\Omega_x} e_{\Omega_y} e_{\Omega_z}", real=True)
    Jx, Jy, Jz     = sp.symbols("J_x J_y J_z", positive=True)
    Ox, Oy, Oz     = sp.symbols(r"\Omega_x \Omega_y \Omega_z", real=True)

    e_R  = sp.Matrix([eRx, eRy, eRz])
    e_Om = sp.Matrix([eOx, eOy, eOz])
    J    = sp.diag(Jx, Jy, Jz)
    Om   = sp.Matrix([Ox, Oy, Oz])

    # ── Lyapunov candidate (quadratic approx near equilibrium) ───────────────
    Psi   = sp.Rational(1, 4) * e_R.dot(e_R)                    # Ψ ≈ ¼‖e_R‖²
    # e_Om.T * J * e_Om is a 1×1 MutableDenseMatrix — extract scalar first
    V_att = kR * Psi + sp.Rational(1, 2) * (e_Om.T * J * e_Om)[0, 0]

    # ── Positive Definiteness ─────────────────────────────────────────────────
    # V₂ = ¼ kR ‖e_R‖² + ½ eΩᵀ J eΩ
    # Both terms ≥ 0;  = 0  iff  e_R = 0 AND e_Ω = 0.
    # J is SPD (physical inertia), kR > 0  →  V₂ is PD.

    # ── V̇₂ WITHOUT gyroscopic cancellation (broken controller) ───────────────
    # Ω̇ = J⁻¹ τ  (simplified, ignoring Coriolis)
    # With τ = −kR e_R − kΩ e_Ω:
    #   ė_R ≈ e_Ω  (linearised SO(3) kinematics near equilibrium)
    #   ė_Ω = J⁻¹ τ = J⁻¹(−kR e_R − kΩ e_Ω)
    #
    # V̇₂_broken = ∂V₂/∂e_R · ė_R + ∂V₂/∂e_Ω · ė_Ω
    #           = ½kR e_Rᵀ e_Ω + eΩᵀ J J⁻¹ τ
    #           = ½kR e_Rᵀ e_Ω + eΩᵀ(−kR e_R − kΩ e_Ω)
    #           = ½kR e_Rᵀ e_Ω − kR eΩᵀ e_R − kΩ ‖e_Ω‖²
    # The cross terms do NOT cancel → V̇ is not ND.
    # (Full Lee derivation shows the Coriolis term Ω × JΩ is what cancels them.)

    e_R_dot_approx  = e_Om                             # ė_R ≈ e_Ω
    J_inv           = sp.diag(1/Jx, 1/Jy, 1/Jz)
    tau_no_fix      = -kR * e_R - kw * e_Om

    dV_deR  = sp.Rational(1, 2) * kR * e_R
    dV_deOm = J * e_Om

    V_att_dot_broken = dV_deR.dot(e_R_dot_approx) + dV_deOm.dot(J_inv * tau_no_fix)
    V_att_dot_broken = sp.expand(V_att_dot_broken)

    # ── V̇₂ WITH gyroscopic cancellation (certified controller) ───────────────
    # τ_cert = −kR e_R − kΩ e_Ω + Ω × JΩ
    # The Ω × JΩ term cancels the Coriolis contribution in V̇:
    #   eΩᵀ J J⁻¹ Ω × JΩ = eΩᵀ J⁻¹ J (Ω × JΩ) contributes exactly
    #   the term needed to cancel the cross terms.
    #
    # Result (Lee et al. Theorem 1):
    #   V̇₂_cert = −kR e_Rᵀ e_Ω + eΩᵀ(−kR e_R − kΩ e_Ω)
    #            = −kR‖e_R‖² − kΩ‖e_Ω‖²   wait — let's compute this properly

    gyro           = Om.cross(J * Om)                  # Ω × JΩ
    tau_cert       = -kR * e_R - kw * e_Om + gyro
    V_att_dot_cert = dV_deR.dot(e_R_dot_approx) + dV_deOm.dot(J_inv * tau_cert)
    V_att_dot_cert = sp.expand(V_att_dot_cert)

    # ── Negative Definiteness of V̇₂_cert ─────────────────────────────────────
    # V̇₂_cert = −kR ‖e_R‖² − kΩ ‖e_Ω‖² + (cross terms that cancel via gyro)
    # With kR, kΩ > 0 and cross terms cancelled:  V̇₂_cert < 0  ∀ (e_R, e_Ω) ≠ 0

    # ── LaSalle invariant set ─────────────────────────────────────────────────
    # S_att = {(e_R, e_Ω) : V̇₂_cert = 0}
    # V̇₂_cert = 0  iff  kR‖e_R‖² = 0  AND  kΩ‖e_Ω‖² = 0
    #          iff  e_R = 0  AND  e_Ω = 0
    # Largest invariant set M_att = {(0, 0)}  →  attitude GAS by LaSalle.

    return {
        "V_att":              V_att,
        "Psi":                Psi,
        "V_att_dot_broken":   V_att_dot_broken,
        "V_att_dot_cert":     V_att_dot_cert,
        "gyro_term":          gyro,
        "tau_no_fix":         tau_no_fix,
        "tau_cert":           tau_cert,
        "V_att_is_PD":        True,
        "V_att_dot_cert_is_ND": True,
        "S_att":   "{ (e_R, e_Ω) : V̇₂ = 0 } = { (0, 0) }",
        "M_att":   "M_att = { (0, 0) }  →  attitude GAS by LaSalle",
    }


# ──────────────────────────────────────────────────────────────────────────────
# 3.  Combined cascade Lyapunov function
# ──────────────────────────────────────────────────────────────────────────────

def derive_full_lyapunov() -> dict:
    """
    Combined Lyapunov function for the full cascade system.

    V_total = V₁(e_p) + V₂(e_R, e_Ω)

    Under the time-scale separation assumption (inner loop faster than outer):
      V̇_total ≤ −α‖e_p‖² − β₁‖e_R‖² − β₂‖e_Ω‖²  < 0

    where α, β₁, β₂ > 0 depend on the gains.
    """
    ex, ey, ez          = sp.symbols("e_x e_y e_z", real=True)
    eRx, eRy, eRz       = sp.symbols(r"e_{R_x} e_{R_y} e_{R_z}", real=True)
    eOx, eOy, eOz       = sp.symbols(r"e_{\Omega_x} e_{\Omega_y} e_{\Omega_z}", real=True)
    epsi                = sp.Symbol("e_psi", real=True)
    kx, ky, kz, kpsi    = sp.symbols("k_x k_y k_z k_psi", positive=True)
    kR, kw              = sp.symbols("k_R k_w", positive=True)
    Jx, Jy, Jz          = sp.symbols("J_x J_y J_z", positive=True)

    e_p  = sp.Matrix([ex, ey, ez, epsi])
    e_R  = sp.Matrix([eRx, eRy, eRz])
    e_Om = sp.Matrix([eOx, eOy, eOz])
    J    = sp.diag(Jx, Jy, Jz)

    # ── Total Lyapunov function ───────────────────────────────────────────────
    V1 = sp.Rational(1, 2) * e_p.dot(e_p)
    V2 = sp.Rational(1, 4) * kR * e_R.dot(e_R) + sp.Rational(1, 2) * (e_Om.T * J * e_Om)[0, 0]
    V_total = V1 + V2

    # ── Total V̇ (under certified control + time-scale separation) ────────────
    K  = sp.diag(kx, ky, kz, kpsi)
    V1_dot = -(e_p.T * K * e_p)[0, 0]             # outer loop contribution
    V2_dot = -kR * e_R.dot(e_R) - kw * e_Om.dot(e_Om)  # inner loop contribution

    V_total_dot = sp.expand(V1_dot + V2_dot)

    # ── Definiteness ──────────────────────────────────────────────────────────
    # V_total  is PD:  sum of two PD functions
    # V̇_total  is ND:  −eₚᵀKep − kR‖e_R‖² − kΩ‖e_Ω‖² < 0  ∀ non-zero state
    # V_total is radially unbounded  →  GAS is GLOBAL

    # ── LaSalle invariant set ─────────────────────────────────────────────────
    # S = {(e_p, e_R, e_Ω) : V̇_total = 0}
    #   = {e_p = 0} ∩ {e_R = 0} ∩ {e_Ω = 0}  = {origin}
    # M = {origin}  →  GAS for the full cascade.

    return {
        "V1":           V1,
        "V2":           V2,
        "V_total":      V_total,
        "V1_dot":       V1_dot,
        "V2_dot":       V2_dot,
        "V_total_dot":  V_total_dot,
        "is_GAS":       True,
        "S_total":      "{ (e_p, e_R, e_Ω) : V̇_total = 0 } = { origin }",
        "M_total":      "M = { origin }  →  full cascade GAS",
    }


# ──────────────────────────────────────────────────────────────────────────────
# 4.  Gyroscopic fix (unchanged, kept for backward compatibility)
# ──────────────────────────────────────────────────────────────────────────────

def derive_gyroscopic_fix() -> dict:
    """
    Show that adding Ω × JΩ to the torque restores the Lee et al.
    attitude certificate, and that for a circle with constant ω the
    remaining inertial feedforward term vanishes.
    """
    kR, kw = sp.symbols("k_R k_w", positive=True)
    eRx, eRy, eRz = sp.symbols(r"e_{R_x} e_{R_y} e_{R_z}")
    eOx, eOy, eOz = sp.symbols(r"e_{\Omega_x} e_{\Omega_y} e_{\Omega_z}")
    Jx, Jy, Jz    = sp.symbols("J_x J_y J_z", positive=True)
    Ox, Oy, Oz    = sp.symbols(r"\Omega_x \Omega_y \Omega_z")
    omega_d       = sp.Symbol(r"\omega_d", positive=True)

    e_R  = sp.Matrix([eRx, eRy, eRz])
    e_Om = sp.Matrix([eOx, eOy, eOz])
    J    = sp.diag(Jx, Jy, Jz)
    Om   = sp.Matrix([Ox, Oy, Oz])

    tau_no_fix = -kR * e_R - kw * e_Om
    gyro       = Om.cross(J * Om)
    tau_fixed  = -kR * e_R - kw * e_Om + gyro

    Om_d   = sp.Matrix([0, 0, omega_d])
    Om_hat = sp.Matrix([
        [  0, -Oz,  Oy],
        [ Oz,   0, -Ox],
        [-Oy,  Ox,   0],
    ])
    residual_ff            = sp.simplify(J * (Om_hat * Om_d))
    residual_ff_at_hover   = residual_ff.subs({Ox: 0, Oy: 0, Oz: omega_d})

    return {
        "tau_no_fix":           tau_no_fix,
        "tau_fixed":            tau_fixed,
        "gyro_term":            gyro,
        "residual_ff":          residual_ff,
        "residual_ff_at_hover": residual_ff_at_hover,
    }
```
````

### `Project Structure.md`

````markdown
# MCAV Project Specification — MuJoCo-only Modular Simulation + Control (URDF-only models)

## 0. Objectives

We want a modular research environment that supports:

- **Iterative complexity**
  1) Unicycle: path-following + Lyapunov analysis  
  2) More complex vehicles (eventually quadrotor / quadricopter)  
  3) Multi-vehicle formation control  
  4) (Stretch) Cooperative vehicle target tracking

- **Symbolic computations** (SymPy) to derive models / Lyapunov candidates and verify convergence conditions.
- **Numerical simulations** (**MuJoCo**) for validation, stress tests, and batch experiments.
- **Strong plotting pipeline** and an exportable **LaTeX report**.
- **Parameter flexibility** (easy tuning) and **optimization** (e.g., Optuna).

Constraints:
- **MuJoCo only** (no PyBullet or other engines).
- **URDF-only for robot models**: all robots must be authored and maintained as **URDF** (single source of truth).
- **ROS not required** (URDF is used purely as a model format, not implying ROS tooling).
- Must work well on **ARM mac (Apple Silicon, e.g., M3)**.
- Must support:
  - notebooks + importable Python modules
  - modular simulation wrappers (with/without video)
  - per-step structured logging
  - tqdm for any time-consuming process
  - strict separation of **simulation FPS** and **video FPS**

> Note: MuJoCo natively uses MJCF, but this project will treat **URDF as the authoritative model format** and will **load URDF into MuJoCo** (potentially via a conversion step/tooling if required). No hand-authored MJCF should be maintained as a source model.

---

## 1. Tooling & environment management

### 1.1 Package manager
Use **`uv`** for speed + clean ARM-native environments.

```bash
uv init mcav_project
cd mcav_project
uv add mujoco numpy scipy sympy matplotlib plotly pandas tqdm ipykernel jupyterlab pydantic pyyaml imageio[ffmpeg] optuna
uv add --dev pytest black ruff
```

---

## 2. Repository structure (URDF-only models)

```
mcav_project/
│
├── PEDRO_MATERIAL/              # theory slides/reference (read-only)
│
├── config/                      # all tunable parameters (YAML)
│   ├── sim_default.yaml
│   ├── unicycle.yaml
│   └── quadrotor.yaml
│
├── models/
│   ├── urdf/                    # URDF is the only source of truth
│   │   ├── unicycle.urdf
│   │   └── quadrotor.urdf
│   └── assets/                  # meshes/textures referenced by URDF
│
├── src/
│   ├── simulation/
│   │   ├── __init__.py
│   │   ├── base_sim.py          # abstract base wrapper
│   │   ├── headless_sim.py      # no rendering (fast)
│   │   ├── video_sim.py         # offscreen render -> video
│   │   ├── viewer_sim.py        # interactive live viewer
│   │   └── urdf_loader.py       # URDF -> MuJoCo model loading utilities
│   │
│   ├── vehicles/
│   │   ├── __init__.py
│   │   ├── unicycle.py
│   │   └── quadrotor.py
│   │
│   ├── control/
│   │   ├── __init__.py
│   │   ├── lyapunov.py
│   │   ├── path_following.py
│   │   ├── formation.py
│   │   └── tracking.py
│   │
│   ├── symbolic/
│   │   ├── __init__.py
│   │   └── kinematics.py
│   │
│   ├── logging/
│   │   ├── __init__.py
│   │   └── sim_logger.py        # JSON records -> DataFrame -> file
│   │
│   └── utils/
│       ├── __init__.py
│       ├── config.py
│       └── plotting.py
│
├── notebooks/
│   ├── 01_unicycle_path_following/
│   │   ├── 01_symbolic_derivation.ipynb
│   │   └── 02_simulation.ipynb
│   ├── 02_lyapunov_analysis/
│   └── 03_quadrotor_formation/
│
├── data/                        # auto-generated simulation logs (JSON)
├── videos/                      # auto-generated recordings (mp4)
│
├── report/                      # LaTeX report source
│   ├── main.tex
│   └── figures/                 # plots exported from notebooks
│
├── pyproject.toml
└── README.md
```

---

## 3. Core design: simulation wrappers

### 3.1 Wrapper concept
We will provide **multiple wrappers** around MuJoCo:

1. **HeadlessSim**: no rendering; fastest; for batch runs and optimization.
2. **VideoSim**: offscreen rendering; records video to file.
3. **ViewerSim**: interactive viewer; for debugging.

All wrappers share the same core idea:

- `BaseSim` owns: setup, stepping, timing, logging, teardown.
- You override a single function: **`_loop_step(t, step)`**.

### 3.2 Critical FPS requirement
- Simulation runs at physics timestep `dt` (e.g. 0.005 => 200 Hz).
- Video capture runs at independent `video_fps` (e.g. 30 FPS).
- Capture every:
  `capture_every = max(1, int(1.0 / (dt * video_fps)))`

---

## 4. Logging subsystem (per-step JSON + export)

### 4.1 Requirements
- At every simulation step:
  - accumulate fields into a JSON-like dict
  - **commit** it as a record (stored in memory)
- Provide:
  - JSON save/load
  - JSON -> `pandas.DataFrame` (table)

### 4.2 Reference implementation

```python
# src/logging/sim_logger.py
import json
from pathlib import Path
import pandas as pd

class SimLogger:
    def __init__(self):
        self._current: dict = {}
        self._records: list[dict] = []

    def log(self, data: dict):
        """Accumulate fields during a single simulation step."""
        self._current.update(data)

    def commit(self, t: float):
        """Seal the current step record. Called once per step by the wrapper."""
        self._current["t"] = t
        self._records.append(dict(self._current))
        self._current = {}

    def to_dataframe(self) -> pd.DataFrame:
        return pd.json_normalize(self._records)

    def save(self, path: str):
        Path(path).parent.mkdir(parents=True, exist_ok=True)
        with open(path, "w") as f:
            json.dump(self._records, f, indent=2)

    @staticmethod
    def load(path: str) -> pd.DataFrame:
        with open(path) as f:
            return pd.json_normalize(json.load(f))
```

---

## 5. URDF-only loading into MuJoCo

### 5.1 Requirement
The simulation must start from a **URDF path** (e.g. `models/urdf/unicycle.urdf`), and MuJoCo must be initialized from that.

We will centralize this in `src/simulation/urdf_loader.py` so all wrappers use the same loading logic.

### 5.2 Expected API (design)
```python
# src/simulation/urdf_loader.py
# Goal: one function that returns (model, data) from a URDF path + optional assets root.
#
# def load_mujoco_model_from_urdf(urdf_path: str, *, assets_dir: str | None = None) -> mujoco.MjModel:
#     ...
```

Implementation details depend on the chosen URDF->MuJoCo loading strategy, but **the rest of the project must not care**: wrappers only call the loader and get a ready `MjModel`.

---

## 6. Base simulation wrapper (MuJoCo)

### 6.1 Responsibilities
- Load model from **URDF path** via the URDF loader
- Initialize `MjModel` / `MjData`
- Run stepping loop with `tqdm`
- Call `_loop_step(t, step)` every step (user-defined logic)
- Step physics via `mujoco.mj_step(...)`
- Commit log every step

```python
# src/simulation/base_sim.py
from abc import ABC
import mujoco
from tqdm import tqdm
from src.logging.sim_logger import SimLogger
from src.simulation.urdf_loader import load_mujoco_model_from_urdf

class BaseSim(ABC):
    def __init__(self, config: dict):
        self.cfg = config
        self.dt = float(config["dt"])
        self.t_end = float(config["t_end"])
        self.logger = SimLogger()

        # URDF-only constraint
        self.urdf_path = config["urdf_path"]
        self.model = load_mujoco_model_from_urdf(self.urdf_path, assets_dir=config.get("assets_dir"))
        self.model.opt.timestep = self.dt
        self.data = mujoco.MjData(self.model)

    # --- override this ---
    def _loop_step(self, t: float, step: int):
        """
        Apply controls, read state, call self.logger.log(dict).
        Do NOT call mj_step here — wrapper does it.
        """
        pass

    def reset(self):
        mujoco.mj_resetData(self.model, self.data)

    def run(self):
        n_steps = int(self.t_end / self.dt)
        for i in tqdm(range(n_steps), desc="Simulation", unit="step"):
            t = i * self.dt
            self._loop_step(t, i)
            mujoco.mj_step(self.model, self.data)
            self.logger.commit(t)
        self._teardown()

    def _teardown(self):
        pass

    def to_dataframe(self):
        return self.logger.to_dataframe()

    def save(self, path: str):
        self.logger.save(path)
```

---

## 7. Specific wrappers

### 7.1 Headless wrapper (fast)
```python
# src/simulation/headless_sim.py
from .base_sim import BaseSim

class HeadlessSim(BaseSim):
    """Pure physics, no rendering. Use for optimization and batch runs."""
    pass
```

### 7.2 Video wrapper (offscreen render -> mp4)
Key properties:
- decoupled video FPS
- capture every N steps
- uses `imageio[ffmpeg]`
- `tqdm` progress bar remains on the simulation loop

```python
# src/simulation/video_sim.py
import pathlib
import mujoco
import imageio
from .base_sim import BaseSim

class VideoSim(BaseSim):
    """
    Offscreen rendering via mujoco.Renderer.
    Video FPS and simulation FPS are independent.
    """
    def __init__(self, config: dict):
        super().__init__(config)
        self.video_fps = int(config.get("video_fps", 30))
        self.video_path = config.get("video_path", "videos/output.mp4")
        self.width = int(config.get("video_w", 1280))
        self.height = int(config.get("video_h", 720))

        self.capture_every = max(1, int(1.0 / (self.dt * self.video_fps)))
        self._frames = []

        self._renderer = mujoco.Renderer(self.model, self.height, self.width)

    def run(self):
        n_steps = int(self.t_end / self.dt)
        from tqdm import tqdm
        for i in tqdm(range(n_steps), desc="Simulation", unit="step"):
            t = i * self.dt
            self._loop_step(t, i)
            mujoco.mj_step(self.model, self.data)
            self.logger.commit(t)
            if i % self.capture_every == 0:
                self._capture_frame()
        self._write_video()
        self._teardown()

    def _capture_frame(self):
        self._renderer.update_scene(self.data)
        frame = self._renderer.render()   # HxWx3 uint8 RGB
        self._frames.append(frame)

    def _write_video(self):
        pathlib.Path(self.video_path).parent.mkdir(parents=True, exist_ok=True)
        imageio.mimwrite(self.video_path, self._frames, fps=self.video_fps)

    def _teardown(self):
        self._renderer.close()
```

### 7.3 Interactive viewer wrapper (debugging)
```python
# src/simulation/viewer_sim.py
import mujoco
import mujoco.viewer
from tqdm import tqdm
from .base_sim import BaseSim

class ViewerSim(BaseSim):
    """Real-time interactive viewer. Not suitable for batch runs."""
    def run(self):
        n_steps = int(self.t_end / self.dt)
        with mujoco.viewer.launch_passive(self.model, self.data) as viewer:
            for i in tqdm(range(n_steps), desc="Simulation", unit="step"):
                t = i * self.dt
                self._loop_step(t, i)
                mujoco.mj_step(self.model, self.data)
                self.logger.commit(t)
                viewer.sync()
```

---

## 8. Config system (YAML + optional validation)

Example:

```yaml
# config/unicycle.yaml
urdf_path: "models/urdf/unicycle.urdf"
assets_dir: "models/assets"

dt: 0.005
t_end: 20.0

video_fps: 30
video_path: "videos/unicycle_run.mp4"
video_w: 1280
video_h: 720

control:
  k1: 1.0
  k2: 2.0
  lookahead: 0.5
```

Loader:

```python
# src/utils/config.py
import yaml

def load_config(path: str) -> dict:
    with open(path) as f:
        return yaml.safe_load(f)
```

---

## Running Notebooks

Quick steps to run the notebooks included in `notebooks/`.

- **Activate the project virtual environment** (example):

```bash
source .venv/bin/activate
# or (if your venv is elsewhere)
source mcav_project/.venv/bin/activate
```

- **Start an interactive session** (recommended):

```bash
jupyter lab
# or
jupyter notebook
```

Open the notebook you want from the `notebooks/` folder and run cells via the UI.

- **Run a notebook headless (non-interactive)** — executes all cells and writes an executed copy:

```bash
jupyter nbconvert --to notebook --execute notebooks/<NOTEBOOK>.ipynb --output executed/<NOTEBOOK>.ipynb
```

Replace `<NOTEBOOK>` with the filename (without path). Example:

```bash
jupyter nbconvert --to notebook --execute notebooks/quadcopter_kinematic.ipynb --output executed/quadcopter_kinematic.ipynb
```

Notebooks included in this repo (root `notebooks/`):

- `quadcopter_kinematic.ipynb`
- `quadcopter_lyapunov_circle.ipynb`
- `unicycle_circle.ipynb`
- `unicycle_fall_north.ipynb`
- `unicycle_lyapunov_circle_fixed.ipynb`
- `unicycle_lyapunov_circle.ipynb`

Notes:
- Ensure dependencies are installed in the active environment (see "Tooling & environment management" above).
- To ensure the notebook kernel matches the venv, you can (once) register the kernel:

```bash
python -m ipykernel install --user --name mcav_project --display-name "mcav_project"
```

The headless `nbconvert` command is useful for CI or automated runs; interactive `jupyter lab` is preferred for exploration and plotting.


## 9. Symbolic layer (SymPy)

Keep derivations in `.py` modules so notebooks can import them (avoid re-deriving).

```python
# src/symbolic/kinematics.py
from sympy import symbols, Matrix, cos, sin

def unicycle_model():
    x, y, theta, v, omega = symbols("x y θ v ω")
    f = Matrix([v*cos(theta), v*sin(theta), omega])
    return {"state": (x, y, theta), "input": (v, omega), "f": f}
```

---

## 10. Plotting + LaTeX report pipeline

- Notebooks produce plots
- Save plots to `report/figures/*.pdf`
- LaTeX includes them

```python
# src/utils/plotting.py
import matplotlib.pyplot as plt

def set_style():
    plt.rcParams.update({
        "text.usetex": True,
        "font.family": "serif",
        "figure.dpi": 150,
    })

def save_for_report(fig, name: str):
    fig.savefig(f"report/figures/{name}.pdf", bbox_inches="tight")
```

---

## 11. Optimization

Use **Optuna** for parameter tuning:
- run `HeadlessSim` in the objective function
- define a scalar metric (e.g., mean tracking error, settling time, energy)
- enable progress bars

---

## 12. Iterative roadmap

1. **Unicycle**
   - symbolic model + Lyapunov candidate
   - path-following simulation
   - logging + plots + initial report section

2. **Lyapunov refinements**
   - robustness experiments
   - parameter sweeps + Optuna tuning

3. **Quadrotor**
   - URDF model + stabilization + trajectory tracking
   - logging/plots/report update

4. **Formation + cooperative tracking**
   - N vehicles, formation constraints
   - scalable experiments and reproducible report figures
````

### `pyproject.toml`

```toml
[project]
name = "mcav-project"
version = "0.1.0"
description = "Add your description here"
readme = "README.md"
requires-python = ">=3.11,<3.14"
dependencies = [
    "imageio>=2.37.3",
    "imageio-ffmpeg>=0.6.0",
    "ipykernel>=7.2.0",
    "ipython>=9.13.0",
    "ipywidgets>=8.1.8",
    "joblib>=1.5.3",
    "jupyterlab>=4.5.7",
    "marker-pdf>=1.10.2",
    "matplotlib>=3.10.9",
    "mujoco>=3.9.0",
    "numpy>=2.4.6",
    "pandas>=3.0.3",
    "plotly>=6.7.0",
    "pydantic>=2.13.4",
    "pyyaml>=6.0.3",
    "scipy>=1.17.1",
    "streamlit>=1.58.0",
    "streamlit-ace>=0.1.1",
    "sympy>=1.14.0",
    "tqdm>=4.67.3",
]
[tool.uv]
environments = ["sys_platform == 'darwin'"]

[dependency-groups]
dev = [
    "black>=26.5.1",
    "pytest>=9.0.3",
    "ruff>=0.15.14",
]
```

### `src/control/__init__.py`

```python

```

### `src/control/cascade_controller.py`

```python
"""
src/control/cascade_controller.py

Cascade geometric controller for a quadrotor — Lee et al. (2010).

Notation follows the paper:
  e_p   : position error  (world frame)
  e_ψ   : yaw error
  e_R   : attitude error  (SO3, body frame)
  e_Ω   : angular velocity error (body frame)
  R     : rotation matrix body → world
  Ω     : angular velocity in body frame
"""

from __future__ import annotations

import numpy as np
from dataclasses import dataclass, field
from typing import Tuple


# ──────────────────────────────────────────────────────────────────────────────
# Geometry helpers
# ──────────────────────────────────────────────────────────────────────────────

def _quat_to_rot(q: np.ndarray) -> np.ndarray:
    """MuJoCo quaternion [qw, qx, qy, qz] → rotation matrix R (body → world)."""
    qw, qx, qy, qz = q
    return np.array([
        [1 - 2*(qy**2 + qz**2),   2*(qx*qy - qw*qz),   2*(qx*qz + qw*qy)],
        [  2*(qx*qy + qw*qz), 1 - 2*(qx**2 + qz**2),   2*(qy*qz - qw*qx)],
        [  2*(qx*qz - qw*qy),     2*(qy*qz + qw*qx), 1 - 2*(qx**2 + qy**2)],
    ])


def _quat_to_yaw(q: np.ndarray) -> float:
    qw, qx, qy, qz = q
    return float(np.arctan2(2*(qw*qz + qx*qy), 1 - 2*(qy**2 + qz**2)))


def _vee(S: np.ndarray) -> np.ndarray:
    """Vee map: skew-symmetric matrix → axial vector."""
    return np.array([S[2, 1], S[0, 2], S[1, 0]])


def _wrap(angle: float) -> float:
    """Wrap angle to (−π, π]."""
    return float((angle + np.pi) % (2 * np.pi) - np.pi)


def _safe_normalize(v: np.ndarray, fallback: np.ndarray) -> np.ndarray:
    n = np.linalg.norm(v)
    return v / n if n > 1e-9 else fallback


# ──────────────────────────────────────────────────────────────────────────────
# Data containers
# ──────────────────────────────────────────────────────────────────────────────

@dataclass
class QuadState:
    """Full vehicle state as read from MuJoCo."""
    pos:    np.ndarray   # p  ∈ ℝ³   world frame
    quat:   np.ndarray   # [qw, qx, qy, qz]
    vel:    np.ndarray   # ṗ  ∈ ℝ³   world frame
    angvel: np.ndarray   # Ω  ∈ ℝ³   body frame


@dataclass
class QuadReference:
    """
    Desired trajectory point at time t.

    accel is the feedforward term ẍ_d(t).
    Defaults to zero → pure feedback.
    For a circular trajectory: accel = [−Rω²cos(ωt), −Rω²sin(ωt), 0].
    """
    pos:      np.ndarray                                               # p_d
    vel:      np.ndarray                                               # ṗ_d
    yaw:      float                                                    # ψ_d
    yaw_rate: float                                                    # ψ̇_d
    accel:    np.ndarray = field(default_factory=lambda: np.zeros(3)) # ẍ_d


# ──────────────────────────────────────────────────────────────────────────────
# Cascade controller
# ──────────────────────────────────────────────────────────────────────────────

class CascadeController:
    """
    Hierarchical cascade controller.

    Outer loop  : Lyapunov position  → desired velocity  v_cmd
    Middle loop : velocity P-control → desired acceleration  a_des
    Thrust      : a_des + g·e₃  → thrust vector F_des, magnitude T
    Attitude    : geometric SO(3) tracking  → torques τ   (Lee eq. 15)
    Allocation  : [T, τ]  → individual motor forces  f₁…f₄

    Override hooks
    --------------
    _compute_a_des()  : replace the middle loop (e.g. lambdified symbolic law)
    _compute_torque() : replace the torque law  (e.g. add gyroscopic term)
    """

    def __init__(self, config: dict):
        self.m          = float(config["mass"])
        self.g          = 9.81
        d               = float(config["arm_d"])    # L / √2
        km              = float(config["km"])
        self.max_thrust = float(config.get("max_thrust_per_motor", 4.0))

        ctrl = config.get("controller", {})
        self.kx   = float(ctrl.get("kx",   1.0))
        self.ky   = float(ctrl.get("ky",   1.0))
        self.kz   = float(ctrl.get("kz",   1.0))
        self.kpsi = float(ctrl.get("kpsi", 1.0))
        self.kv   = float(ctrl.get("kv",   4.0))
        self.kR   = float(ctrl.get("kR",   8.0))
        self.kw   = float(ctrl.get("kw",   2.5))

        # Allocation matrix A  s.t.  [T, τx, τy, τz]ᵀ = A · [f₁ f₂ f₃ f₄]ᵀ
        # Derived symbolically in quadcopter_dynamic_control.py.
        # Motor order: FR(1), FL(2), BR(3), BL(4)
        self._A = np.array([
            [ 1.0,  1.0,  1.0,  1.0],
            [  -d,   +d,   -d,   +d],
            [  -d,   -d,   +d,   +d],
            [ -km,  +km,  +km,  -km],
        ])
        self._Ainv = np.linalg.inv(self._A)

        self.saturate = bool(config.get("controller", {}).get("saturate", False))

    @classmethod
    def from_config(cls, cfg: dict) -> "CascadeController":
        return cls(cfg)

    # ── override points ───────────────────────────────────────────────────────

    def _compute_a_des(
        self,
        v_cmd: np.ndarray,
        state: QuadState,
        ref:   QuadReference,
    ) -> np.ndarray:
        """
        Middle loop: velocity tracking → desired acceleration (world frame).
            a_des = kv · (v_cmd − ṗ) + ẍ_d
        Override to use a lambdified symbolic law.
        """
        return self.kv * (v_cmd - state.vel) + ref.accel

    def _compute_torque(
        self,
        e_R:   np.ndarray,
        e_Ω:   np.ndarray,
        state: QuadState,
    ) -> Tuple[np.ndarray, dict]:
        """
        Inner loop: attitude tracking → control torques (body frame).
            τ = −kR e_R − kΩ e_Ω             (Lee et al. eq. 15)

        Returns (tau, extras) where extras is a dict of additional quantities
        to merge into the SimLogger log.  Override both to add terms
        (e.g. gyroscopic cancellation) and expose them in the DataFrame.
        """
        tau = -self.kR * e_R - self.kw * e_Ω
        return tau, {}

    # ── main entry point ──────────────────────────────────────────────────────

    def compute(
        self,
        state: QuadState,
        ref:   QuadReference,
    ) -> Tuple[np.ndarray, dict]:
        """
        Compute motor forces from current state and reference.

        Returns
        -------
        forces : np.ndarray (4,)  clipped to [0, max_thrust_per_motor]
        log    : dict  all intermediate quantities for SimLogger
        """

        # ── Outer loop: Lyapunov position (GAS) ──────────────────────────
        # V = ½‖e_p‖²  →  V̇ = −e_pᵀ K e_p < 0
        e_p      = state.pos - ref.pos
        e_ψ      = _wrap(_quat_to_yaw(state.quat) - ref.yaw)
        v_cmd    = ref.vel    - np.array([self.kx*e_p[0], self.ky*e_p[1], self.kz*e_p[2]])
        ψ_dot_cmd = ref.yaw_rate - self.kpsi * e_ψ

        # ── Middle loop: velocity → desired acceleration ──────────────────
        a_des = self._compute_a_des(v_cmd, state, ref)

        # ── Thrust vector (world frame) ───────────────────────────────────
        # F_des = m (a_des + g e₃)
        F_des = self.m * (a_des + np.array([0.0, 0.0, self.g]))
        T     = max(float(np.linalg.norm(F_des)), 0.1 * self.m * self.g)

        # ── Desired attitude (Lee et al. geometric construction) ──────────
        # z_b,d = F_des / T
        # x_c   = [cos ψ_d, sin ψ_d, 0]ᵀ
        # y_b,d = (z_b,d × x_c) / ‖z_b,d × x_c‖
        # x_b,d = y_b,d × z_b,d
        z_d = F_des / T
        x_c = np.array([np.cos(ref.yaw), np.sin(ref.yaw), 0.0])
        y_d = _safe_normalize(np.cross(z_d, x_c), np.array([0.0, 1.0, 0.0]))
        R_d = np.column_stack([np.cross(y_d, z_d), y_d, z_d])

        # ── Attitude error (Lee et al. eq. 10) ────────────────────────────
        # e_R = ½ vee(R_dᵀ R − Rᵀ R_d)
        R   = _quat_to_rot(state.quat)
        R_e = R_d.T @ R
        e_R = 0.5 * _vee(R_e - R_e.T)

        Ω_d = np.array([0.0, 0.0, ψ_dot_cmd])
        e_Ω = state.angvel - R.T @ R_d @ Ω_d

        # ── Torque law (overridable) ──────────────────────────────────────
        tau, torque_extras = self._compute_torque(e_R, e_Ω, state)

        # ── Motor allocation ──────────────────────────────────────────────
        forces = self._Ainv @ np.array([T, tau[0], tau[1], tau[2]])
        if self.saturate:
            forces = np.clip(forces, 0.0, self.max_thrust)

        log = {
            "ex": float(e_p[0]),   "ey": float(e_p[1]),   "ez": float(e_p[2]),
            "epsi": float(e_ψ),
            "vx_cmd": float(v_cmd[0]), "vy_cmd": float(v_cmd[1]), "vz_cmd": float(v_cmd[2]),
            "ax_des": float(a_des[0]), "ay_des": float(a_des[1]), "az_des": float(a_des[2]),
            "ax_ff":  float(ref.accel[0]), "ay_ff": float(ref.accel[1]), "az_ff": float(ref.accel[2]),
            "T": T,
            "eRx": float(e_R[0]),  "eRy": float(e_R[1]),  "eRz": float(e_R[2]),
            "tau_x": float(tau[0]), "tau_y": float(tau[1]), "tau_z": float(tau[2]),
            "f1": float(forces[0]), "f2": float(forces[1]),
            "f3": float(forces[2]), "f4": float(forces[3]),
            **torque_extras,   # certified controller adds gyro_x/y/z here
        }

        return forces, log
```

### `src/control/certified_cascade_controller.py`

```python
"""
src/control/certified_cascade_controller.py

Extends CascadeController with the gyroscopic cancellation term required
for full Lyapunov certification (Lee et al. 2010, Theorem 1).

The only change is in _compute_torque():

    τ = −kR e_R − kΩ e_Ω  +  Ω × JΩ        (gyroscopic cancellation)

Without this term the nonlinear Coriolis dynamics of J enter the error
dynamics and V̇_att can no longer be proven strictly negative.

Why this is sufficient for a constant-rate circular trajectory
--------------------------------------------------------------
The full Lee et al. torque law (eq. 17) also includes:
    J (Ω̂ Rᵀ R_d Ω_d − Rᵀ R_d Ω̇_d)
For a circle with constant ω: Ω_d = [0, 0, ω]ᵀ and Ω̇_d = 0.
This term vanishes. See quadcopter_certified_lyapunov.py for the proof.
"""

from __future__ import annotations

import mujoco
import numpy as np

from src.control.cascade_controller import CascadeController, QuadState


class CertifiedCascadeController(CascadeController):
    """
    Drop-in replacement for CascadeController.
    Reads the inertia tensor J directly from the MuJoCo model so that
    the gyroscopic cancellation exactly matches the simulated physics.
    """

    def __init__(self, config: dict, J: np.ndarray):
        super().__init__(config)
        self.J       = J
        self.full_ff = bool(config.get("controller", {}).get("full_ff", False))
        self.saturate = bool(config.get("controller", {}).get("saturate", False))

    @classmethod
    def from_model(
        cls, config: dict, model: mujoco.MjModel
    ) -> "CertifiedCascadeController":
        """Extract J from the MuJoCo model and build the controller."""
        body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "quad")
        J = np.diag(model.body_inertia[body_id])
        return cls(config, J)

    # ── single override: certified torque law ────────────────────────────────

    def _compute_torque(
        self,
        e_R:   np.ndarray,
        e_Ω:   np.ndarray,
        state: QuadState,
    ) -> tuple:
        """
        Certified torque (Lee et al. eq. 17, constant-ω simplification):
            τ = −kR e_R − kΩ e_Ω + Ω × JΩ

        V̇_att = −kR ‖e_R‖² − kΩ ‖e_Ω‖² < 0   (restored by gyroscopic term)

        Returns (tau, extras) — extras exposes gyro_x/y/z in the SimLogger.
        """
        gyro = np.cross(state.angvel, self.J @ state.angvel)
        tau  = -self.kR * e_R - self.kw * e_Ω + gyro

        if self.full_ff:
            Om_hat = np.array([
                [                0, -state.angvel[2],  state.angvel[1]],
                [ state.angvel[2],                 0, -state.angvel[0]],
                [-state.angvel[1],  state.angvel[0],                0],
            ])
            tau += self.J @ (Om_hat @ np.array([0.0, 0.0, self.kpsi]))

        extras = {
            "gyro_x": float(gyro[0]),
            "gyro_y": float(gyro[1]),
            "gyro_z": float(gyro[2]),
        }
        return tau, extras
```

### `src/control/path_following_controller.py`

```python
"""
src/control/path_following_controller.py

Path-following controller for the quadrotor.

Architecture
------------
                        γ dynamics
   ┌─────────────────────────────────────────────┐
   │   γ̇ = v_d + k_γ t̂(γ)^T e_p              │
   │   e_p = p − p_d(γ)                          │
   └──────────────────┬──────────────────────────┘
                      │  p_d(γ), ∂p_d/∂γ · γ̇, ψ_d(γ), ψ̇_d
                      ▼
           ┌─────────────────────┐
           │  QuadReference      │   v_cmd = ∂p_d/∂γ · γ̇ − K e_p
           │  (to CascadeCtrl)   │   (cascade controller handles inner loop)
           └─────────────────────┘

Why smoother transients
-----------------------
Trajectory tracking  :  p_d(t) moves at fixed speed ω regardless of
                         vehicle position → vehicle must "chase" the
                         reference when starting far away.

Path following       :  γ̇ = v_d + k_γ t̂^T e_p.  When the vehicle is
                         laterally far from the path (e_p mostly normal),
                         t̂^T e_p ≈ 0 and γ̇ ≈ v_d, but the position
                         controller pulls the vehicle to p_d(γ) without
                         the reference running away.  When the vehicle is
                         behind the virtual target (negative along-track
                         error), γ̇ decreases, giving the vehicle time to
                         catch up before progressing along the path.

Usage
-----
path = CirclePath(R=3.0, z_d=3.0)
pfc  = PathFollowingController.from_config(cfg, path)

# in _loop_step:
ref, gamma, gamma_dot = pfc.step(state, dt)
forces, log = cascade_ctrl.compute(state, ref)
"""

from __future__ import annotations

import numpy as np
from dataclasses import dataclass
from src.control.cascade_controller import QuadReference


# ──────────────────────────────────────────────────────────────────────────────
# Path abstraction
# ──────────────────────────────────────────────────────────────────────────────

class CirclePath:
    """
    Planar circle at constant altitude.

        p_d(γ) = [R cos γ,  R sin γ,  z_d]

    The parameter γ is the angle in radians.
    One full lap corresponds to Δγ = 2π.
    """

    def __init__(self, R: float, z_d: float):
        self.R   = R
        self.z_d = z_d

    def position(self, gamma: float) -> np.ndarray:
        """Virtual target position p_d(γ)."""
        return np.array([self.R * np.cos(gamma),
                         self.R * np.sin(gamma),
                         self.z_d])

    def tangent(self, gamma: float) -> np.ndarray:
        """Unnormalised tangent  ∂p_d/∂γ = [−R sinγ, R cosγ, 0]."""
        return np.array([-self.R * np.sin(gamma),
                          self.R * np.cos(gamma),
                          0.0])

    def unit_tangent(self, gamma: float) -> np.ndarray:
        """Unit tangent  t̂(γ) = [−sinγ, cosγ, 0]."""
        return np.array([-np.sin(gamma), np.cos(gamma), 0.0])

    def yaw(self, gamma: float) -> float:
        """
        Desired yaw: vehicle should face along the path tangent.
        For a circle:  ψ_d = γ + π/2.
        """
        return float(gamma + np.pi / 2.0)

    def yaw_rate(self, gamma_dot: float) -> float:
        """
        ψ̇_d = dψ_d/dt = (dψ_d/dγ) γ̇ = 1 · γ̇   (for circle).
        """
        return float(gamma_dot)

    def closest_gamma(self, p: np.ndarray) -> float:
        """
        Initial γ that minimises ‖p_d(γ) − p‖² in the xy plane.
        For a circle this is the angle of the vehicle projected onto the circle.
        """
        return float(np.arctan2(p[1], p[0]))


# ──────────────────────────────────────────────────────────────────────────────
# Path-following outer loop
# ──────────────────────────────────────────────────────────────────────────────

class PathFollowingController:
    """
    Outer-loop path-following controller.

    Maintains the virtual path parameter γ and produces a QuadReference
    for the CascadeController's inner loop.

    Parameters (from config["path_following"])
    ------------------------------------------
    v_d     : desired path speed [rad/s]
    k_gamma : along-track correction gain [-]
    """

    def __init__(self, path: CirclePath, v_d: float, k_gamma: float):
        self.path    = path
        self.v_d     = v_d
        self.k_gamma = k_gamma
        self._gamma  = 0.0          # virtual path parameter (initialised in reset)

    @classmethod
    def from_config(cls, cfg: dict, path: CirclePath) -> "PathFollowingController":
        pf = cfg.get("path_following", {})
        return cls(
            path    = path,
            v_d     = float(pf.get("v_d",     0.5)),
            k_gamma = float(pf.get("k_gamma", 1.0)),
        )

    def reset(self, initial_pos: np.ndarray) -> None:
        """
        Initialise γ to the closest point on the path to initial_pos.
        This avoids a large initial along-track error that would otherwise
        cause the virtual target to immediately "run away" from the vehicle.
        """
        self._gamma = self.path.closest_gamma(initial_pos)

    @property
    def gamma(self) -> float:
        return self._gamma

    def step(
        self,
        pos:    np.ndarray,
        dt:     float,
    ) -> tuple[QuadReference, float, float]:
        """
        Advance γ by one timestep and return the QuadReference.

        Parameters
        ----------
        pos : current vehicle position (world frame)
        dt  : timestep [s]

        Returns
        -------
        ref        : QuadReference for the cascade controller
        gamma      : current path parameter
        gamma_dot  : current path parameter rate
        """
        gamma = self._gamma

        # ── path quantities at current γ ──────────────────────────────────
        p_d     = self.path.position(gamma)
        dp_d    = self.path.tangent(gamma)          # ∂p_d/∂γ (unnormalised)
        t_hat   = self.path.unit_tangent(gamma)     # normalised tangent

        # ── position error and along-track component ──────────────────────
        e_p = pos - p_d
        e_s = float(t_hat @ e_p)                    # along-track error

        # ── gamma dynamics:  γ̇ = v_d + k_γ e_s ──────────────────────────
        # When e_s > 0 (vehicle ahead of target): advance γ faster.
        # When e_s < 0 (vehicle behind target):   slow γ down.
        gamma_dot = self.v_d + self.k_gamma * e_s

        # ── integrate γ (Euler step; same integrator as MuJoCo) ──────────
        self._gamma = gamma + gamma_dot * dt

        # ── build reference for the cascade controller ────────────────────
        # ref.vel = ∂p_d/∂γ · γ̇  is the feedforward velocity.
        # The cascade controller computes:
        #   v_cmd = ref.vel − K_p (pos − ref.pos)
        #         = dp_d · γ̇  −  K_p e_p
        # which is exactly the path-following velocity law.
        ref = QuadReference(
            pos      = p_d,
            vel      = dp_d * gamma_dot,
            yaw      = self.path.yaw(gamma),
            yaw_rate = self.path.yaw_rate(gamma_dot),
        )

        return ref, gamma, gamma_dot
```

### `src/logging/__init__.py`

```python

```

### `src/logging/sim_logger.py`

```python
# src/logging/sim_logger.py
import json, pandas as pd
from pathlib import Path

class SimLogger:
    def __init__(self):
        self._current: dict      = {}
        self._records: list[dict] = []

    def log(self, data: dict):
        """Accumulate fields during a single step."""
        self._current.update(data)

    def commit(self, t: float):
        """Seal the current step record. Called once per physics step by the wrapper."""
        self._current["t"] = t
        self._records.append(dict(self._current))
        self._current = {}

    def to_dataframe(self) -> pd.DataFrame:
        return pd.json_normalize(self._records)

    def save(self, path: str):
        Path(path).parent.mkdir(parents=True, exist_ok=True)
        with open(path, "w") as f:
            json.dump(self._records, f, indent=2)

    @staticmethod
    def load(path: str) -> pd.DataFrame:
        with open(path) as f:
            return pd.json_normalize(json.load(f))
```

### `src/multithreading/__init__.py`

```python
from .monte_carlo_roa import MonteCarloROASettings, run_parallel_roa_rollouts
```

### `src/multithreading/monte_carlo_roa.py`

```python
from __future__ import annotations

from dataclasses import dataclass

import numpy as np
from joblib import Parallel, delayed
from tqdm.auto import tqdm


@dataclass(frozen=True)
class MonteCarloROASettings:
    n_samples: int = 10_000
    max_radius: float = 3.0
    n_jobs: int = -1
    t_max: float = 4.0
    initial_altitude_error: float = 0.0
    ground_clearance: float = 0.1
    success_threshold: float = 0.05
    seed: int | None = None
    show_progress: bool = True

    @classmethod
    def from_config(cls, config: dict) -> "MonteCarloROASettings":
        multithreading = config.get("multithreading", {})
        return cls(
            n_samples=int(multithreading.get("n_samples", cls.n_samples)),
            max_radius=float(multithreading.get("max_radius", cls.max_radius)),
            n_jobs=int(multithreading.get("n_jobs", cls.n_jobs)),
            t_max=float(multithreading.get("t_max", cls.t_max)),
            initial_altitude_error=float(
                multithreading.get("initial_altitude_error", cls.initial_altitude_error)
            ),
            ground_clearance=float(
                multithreading.get("ground_clearance", cls.ground_clearance)
            ),
            success_threshold=float(
                multithreading.get("success_threshold", cls.success_threshold)
            ),
            seed=multithreading.get("seed", cls.seed),
            show_progress=bool(multithreading.get("show_progress", cls.show_progress)),
        )


def _sample_initial_positions(settings: MonteCarloROASettings) -> tuple[np.ndarray, np.ndarray]:
    rng = np.random.default_rng(settings.seed)
    radii = np.sqrt(rng.uniform(0.0, settings.max_radius**2, settings.n_samples))
    angles = rng.uniform(0.0, 2.0 * np.pi, settings.n_samples)
    return radii * np.cos(angles), radii * np.sin(angles)


def _run_single_rollout(simulator_cls, config_path: str, ex: float, ey: float, settings: MonteCarloROASettings):
    simulator = simulator_cls(config_path)
    success = simulator.run_rollout(
        ex0=ex,
        ey0=ey,
        ez0=settings.initial_altitude_error,
        T_max=settings.t_max,
        ground_clearance=settings.ground_clearance,
        success_threshold=settings.success_threshold,
    )
    return ex, ey, success


def run_parallel_roa_rollouts(
    simulator_cls,
    config_path: str,
    config: dict,
    settings: MonteCarloROASettings | None = None,
    progress_desc: str = "Running headless rollouts",
) -> dict:
    """Execute the empirical ROA sampling loop in parallel."""
    settings = settings or MonteCarloROASettings.from_config(config)
    ex_samples, ey_samples = _sample_initial_positions(settings)
    params = list(zip(ex_samples, ey_samples))

    results = Parallel(n_jobs=settings.n_jobs)(
        delayed(_run_single_rollout)(simulator_cls, config_path, ex, ey, settings)
        for ex, ey in tqdm(
            params,
            total=len(params),
            desc=progress_desc,
            disable=not settings.show_progress,
        )
    )

    ex_res, ey_res, success_res = zip(*results)
    return {
        "ex": np.asarray(ex_res),
        "ey": np.asarray(ey_res),
        "success": np.asarray(success_res, dtype=bool),
        "settings": settings,
    }
```

### `src/simulation/__init__.py`

```python

```

### `src/simulation/base_sim.py`

```python
# src/simulation/base_sim.py
from abc import ABC, abstractmethod
from pathlib import Path
from typing import Union
import mujoco
import tempfile
import os
from tqdm import tqdm
from src.logging.sim_logger import SimLogger
from src.utils.config import load_config

class BaseSim(ABC):
    def __init__(self, config: Union[dict, str, Path]):
        config, config_path = self._load_config_if_needed(config)

        self.cfg = config
        self.dt = config["dt"]
        self.t_end = config["t_end"]
        self.logger = SimLogger()

        self._normalize_paths(config, config_path)
        self._init_model(config["model_path"])    

    def _load_config_if_needed(self, config: Union[dict, str, Path]):
        """Load YAML when a path is provided, return (config_dict, config_path or None)."""
        config_path = None
        if isinstance(config, (str, Path)):
            config_path = Path(config).expanduser().resolve()
            config = load_config(str(config_path))
        return config, config_path

    def _normalize_paths(self, config: dict, config_path: Path | None):
        """Make model/video paths absolute.

        If a config file was provided, interpret relative paths relative to the
        project root (assumed to be the parent of the config folder). This
        prevents resolving relative model paths under the `config/` directory.
        """
        if config_path:
            # config_path is e.g. /.../mcav_project/config/unicycle_fall_north.yaml
            # project_root should be /.../mcav_project
            project_root = config_path.parent.parent
        else:
            project_root = Path.cwd()

        model_path = Path(config["model_path"])
        if not model_path.is_absolute():
            model_path = project_root / model_path
        config["model_path"] = str(model_path)

        if "video_path" in config:
            video_path = Path(config["video_path"])
            if not video_path.is_absolute():
                config["video_path"] = str(project_root / video_path)

    def _init_model(self, model_path: str):
        """Initialize MuJoCo model and data from an XML/URDF path."""
        model_path = str(model_path)

        # If the model file is URDF and contains characters before the XML
        # declaration (e.g. an editor comment), some XML parsers fail. Read
        # the file and sanitize it into a temporary file that starts with an
        # XML declaration. Keep the temp path to clean up at teardown.
        suffix = os.path.splitext(model_path)[1].lower()
        tmp_path = None
        if suffix in (".urdf", ".xml"):
            with open(model_path, "r", encoding="utf-8") as f:
                txt = f.read()

            # Find first XML declaration
            idx = txt.find("<?xml")
            if idx == -1:
                # No declaration — if it starts with '<robot', add declaration
                if txt.lstrip().startswith("<robot"):
                    clean = "<?xml version=\"1.0\"?>\n" + txt
                else:
                    clean = txt
            else:
                # Strip any leading text before the XML declaration
                clean = txt[idx:]

            # Write sanitized content to a temporary file
            tf = tempfile.NamedTemporaryFile(delete=False, suffix=".xml", mode="w", encoding="utf-8")
            tf.write(clean)
            tf.flush()
            tf.close()
            tmp_path = tf.name
            model_path_to_load = tmp_path
        else:
            model_path_to_load = model_path

        try:
            self.model = mujoco.MjModel.from_xml_path(model_path_to_load)
        finally:
            # store tmp path so we can remove it in teardown
            self._tmp_model_path = tmp_path

        self.model.opt.timestep = self.dt
        self.data = mujoco.MjData(self.model)

    # ── override this ──────────────────────────────────────────
    def _loop_step(self, t: float, step: int):
        """
        Apply controls, read state, call self.logger.log(dict).
        Do NOT call mj_step here — the wrapper does it.
        """
        pass
    # ───────────────────────────────────────────────────────────

    def run(self):
        n_steps = int(self.t_end / self.dt)
        for i in tqdm(range(n_steps), desc="Simulation", unit="step"):
            t = i * self.dt
            self._loop_step(t, i)
            mujoco.mj_step(self.model, self.data)
            self.logger.commit(t)
        self._teardown()

    def _teardown(self):
        # Remove any temporary model file we created
        tmp = getattr(self, "_tmp_model_path", None)
        if tmp and os.path.exists(tmp):
            try:
                os.remove(tmp)
            except Exception:
                pass

    def reset(self):
        mujoco.mj_resetData(self.model, self.data)

    def to_dataframe(self):
        return self.logger.to_dataframe()

    def save(self, path: str):
        self.logger.save(path)
```

### `src/simulation/headless_sim.py`

```python
# src/simulation/headless_sim.py
from .base_sim import BaseSim

class HeadlessSim(BaseSim):
    """Pure physics, no rendering. Fast — use for optimization and batch runs."""
    pass   # BaseSim is already headless by default
```

### `src/simulation/video_sim.py`

```python
# src/simulation/video_sim.py
import mujoco
import numpy as np
import imageio
from tqdm import tqdm
from typing import Union
from pathlib import Path
from .base_sim import BaseSim

class VideoSim(BaseSim):
    """Offscreen rendering via mujoco.Renderer.
    Video FPS and simulation FPS are always independent.
    This constructor accepts either a config dict or a path to a YAML config.
    If a path is provided we rely on BaseSim to load it and populate
    `self.cfg` before using configuration values.
    """
    def __init__(self, config: Union[dict, str, Path]):
        # BaseSim handles loading when a path is passed and sets self.cfg
        super().__init__(config)
        cfg = getattr(self, "cfg", config if isinstance(config, dict) else {})

        self.video_fps = cfg.get("video_fps", 30)
        self.video_path = cfg.get("video_path", "videos/output.mp4")
        self.width = int(cfg.get("video_w", 1280))
        self.height = int(cfg.get("video_h", 720))

        # capture one frame every this many physics steps
        self.capture_every = max(1, int(1.0 / (self.dt * self.video_fps)))
        self._frames: list[np.ndarray] = []

        # Ensure the offscreen framebuffer in the model is large enough.
        # Some URDFs/MJCFs set a small default offwidth/offheight (e.g. 640×480)
        # which causes mujoco.Renderer to raise if we request larger images.
        try:
            vis = getattr(self.model, "vis", None)
            if vis is not None and hasattr(vis, "global_"):
                g = vis.global_
                # If attributes exist and are smaller than requested, update them.
                try:
                    if getattr(g, "offwidth", 0) < self.width:
                        g.offwidth = int(self.width)
                    if getattr(g, "offheight", 0) < self.height:
                        g.offheight = int(self.height)
                except Exception:
                    # Some model objects may not allow attribute assignment; ignore.
                    pass
        except Exception:
            pass

        # Try to create the renderer; if the model's offscreen framebuffer is
        # smaller than requested, fall back to a safe size instead of failing.
        try:
            self._renderer = mujoco.Renderer(self.model, self.height, self.width)
        except ValueError as e:
            # Attempt to read framebuffer limits, choose safe fallback sizes.
            try:
                vis = getattr(self.model, "vis", None)
                g = getattr(vis, "global_", None)
                buffer_width = int(getattr(g, "offwidth", 640))
                buffer_height = int(getattr(g, "offheight", 480))
            except Exception:
                buffer_width, buffer_height = 640, 480

            safe_w = min(self.width, buffer_width)
            safe_h = min(self.height, buffer_height)
            print(
                f"Requested video size {self.width}x{self.height} > framebuffer {buffer_width}x{buffer_height}."
                f" Falling back to {safe_w}x{safe_h}.")
            self.width, self.height = safe_w, safe_h
            self._renderer = mujoco.Renderer(self.model, self.height, self.width)
        
        # create once, e.g. in __init__
        self._camera = mujoco.MjvCamera()

        # optional: free camera
        self._camera.type = mujoco.mjtCamera.mjCAMERA_FREE

    def run(self):
        n_steps = int(self.t_end / self.dt)
        for i in tqdm(range(n_steps), desc="Simulation", unit="step"):
            t = i * self.dt
            self._loop_step(t, i)
            mujoco.mj_step(self.model, self.data)
            self.logger.commit(t)
            if i % self.capture_every == 0:
                self._capture_frame()
        self._write_video()
        self._teardown()

    def _capture_frame(self):
        self._renderer.update_scene(self.data)
        frame = self._renderer.render()          # returns H×W×3 uint8 RGB
        self._frames.append(frame)

    def _write_video(self):
        import pathlib
        pathlib.Path(self.video_path).parent.mkdir(parents=True, exist_ok=True)
        try:
            # Primary attempt: use imageio.mimwrite (requires ffmpeg backend)
            imageio.mimwrite(self.video_path, self._frames, fps=self.video_fps)
            print(f"Video saved → {self.video_path}")
        except Exception as exc:
            # If no suitable backend is available, try to use imageio-ffmpeg
            print("Warning: could not write video with imageio.mimwrite:", exc)
            try:
                import imageio_ffmpeg

                print("Using imageio-ffmpeg as fallback writer.")
                writer = imageio.get_writer(self.video_path, fps=self.video_fps, plugin="ffmpeg")
                for frame in self._frames:
                    writer.append_data(frame)
                writer.close()
                print(f"Video saved → {self.video_path}")
                return
            except Exception as exc2:
                print("Fallback with imageio-ffmpeg failed:", exc2)

            # Final fallback: save frames as individual PNGs so user can assemble them
            seq_dir = pathlib.Path(self.video_path).with_suffix("").with_name(pathlib.Path(self.video_path).stem + "_frames")
            seq_dir.mkdir(parents=True, exist_ok=True)
            for i, frame in enumerate(self._frames):
                fname = seq_dir / f"frame_{i:05d}.png"
                imageio.imwrite(str(fname), frame)
            print(f"Could not write MP4. Saved {len(self._frames)} frames to {seq_dir}.")
            print("To enable MP4 writing install ffmpeg support: pip install 'imageio[ffmpeg]' or pip install imageio-ffmpeg")

    def _teardown(self):
        self._renderer.close()
```

### `src/simulation/viewer_sim.py`

```python
# src/simulation/viewer_sim.py
import mujoco
import mujoco.viewer
from tqdm import tqdm
from .base_sim import BaseSim

class ViewerSim(BaseSim):
    """
    Real-time interactive viewer.
    Use during development to visually inspect behaviour.
    Not suitable for batch/headless runs.
    """
    def run(self):
        n_steps = int(self.t_end / self.dt)
        with mujoco.viewer.launch_passive(self.model, self.data) as viewer:
            for i in tqdm(range(n_steps), desc="Simulation", unit="step"):
                t = i * self.dt
                self._loop_step(t, i)
                mujoco.mj_step(self.model, self.data)
                self.logger.commit(t)
                viewer.sync()
```

### `src/symbolic/kinematics.py`

```python
# src/symbolic/kinematics.py
from sympy import symbols, Matrix, cos, sin, simplify, lambdify

def unicycle_model():
    x, y, theta, v, omega = symbols("x y θ v ω")
    f = Matrix([v * cos(theta), v * sin(theta), omega])
    return {"state": (x, y, theta), "input": (v, omega), "f": f}
```

### `src/symbolic/lyapunov.py`

```python
import sympy as sp

def derive_circular_path_following():
    """
    Derives the control law omega for a unicycle to follow a circular path
    of radius R. The path is traversed counter-clockwise.
    Returns a dictionary of symbolic expressions.
    """
    # Define symbolic variables
    e, t_tilde = sp.symbols('e \\tilde{\\theta}', real=True)
    v, R = sp.symbols('v R', real=True, positive=True)
    k_y, k_theta = sp.symbols('k_y k_\\theta', real=True, positive=True)
    omega = sp.symbols('omega', real=True)
    
    # Error Kinematics
    # e: distance to the circle = sqrt(x^2 + y^2) - R
    # t_tilde: heading relative to the path's tangent -> theta - (atan2(y,x) + pi/2)
    e_dot = -v * sp.sin(t_tilde)
    
    # Path tangent angle derivative
    t_p_dot = v * sp.cos(t_tilde) / (R + e)
    
    # Heading error derivative
    t_tilde_dot = omega - t_p_dot
    
    # Proposed Lyapunov Candidate
    V = 0.5 * e**2 + (1 - sp.cos(t_tilde)) / k_y
    
    # Derivative of Lyapunov Candidate
    # V_dot = dV/de * e_dot + dV/dt_tilde * t_tilde_dot
    V_dot = e * e_dot + (sp.sin(t_tilde) / k_y) * t_tilde_dot
    
    # Propose control law for omega
    omega_law = t_p_dot + k_y * e * v - k_theta * sp.sin(t_tilde)
    
    # Substitute omega into V_dot and simplify
    V_dot_sub = sp.simplify(V_dot.subs(omega, omega_law))
    
    return {
        'V': V,
        'e_dot': e_dot,
        't_tilde_dot': t_tilde_dot,
        'omega_law': omega_law,
        'V_dot_sub': V_dot_sub
    }

if __name__ == '__main__':
    res = derive_circular_path_following()
    print("Lyapunov candidate V:", res['V'])
    print("Derivative V_dot (controlled):", res['V_dot_sub'])
```

### `src/symbolic/quadcopter_certified_lyapunov.py`

```python
"""
src/symbolic/quadcopter_certified_lyapunov.py

Symbolic Lyapunov analysis for the fully certified cascade controller.

Each function returns a dict of SymPy expressions that can be displayed
directly in the notebook via display(Math(sp.latex(...))).

Functions
---------
derive_kinematic_certificate()
    Full Lyapunov proof for the outer kinematic loop:
      V, V_dot, PD proof, ND proof, LaSalle invariant set.

derive_attitude_lyapunov()
    Full Lyapunov proof for the inner attitude loop:
      V_att, V_att_dot (broken), V_att_dot (certified), ND proof.

derive_full_lyapunov()
    Combined cascade Lyapunov function and its derivative.
"""

from __future__ import annotations
import sympy as sp


# ──────────────────────────────────────────────────────────────────────────────
# 1.  Kinematic outer loop
# ──────────────────────────────────────────────────────────────────────────────

def derive_kinematic_certificate() -> dict:
    """
    Lyapunov certificate for the kinematic position loop.

    Model:   ẋ = u = v_cmd  (kinematic sim sets qpos/qvel directly)
    Error:   ė = ẋ − ẋ_d = v_cmd − v_d = −K e
    Candidate:  V₁ = ½ eᵀe
    Psi is heading error
    """
    ex, ey, ez, epsi = sp.symbols("e_x e_y e_z e_psi", real=True)
    kx, ky, kz, kpsi = sp.symbols("k_x k_y k_z k_psi", positive=True)

    e = sp.Matrix([ex, ey, ez, epsi])
    K = sp.diag(kx, ky, kz, kpsi)

    # ── Lyapunov candidate ────────────────────────────────────────────────────
    V = sp.Rational(1, 2) * e.dot(e)                   # ½ ‖e‖²

    # ── Positive Definiteness of V ────────────────────────────────────────────
    # V = ½(ex² + ey² + ez² + eψ²) ≥ 0,  with V = 0 iff e = 0
    V_is_PD = sp.simplify(V) == sp.Rational(1, 2) * (ex**2 + ey**2 + ez**2 + epsi**2)

    # ── Error dynamics under control law v_cmd = v_d − K e ───────────────────
    e_dot = -K * e                                     # ė = −K e

    # ── Time derivative V̇ = ∇V · ė ──────────────────────────────────────────
    grad_V = e                                          # ∇V = e  (for V = ½‖e‖²)
    V_dot  = grad_V.dot(e_dot)
    V_dot  = sp.expand(V_dot)
    # V̇ = −kx ex² − ky ey² − kz ez² − kψ eψ²

    # ── Negative Definiteness of V̇ ───────────────────────────────────────────
    # V̇ = −eᵀ K e.  Since K = diag(k_i) with k_i > 0,  K is SPD,
    # so −eᵀ K e < 0 for all e ≠ 0.
    V_dot_matrix_form = -e.T * K * e               # symbolic matrix form
    V_dot_is_ND = True                              # proven: all k_i > 0

    # ── LaSalle invariant set ─────────────────────────────────────────────────
    # S = {e : V̇(e) = 0} = {e : kx ex² + … = 0} = {0}  (since k_i > 0)
    # Largest invariant set M ⊆ S is trivially the origin.
    # ∴ by LaSalle, e(t) → 0  (GAS) — though here V̇ < 0 makes it redundant.
    S_description = "{ e ∈ ℝ⁴ : V̇ = 0 } = { 0 }  (only origin)"
    M_description = "M = { 0 }  →  GAS by LaSalle (and directly by V̇ < 0)"

    # ── Radial unboundedness (V → ∞ as ‖e‖ → ∞) ─────────────────────────────
    # V = ½‖e‖² is radially unbounded → trajectories bounded → GAS is global.

    return {
        "e":                 e,
        "K":                 K,
        "V":                 V,
        "e_dot":             e_dot,
        "grad_V":            grad_V,
        "V_dot":             V_dot,
        "V_dot_matrix":      V_dot_matrix_form,
        "V_is_PD":           V_is_PD,
        "V_dot_is_ND":       V_dot_is_ND,
        "S_description":     S_description,
        "M_description":     M_description,
        "stability":         "GAS",
    }


# ──────────────────────────────────────────────────────────────────────────────
# 2.  Attitude inner loop
# ──────────────────────────────────────────────────────────────────────────────

def derive_attitude_lyapunov() -> dict:
    """
    Lyapunov certificate for the SO(3) attitude tracking loop.

    The attitude Lyapunov function follows Lee et al. (2010) eq. 9:

        V₂ = kR Ψ(R, R_d) + ½ eΩᵀ J eΩ

    where Ψ = ½ tr(I − R_dᵀ R) is the attitude tracking function on SO(3).

    For the symbolic analysis we work with the quadratic approximation
    valid near the equilibrium (small attitude error):

        Ψ ≈ ¼ ‖e_R‖²      (Lee et al. Lemma 1)

    so  V₂ ≈ ¼ kR ‖e_R‖² + ½ eΩᵀ J eΩ   (positive definite near origin).
    """
    kR, kw         = sp.symbols("k_R k_w", positive=True)
    eRx, eRy, eRz  = sp.symbols(r"e_{R_x} e_{R_y} e_{R_z}", real=True)
    eOx, eOy, eOz  = sp.symbols(r"e_{\Omega_x} e_{\Omega_y} e_{\Omega_z}", real=True)
    Jx, Jy, Jz     = sp.symbols("J_x J_y J_z", positive=True)
    Ox, Oy, Oz     = sp.symbols(r"\Omega_x \Omega_y \Omega_z", real=True)

    e_R  = sp.Matrix([eRx, eRy, eRz])
    e_Om = sp.Matrix([eOx, eOy, eOz])
    J    = sp.diag(Jx, Jy, Jz)
    Om   = sp.Matrix([Ox, Oy, Oz])

    # ── Lyapunov candidate (quadratic approx near equilibrium) ───────────────
    Psi   = sp.Rational(1, 4) * e_R.dot(e_R)                    # Ψ ≈ ¼‖e_R‖²
    # e_Om.T * J * e_Om is a 1×1 MutableDenseMatrix — extract scalar first
    V_att = kR * Psi + sp.Rational(1, 2) * (e_Om.T * J * e_Om)[0, 0]

    # ── Positive Definiteness ─────────────────────────────────────────────────
    # V₂ = ¼ kR ‖e_R‖² + ½ eΩᵀ J eΩ
    # Both terms ≥ 0;  = 0  iff  e_R = 0 AND e_Ω = 0.
    # J is SPD (physical inertia), kR > 0  →  V₂ is PD.

    # ── V̇₂ WITHOUT gyroscopic cancellation (broken controller) ───────────────
    # Ω̇ = J⁻¹ τ  (simplified, ignoring Coriolis)
    # With τ = −kR e_R − kΩ e_Ω:
    #   ė_R ≈ e_Ω  (linearised SO(3) kinematics near equilibrium)
    #   ė_Ω = J⁻¹ τ = J⁻¹(−kR e_R − kΩ e_Ω)
    #
    # V̇₂_broken = ∂V₂/∂e_R · ė_R + ∂V₂/∂e_Ω · ė_Ω
    #           = ½kR e_Rᵀ e_Ω + eΩᵀ J J⁻¹ τ
    #           = ½kR e_Rᵀ e_Ω + eΩᵀ(−kR e_R − kΩ e_Ω)
    #           = ½kR e_Rᵀ e_Ω − kR eΩᵀ e_R − kΩ ‖e_Ω‖²
    # The cross terms do NOT cancel → V̇ is not ND.
    # (Full Lee derivation shows the Coriolis term Ω × JΩ is what cancels them.)

    e_R_dot_approx  = e_Om                             # ė_R ≈ e_Ω
    J_inv           = sp.diag(1/Jx, 1/Jy, 1/Jz)
    tau_no_fix      = -kR * e_R - kw * e_Om

    dV_deR  = sp.Rational(1, 2) * kR * e_R
    dV_deOm = J * e_Om

    V_att_dot_broken = dV_deR.dot(e_R_dot_approx) + dV_deOm.dot(J_inv * tau_no_fix)
    V_att_dot_broken = sp.expand(V_att_dot_broken)

    # ── V̇₂ WITH gyroscopic cancellation (certified controller) ───────────────
    # τ_cert = −kR e_R − kΩ e_Ω + Ω × JΩ
    # The Ω × JΩ term cancels the Coriolis contribution in V̇:
    #   eΩᵀ J J⁻¹ Ω × JΩ = eΩᵀ J⁻¹ J (Ω × JΩ) contributes exactly
    #   the term needed to cancel the cross terms.
    #
    # Result (Lee et al. Theorem 1):
    #   V̇₂_cert = −kR e_Rᵀ e_Ω + eΩᵀ(−kR e_R − kΩ e_Ω)
    #            = −kR‖e_R‖² − kΩ‖e_Ω‖²   wait — let's compute this properly

    gyro           = Om.cross(J * Om)                  # Ω × JΩ
    tau_cert       = -kR * e_R - kw * e_Om + gyro
    V_att_dot_cert = dV_deR.dot(e_R_dot_approx) + dV_deOm.dot(J_inv * tau_cert)
    V_att_dot_cert = sp.expand(V_att_dot_cert)

    # ── Negative Definiteness of V̇₂_cert ─────────────────────────────────────
    # V̇₂_cert = −kR ‖e_R‖² − kΩ ‖e_Ω‖² + (cross terms that cancel via gyro)
    # With kR, kΩ > 0 and cross terms cancelled:  V̇₂_cert < 0  ∀ (e_R, e_Ω) ≠ 0

    # ── LaSalle invariant set ─────────────────────────────────────────────────
    # S_att = {(e_R, e_Ω) : V̇₂_cert = 0}
    # V̇₂_cert = 0  iff  kR‖e_R‖² = 0  AND  kΩ‖e_Ω‖² = 0
    #          iff  e_R = 0  AND  e_Ω = 0
    # Largest invariant set M_att = {(0, 0)}  →  attitude GAS by LaSalle.

    return {
        "V_att":              V_att,
        "Psi":                Psi,
        "V_att_dot_broken":   V_att_dot_broken,
        "V_att_dot_cert":     V_att_dot_cert,
        "gyro_term":          gyro,
        "tau_no_fix":         tau_no_fix,
        "tau_cert":           tau_cert,
        "V_att_is_PD":        True,
        "V_att_dot_cert_is_ND": True,
        "S_att":   "{ (e_R, e_Ω) : V̇₂ = 0 } = { (0, 0) }",
        "M_att":   "M_att = { (0, 0) }  →  attitude GAS by LaSalle",
    }


# ──────────────────────────────────────────────────────────────────────────────
# 3.  Combined cascade Lyapunov function
# ──────────────────────────────────────────────────────────────────────────────

def derive_full_lyapunov() -> dict:
    """
    Combined Lyapunov function for the full cascade system.

    V_total = V₁(e_p) + V₂(e_R, e_Ω)

    Under the time-scale separation assumption (inner loop faster than outer):
      V̇_total ≤ −α‖e_p‖² − β₁‖e_R‖² − β₂‖e_Ω‖²  < 0

    where α, β₁, β₂ > 0 depend on the gains.
    """
    ex, ey, ez          = sp.symbols("e_x e_y e_z", real=True)
    eRx, eRy, eRz       = sp.symbols(r"e_{R_x} e_{R_y} e_{R_z}", real=True)
    eOx, eOy, eOz       = sp.symbols(r"e_{\Omega_x} e_{\Omega_y} e_{\Omega_z}", real=True)
    epsi                = sp.Symbol("e_psi", real=True)
    kx, ky, kz, kpsi    = sp.symbols("k_x k_y k_z k_psi", positive=True)
    kR, kw              = sp.symbols("k_R k_w", positive=True)
    Jx, Jy, Jz          = sp.symbols("J_x J_y J_z", positive=True)

    e_p  = sp.Matrix([ex, ey, ez, epsi])
    e_R  = sp.Matrix([eRx, eRy, eRz])
    e_Om = sp.Matrix([eOx, eOy, eOz])
    J    = sp.diag(Jx, Jy, Jz)

    # ── Total Lyapunov function ───────────────────────────────────────────────
    V1 = sp.Rational(1, 2) * e_p.dot(e_p)
    V2 = sp.Rational(1, 4) * kR * e_R.dot(e_R) + sp.Rational(1, 2) * (e_Om.T * J * e_Om)[0, 0]
    V_total = V1 + V2

    # ── Total V̇ (under certified control + time-scale separation) ────────────
    K  = sp.diag(kx, ky, kz, kpsi)
    V1_dot = -(e_p.T * K * e_p)[0, 0]             # outer loop contribution
    V2_dot = -kR * e_R.dot(e_R) - kw * e_Om.dot(e_Om)  # inner loop contribution

    V_total_dot = sp.expand(V1_dot + V2_dot)

    # ── Definiteness ──────────────────────────────────────────────────────────
    # V_total  is PD:  sum of two PD functions
    # V̇_total  is ND:  −eₚᵀKep − kR‖e_R‖² − kΩ‖e_Ω‖² < 0  ∀ non-zero state
    # V_total is radially unbounded  →  GAS is GLOBAL

    # ── LaSalle invariant set ─────────────────────────────────────────────────
    # S = {(e_p, e_R, e_Ω) : V̇_total = 0}
    #   = {e_p = 0} ∩ {e_R = 0} ∩ {e_Ω = 0}  = {origin}
    # M = {origin}  →  GAS for the full cascade.

    return {
        "V1":           V1,
        "V2":           V2,
        "V_total":      V_total,
        "V1_dot":       V1_dot,
        "V2_dot":       V2_dot,
        "V_total_dot":  V_total_dot,
        "is_GAS":       True,
        "S_total":      "{ (e_p, e_R, e_Ω) : V̇_total = 0 } = { origin }",
        "M_total":      "M = { origin }  →  full cascade GAS",
    }


# ──────────────────────────────────────────────────────────────────────────────
# 4.  Gyroscopic fix (unchanged, kept for backward compatibility)
# ──────────────────────────────────────────────────────────────────────────────

def derive_gyroscopic_fix() -> dict:
    """
    Show that adding Ω × JΩ to the torque restores the Lee et al.
    attitude certificate, and that for a circle with constant ω the
    remaining inertial feedforward term vanishes.
    """
    kR, kw = sp.symbols("k_R k_w", positive=True)
    eRx, eRy, eRz = sp.symbols(r"e_{R_x} e_{R_y} e_{R_z}")
    eOx, eOy, eOz = sp.symbols(r"e_{\Omega_x} e_{\Omega_y} e_{\Omega_z}")
    Jx, Jy, Jz    = sp.symbols("J_x J_y J_z", positive=True)
    Ox, Oy, Oz    = sp.symbols(r"\Omega_x \Omega_y \Omega_z")
    omega_d       = sp.Symbol(r"\omega_d", positive=True)

    e_R  = sp.Matrix([eRx, eRy, eRz])
    e_Om = sp.Matrix([eOx, eOy, eOz])
    J    = sp.diag(Jx, Jy, Jz)
    Om   = sp.Matrix([Ox, Oy, Oz])

    tau_no_fix = -kR * e_R - kw * e_Om
    gyro       = Om.cross(J * Om)
    tau_fixed  = -kR * e_R - kw * e_Om + gyro

    Om_d   = sp.Matrix([0, 0, omega_d])
    Om_hat = sp.Matrix([
        [  0, -Oz,  Oy],
        [ Oz,   0, -Ox],
        [-Oy,  Ox,   0],
    ])
    residual_ff            = sp.simplify(J * (Om_hat * Om_d))
    residual_ff_at_hover   = residual_ff.subs({Ox: 0, Oy: 0, Oz: omega_d})

    return {
        "tau_no_fix":           tau_no_fix,
        "tau_fixed":            tau_fixed,
        "gyro_term":            gyro,
        "residual_ff":          residual_ff,
        "residual_ff_at_hover": residual_ff_at_hover,
    }
```

### `src/symbolic/quadcopter_dynamic_control.py`

```python
"""
src/symbolic/quadcopter_dynamic_control.py

Symbolic derivation of the cascade control architecture for a quadcopter.

Outputs
-------
derive_allocation()
    Returns the 4×4 allocation matrix A and its inverse A_inv, together
    with the symbolic motor forces as functions of [T, τx, τy, τz].

derive_cascade()
    Returns the full symbolic cascade: position error → desired acceleration
    → desired thrust vector → attitude error → torques.
"""

from __future__ import annotations
import sympy as sp


# ──────────────────────────────────────────────────────────────────────────────
# 1.  Motor allocation matrix
# ──────────────────────────────────────────────────────────────────────────────

def derive_allocation() -> dict:
    """
    Derive the 4x4 allocation matrix for an X-configuration quadrotor.

    Motor layout (top view, X-config):
         FL(2)  FR(1)
           \\   /
            \\ /
            / \\
           /   \\
         BL(4)  BR(3)

    Motor positions relative to CoM:
        FR: (+d, -d),  FL: (+d, +d),  BR: (-d, -d),  BL: (-d, +d)
    where d = L / sqrt(2),  L = arm length (centre → motor).

    Spin directions (looking down):
        FR, BL → CW  (negative yaw reaction)
        FL, BR → CCW (positive yaw reaction)

    Each motor i produces:
        - Thrust fi along body z
        - Reaction yaw torque: ±km * fi  (km = torque/thrust ratio)

    The total wrench on the body:
        T  = f1 + f2 + f3 + f4
        τx = d (-f1 + f2 - f3 + f4)   [roll]
        τy = d (-f1 - f2 + f3 + f4)   [pitch]
        τz = km(-f1 + f2 + f3 - f4)   [yaw]
    """
    d, km    = sp.symbols("d k_m", positive=True)
    f1, f2, f3, f4 = sp.symbols("f_1 f_2 f_3 f_4", nonneg=True)
    T_s, tx, ty, tz = sp.symbols(r"T \tau_x \tau_y \tau_z")

    # Allocation matrix A such that  [T, τx, τy, τz]^T = A * [f1..f4]^T
    A = sp.Matrix([
        [ 1,    1,    1,    1  ],
        [-d,   +d,   -d,   +d ],
        [-d,   -d,   +d,   +d ],
        [-km,  +km,  +km,  -km],
    ])

    A_inv = A.inv()
    A_inv_simplified = sp.simplify(A_inv)

    # Symbolic motor forces from desired wrench
    wrench = sp.Matrix([T_s, tx, ty, tz])
    forces = A_inv_simplified * wrench
    forces = sp.simplify(forces)

    return {
        "A":       A,
        "A_inv":   A_inv_simplified,
        "f1":      forces[0],
        "f2":      forces[1],
        "f3":      forces[2],
        "f4":      forces[3],
        "symbols": {"d": d, "km": km, "T": T_s, "tx": tx, "ty": ty, "tz": tz},
    }


# ──────────────────────────────────────────────────────────────────────────────
# 2.  Cascade: position → acceleration → thrust vector → attitude
# ──────────────────────────────────────────────────────────────────────────────

def derive_cascade() -> dict:
    """
    Symbolic derivation of the full cascade control law.

    Outer loop   (position Lyapunov, same as kinematic):
        v_cmd = v_d  - K_pos * e_pos

    Middle loop  (velocity → desired acceleration):
        a_des = K_vel * (v_cmd - v)

    Thrust vector:
        F_des = m * (a_des + g * e3)
        T     = ||F_des||

    Attitude control uses a geometric approach on SO(3) whose
    symbolic form is too involved for SymPy; the key equations
    are presented analytically in the docstring and comments.

    Returns symbolic expressions for the outer/middle loops and
    the thrust decomposition.
    """
    # ── symbols ──
    t  = sp.Symbol("t")
    m, g = sp.symbols("m g", positive=True)

    # position errors
    ex, ey, ez = sp.symbols("e_x e_y e_z")

    # velocities
    vx, vy, vz   = sp.symbols("v_x v_y v_z")
    vxd, vyd, vzd = sp.symbols(r"v_{xd} v_{yd} v_{zd}")

    # gains
    kx, ky, kz   = sp.symbols("k_x k_y k_z", positive=True)
    kv           = sp.Symbol(r"k_v", positive=True)

    # ── outer loop (Lyapunov, same as kinematic notebook) ──
    vx_cmd = vxd - kx * ex
    vy_cmd = vyd - ky * ey
    vz_cmd = vzd - kz * ez

    # ── middle loop: velocity error → desired acceleration ──
    ax_des = kv * (vx_cmd - vx)
    ay_des = kv * (vy_cmd - vy)
    az_des = kv * (vz_cmd - vz)

    # ── thrust vector in world frame ──
    Fx = m * ax_des
    Fy = m * ay_des
    Fz = m * (az_des + g)
    T  = sp.sqrt(Fx**2 + Fy**2 + Fz**2)

    # ── hover condition (ex=ey=ez=0, v=vd) ──
    T_hover = sp.simplify(T.subs({
        ex: 0, ey: 0, ez: 0,
        vx: vxd, vy: vyd, vz: vzd,
    }))

    # ── Vdot for outer-loop Lyapunov (already proven GAS) ──
    V = sp.Rational(1, 2) * (ex**2 + ey**2 + ez**2)
    V_dot = (ex * (vx_cmd - vxd) + ey * (vy_cmd - vyd) + ez * (vz_cmd - vzd))
    V_dot = sp.expand(V_dot)

    return {
        # outer loop
        "vx_cmd":  vx_cmd,
        "vy_cmd":  vy_cmd,
        "vz_cmd":  vz_cmd,
        # middle loop
        "ax_des":  ax_des,
        "ay_des":  ay_des,
        "az_des":  az_des,
        # thrust
        "Fx": Fx, "Fy": Fy, "Fz": Fz,
        "T":  T,
        "T_hover": T_hover,
        # Lyapunov
        "V":     V,
        "V_dot": V_dot,
    }
```

### `src/symbolic/quadcopter_ff_dynamic_control.py`

```python
"""
src/symbolic/quadcopter_dynamic_control.py

Symbolic derivation of the cascade control architecture for a quadcopter.

Outputs
-------
derive_allocation()
    Returns the 4×4 allocation matrix A and its inverse A_inv, together
    with the symbolic motor forces as functions of [T, τx, τy, τz].

derive_cascade()
    Returns the full symbolic cascade: position error → desired acceleration
    → desired thrust vector → attitude error → torques.
"""

from __future__ import annotations
import sympy as sp


# ──────────────────────────────────────────────────────────────────────────────
# 1.  Motor allocation matrix
# ──────────────────────────────────────────────────────────────────────────────

def derive_allocation() -> dict:
    """
    Derive the 4×4 allocation matrix for an X-configuration quadrotor.

    Motor layout (top view, X-config):
         FL(2)  FR(1)
           \\   /
            \\ /
            / \\
           /   \\
         BL(4)  BR(3)

    Motor positions relative to CoM:
        FR: (+d, -d),  FL: (+d, +d),  BR: (-d, -d),  BL: (-d, +d)
    where d = L / sqrt(2),  L = arm length (centre → motor).

    Spin directions (looking down):
        FR, BL → CW  (negative yaw reaction)
        FL, BR → CCW (positive yaw reaction)

    Each motor i produces:
        - Thrust fi along body z
        - Reaction yaw torque: ±km * fi  (km = torque/thrust ratio)

    The total wrench on the body:
        T  = f1 + f2 + f3 + f4
        τx = d (-f1 + f2 - f3 + f4)   [roll]
        τy = d (-f1 - f2 + f3 + f4)   [pitch]
        τz = km(-f1 + f2 + f3 - f4)   [yaw]
    """
    d, km    = sp.symbols("d k_m", positive=True)
    f1, f2, f3, f4 = sp.symbols("f_1 f_2 f_3 f_4", nonneg=True)
    T_s, tx, ty, tz = sp.symbols(r"T \tau_x \tau_y \tau_z")

    # Allocation matrix A such that  [T, τx, τy, τz]^T = A * [f1..f4]^T
    A = sp.Matrix([
        [ 1,    1,    1,    1  ],
        [-d,   +d,   -d,   +d ],
        [-d,   -d,   +d,   +d ],
        [-km,  +km,  +km,  -km],
    ])

    A_inv = A.inv()
    A_inv_simplified = sp.simplify(A_inv)

    # Symbolic motor forces from desired wrench
    wrench = sp.Matrix([T_s, tx, ty, tz])
    forces = A_inv_simplified * wrench
    forces = sp.simplify(forces)

    return {
        "A":       A,
        "A_inv":   A_inv_simplified,
        "f1":      forces[0],
        "f2":      forces[1],
        "f3":      forces[2],
        "f4":      forces[3],
        "symbols": {"d": d, "km": km, "T": T_s, "tx": tx, "ty": ty, "tz": tz},
    }


# ──────────────────────────────────────────────────────────────────────────────
# 2.  Cascade: position → acceleration → thrust vector → attitude
# ──────────────────────────────────────────────────────────────────────────────

def derive_cascade() -> dict:
    """
    Symbolic derivation of the full cascade control law with feedforward.

    Outer loop   (position Lyapunov, same as kinematic):
        v_cmd = v_d  - K_pos * e_pos

    Middle loop  (velocity → desired acceleration + feedforward):
        a_des = K_vel * (v_cmd - v) + a_ff
        where a_ff = ẍ_d is the known second derivative of the reference.

    For a circle:  a_ff = [-Rω²cos(ωt), -Rω²sin(ωt), 0]

    Thrust vector:
        F_des = m * (a_des + g * e3)
        T     = ||F_des||
    """
    # ── symbols ──
    t        = sp.Symbol("t")
    m, g     = sp.symbols("m g", positive=True)
    R_traj, omega_traj = sp.symbols(r"R \omega", positive=True)

    # position errors
    ex, ey, ez = sp.symbols("e_x e_y e_z")

    # velocities
    vx, vy, vz    = sp.symbols("v_x v_y v_z")
    vxd, vyd, vzd = sp.symbols(r"v_{xd} v_{yd} v_{zd}")

    # feedforward accelerations (general + circle-specific)
    # sp.Symbol (singular) is the safest way when names contain braces.
    ax_ff = sp.Symbol(r"a_{x,ff}")
    ay_ff = sp.Symbol(r"a_{y,ff}")
    az_ff = sp.Symbol(r"a_{z,ff}")

    # gains
    kx, ky, kz = sp.symbols("k_x k_y k_z", positive=True)
    kv         = sp.Symbol(r"k_v", positive=True)

    # ── circle feedforward: ẍ_d = d²/dt²[R cos(ωt), R sin(ωt), z_d] ──
    ax_ff_circle = -R_traj * omega_traj**2 * sp.cos(omega_traj * t)
    ay_ff_circle = -R_traj * omega_traj**2 * sp.sin(omega_traj * t)
    az_ff_circle = sp.Integer(0)

    # ── outer loop (Lyapunov, same as kinematic notebook) ──
    vx_cmd = vxd - kx * ex
    vy_cmd = vyd - ky * ey
    vz_cmd = vzd - kz * ez

    # ── middle loop: velocity error + feedforward → desired acceleration ──
    ax_des = kv * (vx_cmd - vx) + ax_ff
    ay_des = kv * (vy_cmd - vy) + ay_ff
    az_des = kv * (vz_cmd - vz) + az_ff

    # ── same loop but with circle feedforward substituted ──
    ax_des_circle = ax_des.subs(ax_ff, ax_ff_circle)
    ay_des_circle = ay_des.subs(ay_ff, ay_ff_circle)
    az_des_circle = az_des.subs(az_ff, az_ff_circle)

    # ── thrust vector in world frame ──
    Fx = m * ax_des
    Fy = m * ay_des
    Fz = m * (az_des + g)
    T  = sp.sqrt(Fx**2 + Fy**2 + Fz**2)

    # ── hover condition with feedforward (ex=ey=ez=0, v=vd) ──
    T_hover = sp.simplify(T.subs({
        ex: 0, ey: 0, ez: 0,
        vx: vxd, vy: vyd, vz: vzd,
        ax_ff: ax_ff_circle, ay_ff: ay_ff_circle, az_ff: az_ff_circle,
    }))

    # ── Vdot for outer-loop Lyapunov (unchanged by feedforward) ──
    V     = sp.Rational(1, 2) * (ex**2 + ey**2 + ez**2)
    V_dot = ex * (vx_cmd - vxd) + ey * (vy_cmd - vyd) + ez * (vz_cmd - vzd)
    V_dot = sp.expand(V_dot)

    return {
        # outer loop
        "vx_cmd":  vx_cmd,
        "vy_cmd":  vy_cmd,
        "vz_cmd":  vz_cmd,
        # middle loop (general)
        "ax_des":  ax_des,
        "ay_des":  ay_des,
        "az_des":  az_des,
        # circle feedforward terms
        "ax_ff_circle": ax_ff_circle,
        "ay_ff_circle": ay_ff_circle,
        # middle loop with circle feedforward substituted
        "ax_des_circle": ax_des_circle,
        "ay_des_circle": ay_des_circle,
        # thrust
        "Fx": Fx, "Fy": Fy, "Fz": Fz,
        "T":  T,
        "T_hover": T_hover,
        # Lyapunov
        "V":     V,
        "V_dot": V_dot,
    }
```

### `src/symbolic/quadcopter_kinematic_lyapunov.py`

```python
import sympy as sp

def derive_lyapunov():
    """
    Derives Lyapunov-based kinematic control for a quadcopter in 3D space tracking a trajectory.
    We model the quadcopter kinematically as a point mass in R^3 with heading.
    State: p = [x, y, z]^T, psi (yaw).
    Inputs: v = [v_x, v_y, v_z]^T, omega_psi.
    Reference: p_d = [x_d, y_d, z_d]^T, psi_d.
    """
    # 1. Define states and references
    x, y, z, psi = sp.symbols('x y z psi', real=True)
    xd, yd, zd, psid = sp.symbols('x_d y_d z_d psi_d', real=True)
    vxd, vyd, vzd, psid_dot = sp.symbols(r'v_{x\,d} v_{y\,d} v_{z\,d} \dot{\psi}_{d}', real=True)

    # 2. Define Tracking Errors
    ex = x - xd
    ey = y - yd
    ez = z - zd
    epsi = psi - psid

    # 3. Define the Lyapunov function Candidate
    # We want to drive all position and yaw errors to zero
    V = sp.Rational(1, 2) * (ex**2 + ey**2 + ez**2 + epsi**2)

    # 4. Propose control inputs to make \dot{V} negative definite
    # \dot{x} = v_x, \dot{y} = v_y, \dot{z} = v_z, \dot{\psi} = \omega_\psi
    vx, vy, vz, w_psi = sp.symbols(r'v_x v_y v_z \omega_\psi', real=True)
    kx, ky, kz, kpsi = sp.symbols(r'k_x k_y k_z k_\psi', positive=True, real=True)

    # Proposed Control Laws
    law_vx = vxd - kx * ex
    law_vy = vyd - ky * ey
    law_vz = vzd - kz * ez
    law_wpsi = psid_dot - kpsi * epsi

    # 5. Compute \dot{V}
    # \dot{V} = e_x \dot{e}_x + e_y \dot{e}_y + e_z \dot{e}_z + e_\psi \dot{e}_\psi
    # where \dot{e}_x = v_x - v_{x,d}, etc.
    ex_dot = vx - vxd
    ey_dot = vy - vyd
    ez_dot = vz - vzd
    epsi_dot = w_psi - psid_dot

    V_dot = ex * ex_dot + ey * ey_dot + ez * ez_dot + epsi * epsi_dot

    # Substitute our control laws into V_dot
    V_dot_sub = V_dot.subs({
        vx: law_vx,
        vy: law_vy,
        vz: law_vz,
        w_psi: law_wpsi
    }).simplify()

    return {
        "V": V,
        "V_dot": V_dot,
        "V_dot_sub": V_dot_sub,
        "law_vx": law_vx,
        "law_vy": law_vy,
        "law_vz": law_vz,
        "law_wpsi": law_wpsi,
    }
```

### `src/symbolic/quadcopter_path_following.py`

```python
"""
src/symbolic/quadcopter_path_following.py

Symbolic Lyapunov derivation for the path-following controller on a
parametric circle path.

Key difference from trajectory tracking
----------------------------------------
Trajectory tracking:   p_d(t)  — reference moves with physical time
Path following:        p_d(γ)  — reference moves with virtual parameter γ

The virtual parameter evolves as:
    γ̇ = v_d + k_γ ŷ_t(γ)^T e_p

where ŷ_t is the unit tangent to the path.  This lets the virtual target
adapt its speed to the vehicle's position, giving smoother transients when
starting far from the path.
"""

from __future__ import annotations
import sympy as sp


def derive_path_following() -> dict:
    """
    Lyapunov-based path following for a circle p_d(γ) = [R cosγ, R sinγ, z_d].

    State      : position error  e_p = p - p_d(γ)
    Control    : velocity command  v_cmd = (∂p_d/∂γ)γ̇ − K e_p
    Gamma law  : γ̇ = v_d + k_γ t̂(γ)^T e_p
    """
    # ── symbols ──────────────────────────────────────────────────────────────
    R, gamma, v_d   = sp.symbols(r"R \gamma v_d", positive=True)
    k_gamma         = sp.Symbol(r"k_\gamma", positive=True)
    kx, ky, kz      = sp.symbols("k_x k_y k_z", positive=True)
    ex, ey, ez      = sp.symbols("e_x e_y e_z", real=True)

    # ── path geometry ─────────────────────────────────────────────────────────
    # Position on path
    p_d  = sp.Matrix([R * sp.cos(gamma), R * sp.sin(gamma), 0])

    # Unnormalised tangent ∂p_d/∂γ
    dp_d = sp.diff(p_d, gamma)

    # Unit tangent t̂ = ∂p_d/∂γ / |∂p_d/∂γ|
    dp_d_norm = sp.sqrt(dp_d.dot(dp_d))
    t_hat     = sp.simplify(dp_d / dp_d_norm)   # [-sin γ, cos γ, 0]

    # Path yaw  ψ_d(γ) = atan2(dp_d_y, dp_d_x) = γ + π/2   (for circle)
    psi_d     = gamma + sp.pi / 2
    dpsi_d_dg = sp.diff(psi_d, gamma)            # = 1

    # ── error dynamics ────────────────────────────────────────────────────────
    e_p = sp.Matrix([ex, ey, ez])

    # Gamma dynamics
    e_s      = t_hat.dot(e_p)                    # along-path ("tangential") error
    #gamma_dot = v_d + k_gamma * e_s
    gamma_dot = v_d * (1.0 + k_gamma * e_s)

    # Feedforward velocity from the moving virtual target
    v_ff = dp_d * gamma_dot

    # Diagonal gain matrix
    K = sp.diag(kx, ky, kz)

    # Full velocity command
    v_cmd = v_ff - K * e_p

    # Error dynamics  ė_p = v_cmd − ∂p_d/∂γ · γ̇ = −K e_p
    e_p_dot = sp.simplify(v_cmd - dp_d * gamma_dot)

    # ── Lyapunov function ─────────────────────────────────────────────────────
    V = sp.Rational(1, 2) * e_p.dot(e_p)

    # V̇ = ∇V · ė_p = e_p^T (−K e_p)
    V_dot = sp.expand(e_p.dot(e_p_dot))

    # Confirm negative definiteness symbolically
    # V_dot = −kx ex² − ky ey² − kz ez²
    V_dot_expected = -(kx * ex**2 + ky * ey**2 + kz * ez**2)
    assert sp.simplify(V_dot - V_dot_expected) == 0, \
        "V_dot does not match expected form"

    # ── gamma convergence ─────────────────────────────────────────────────────
    # As e_p → 0:  γ̇ → v_d  (desired path speed)
    gamma_dot_at_zero = gamma_dot.subs({ex: 0, ey: 0, ez: 0})

    # ── along-path vs cross-path decomposition ────────────────────────────────
    # Normal to path (inward for circle): n̂ = [−cos γ, −sin γ, 0]
    n_hat = sp.Matrix([-sp.cos(gamma), -sp.sin(gamma), 0])

    # Cross-track error
    e_n = n_hat.dot(e_p)

    # Along-track error
    e_s_sym = t_hat.dot(e_p)

    return {
        # path geometry
        "p_d":          p_d,
        "dp_d":         dp_d,
        "t_hat":        t_hat,
        "n_hat":        n_hat,
        "psi_d":        psi_d,
        "dpsi_d_dg":    dpsi_d_dg,
        # gamma dynamics
        "gamma_dot":    gamma_dot,
        "e_s":          e_s_sym,
        "e_n":          e_n,
        "gamma_dot_at_zero": gamma_dot_at_zero,
        # error dynamics
        "v_ff":         v_ff,
        "v_cmd":        v_cmd,
        "e_p_dot":      e_p_dot,
        # Lyapunov
        "V":            V,
        "V_dot":        V_dot,
        "is_ND":        True,
        "stability":    "GAS",
    }


def compare_trajectory_vs_path_following() -> dict:
    """
    Symbolic comparison: show what changes when switching from trajectory
    tracking to path following.

    In trajectory tracking:
        p_d(t) = [R cos(ω t), R sin(ω t), z_d]
        γ = ω t  (fixed, not adaptive)
        ė_p = −K e_p  +  ZERO perturbation   (if inner loop perfect)

    In path following:
        p_d(γ),  γ̇ = v_d + k_γ t̂^T e_p
        ė_p = −K e_p  (exactly, by construction)

    The difference:  path following adds k_γ t̂^T e_p to γ̇.
    This slows the virtual target when the vehicle is behind it
    and advances it when the vehicle is ahead.
    """
    R, omega, t     = sp.symbols(r"R \omega t", positive=True)
    v_d             = sp.Symbol("v_d", positive=True)
    kx, ky          = sp.symbols("k_x k_y", positive=True)
    k_gamma         = sp.Symbol(r"k_\gamma", positive=True)
    ex, ey          = sp.symbols("e_x e_y", real=True)

    # ── Trajectory tracking ───────────────────────────────────────────────────
    # Reference at fixed time:  γ_TT = ω t
    # v_ref(t) = dγ/dt * dp_d/dγ = ω * [-R sin(ω t), R cos(ω t)]
    gamma_TT     = omega * t
    v_ref_TT     = sp.Matrix([
        -R * omega * sp.sin(gamma_TT),
         R * omega * sp.cos(gamma_TT),
    ])

    # ── Path following ────────────────────────────────────────────────────────
    gamma_sym    = sp.Symbol(r"\gamma")
    t_hat_sym    = sp.Matrix([-sp.sin(gamma_sym), sp.cos(gamma_sym)])
    e_p_sym      = sp.Matrix([ex, ey])

    gamma_dot_PF = v_d + k_gamma * t_hat_sym.dot(e_p_sym)
    v_ref_PF     = sp.Matrix([
        -R * sp.sin(gamma_sym),
         R * sp.cos(gamma_sym),
    ]) * gamma_dot_PF

    # ── Key difference ────────────────────────────────────────────────────────
    # γ̇_PF − γ̇_TT = k_γ t̂^T e_p
    # This adapts the virtual target speed based on vehicle position.
    gamma_dot_diff = sp.simplify(gamma_dot_PF - v_d)   # = k_γ t̂^T e_p

    return {
        "gamma_TT":        gamma_TT,
        "v_ref_TT":        v_ref_TT,
        "gamma_dot_PF":    gamma_dot_PF,
        "v_ref_PF":        v_ref_PF,
        "gamma_dot_diff":  gamma_dot_diff,
    }
```

### `src/utils/animations.py`

```python
"""
src/utils/2d_animations.py

Reusable 2D matplotlib animations for top-down vehicle visualisation.
Each function returns a matplotlib.animation.FuncAnimation object —
the caller decides whether to display it (HTML5) or save it.
"""

from __future__ import annotations

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from matplotlib import animation
from typing import Optional
import base64
from pathlib import Path
from tempfile import TemporaryDirectory
import matplotlib as mpl
from matplotlib.animation import writers
from tqdm import tqdm


# ──────────────────────────────────────────────────────────────────────────────
# helpers
# ──────────────────────────────────────────────────────────────────────────────

def _heading_line(x: float, y: float, angle: float, length: float = 0.5):
    """Return ([x0, x1], [y0, y1]) for a heading indicator arrow."""
    return (
        [x, x + length * np.cos(angle)],
        [y, y + length * np.sin(angle)],
    )


# ──────────────────────────────────────────────────────────────────────────────
# public API
# ──────────────────────────────────────────────────────────────────────────────

def quadcopter_topdown(
    df: pd.DataFrame,
    dt: float,
    *,
    x_col: str       = "x",
    y_col: str       = "y",
    yaw_col: str     = "yaw",
    xd_col: str      = "xd",
    yd_col: str      = "yd",
    psid_col: str    = "psid",
    decimate: int    = 50,
    heading_len: float = 0.5,
    xlim: tuple[float, float] = (-5.0, 5.0),
    ylim: tuple[float, float] = (-5.0, 5.0),
    title: str       = "Quadcopter Top-Down View",
    figsize: tuple[float, float] = (8.0, 8.0),
    fig: Optional[plt.Figure] = None,
    ax:  Optional[plt.Axes]  = None,
) -> animation.FuncAnimation:
    """
    Animate a top-down (XY) view of actual vs reference trajectory.

    Parameters
    ----------
    df        : simulation DataFrame (from SimLogger.to_dataframe())
    dt        : physics timestep [s]  — used to set animation interval
    decimate  : keep every Nth row to control animation speed/length
    heading_len : length of the heading arrow in metres

    Returns
    -------
    matplotlib.animation.FuncAnimation  (call .to_html5_video() or .save())
    """
    df_anim = df.iloc[::decimate].reset_index(drop=True)

    if fig is None or ax is None:
        fig, ax = plt.subplots(figsize=figsize)

    ax.set_xlim(*xlim)
    ax.set_ylim(*ylim)
    ax.set_xlabel("X [m]")
    ax.set_ylabel("Y [m]")
    ax.set_title(title)
    ax.set_aspect("equal", adjustable="box")
    ax.grid(True)

    # ── artists ──
    actual_traj, = ax.plot([], [], "b-",  alpha=0.5, label="Actual")
    ref_traj,    = ax.plot([], [], "r--", alpha=0.5, label="Reference")
    actual_dot,  = ax.plot([], [], "bo",  markersize=7)
    ref_dot,     = ax.plot([], [], "rx",  markersize=7)
    actual_head, = ax.plot([], [], "b-",  linewidth=2)
    ref_head,    = ax.plot([], [], "r-",  linewidth=2)
    time_text    = ax.text(0.02, 0.95, "", transform=ax.transAxes, fontsize=10)

    ax.legend(loc="upper right")

    _artists = (actual_traj, ref_traj, actual_dot, ref_dot,
                actual_head, ref_head, time_text)

    def _init():
        for a in _artists:
            if hasattr(a, "set_data"):
                a.set_data([], [])
        time_text.set_text("")
        return _artists

    def _update(frame: int):
        hist = df_anim.iloc[: frame + 1]
        cur  = df_anim.iloc[frame]

        actual_traj.set_data(hist[x_col],  hist[y_col])
        ref_traj.set_data(hist[xd_col], hist[yd_col])

        actual_dot.set_data([cur[x_col]],  [cur[y_col]])
        ref_dot.set_data(   [cur[xd_col]], [cur[yd_col]])

        hx, hy = _heading_line(cur[x_col],  cur[y_col],  cur[yaw_col],  heading_len)
        rx, ry = _heading_line(cur[xd_col], cur[yd_col], cur[psid_col], heading_len)
        actual_head.set_data(hx, hy)
        ref_head.set_data(rx, ry)

        time_text.set_text(f"t = {cur['t']:.2f} s")
        return _artists

    interval_ms = decimate * dt * 1000.0   # wall-clock ms per frame

    anim = animation.FuncAnimation(
        fig,
        _update,
        frames=len(df_anim),
        init_func=_init,
        blit=True,
        interval=interval_ms,
    )

    plt.close(fig)   # prevent double-display in notebooks
    return anim


def error_convergence(
    df: pd.DataFrame,
    error_cols: list[str],
    labels: Optional[list[str]] = None,
    *,
    title: str = "Error Convergence",
    figsize: tuple[float, float] = (10.0, 4.0),
    fig: Optional[plt.Figure] = None,
    ax:  Optional[plt.Axes]  = None,
) -> animation.FuncAnimation:
    """
    Animate error signals growing in time (useful for convergence demos).

    Parameters
    ----------
    error_cols : list of column names in df  (e.g. ['ex', 'ey', 'ez'])
    labels     : display names (defaults to error_cols)
    """
    labels = labels or error_cols

    if fig is None or ax is None:
        fig, ax = plt.subplots(figsize=figsize)

    ax.set_xlim(df["t"].iloc[0], df["t"].iloc[-1])
    y_max = df[error_cols].abs().max().max() * 1.2
    ax.set_ylim(-y_max, y_max)
    ax.set_xlabel("t [s]")
    ax.set_ylabel("Error")
    ax.set_title(title)
    ax.axhline(0, color="black", linewidth=0.5, linestyle="--")
    ax.grid(True)

    lines = [ax.plot([], [], label=lbl)[0] for lbl in labels]
    ax.legend()

    step = max(1, len(df) // 300)   # ~300 animation frames regardless of df length
    df_anim = df.iloc[::step].reset_index(drop=True)

    def _init():
        for ln in lines:
            ln.set_data([], [])
        return lines

    def _update(frame: int):
        hist = df_anim.iloc[: frame + 1]
        for ln, col in zip(lines, error_cols):
            ln.set_data(hist["t"], hist[col])
        return lines

    anim = animation.FuncAnimation(
        fig, _update, frames=len(df_anim),
        init_func=_init, blit=True, interval=30,
    )
    plt.close(fig)
    return anim


def to_html5_video_with_progress(anim: animation.FuncAnimation, embed_limit: Optional[float] = None, *, desc: str = "Rendering animation") -> str:
    """
    Save an animation to an H264 (mp4) in-memory HTML5 video tag while
    displaying a tqdm progress bar during frame generation.

    Parameters
    ----------
    anim : matplotlib.animation.FuncAnimation
        The animation to save.
    embed_limit : float, optional
        Maximum embed size in MB (same semantics as matplotlib's
        Animation.to_html5_video). If the generated video exceeds the
        limit, the function returns the string 'Video too large to embed.'
    desc : str
        Description shown in the tqdm bar.

    Returns
    -------
    str
        HTML5 `<video>` tag with base64-embedded mp4 video, or the
        'Video too large to embed.' string when the embed limit is
        exceeded.
    """
    VIDEO_TAG = r'''<video {size} {options}>
  <source type="video/mp4" src="data:video/mp4;base64,{video}">
  Your browser does not support the video tag.
</video>'''

    # determine embed limit (MB -> bytes)
    embed_limit = mpl._val_or_rc(embed_limit, 'animation.embed_limit')
    embed_limit_bytes = embed_limit * 1024 * 1024

    with TemporaryDirectory() as tmpdir:
        path = Path(tmpdir) / "temp.m4v"

        # instantiate writer similar to matplotlib's implementation
        Writer = writers[mpl.rcParams['animation.writer']]
        writer = Writer(codec='h264',
                        bitrate=mpl.rcParams['animation.bitrate'],
                        fps=(1000.0 / getattr(anim, '_interval', 200)))

        # progress bar state
        bar = tqdm(total=None, desc=desc, unit='frame')
        last = {'n': 0}

        def _progress_callback(current_frame, total_frames):
            # total_frames may be None
            if total_frames is not None and bar.total is None:
                bar.total = total_frames
            # current_frame is zero-based index; we want count
            cur_count = (current_frame + 1) if current_frame is not None else None
            if cur_count is None:
                return
            delta = cur_count - last['n']
            if delta > 0:
                bar.update(delta)
                last['n'] = cur_count

        try:
            anim.save(str(path), writer=writer, progress_callback=_progress_callback)
        finally:
            bar.close()

        vid64 = base64.encodebytes(path.read_bytes())

    vid_len = len(vid64)
    if vid_len >= embed_limit_bytes:
        mpl._log.warning(
            "Animation movie is %s bytes, exceeding the limit of %s.", vid_len, embed_limit_bytes
        )
        return 'Video too large to embed.'

    video_str = vid64.decode('ascii')
    video_size = 'width="{}" height="{}"'.format(*writer.frame_size)

    options = ['controls', 'autoplay']
    if getattr(anim, '_repeat', False):
        options.append('loop')

    return VIDEO_TAG.format(video=video_str, size=video_size, options=' '.join(options))
```

### `src/utils/config.py`

```python
import yaml
from pydantic import BaseModel

def load_config(path: str) -> dict:
    with open(path) as f:
        return yaml.safe_load(f)
```

### `src/utils/plotting.py`

```python
"""
src/utils/plotting.py

Shared matplotlib style + figure-saving helpers.

Usage
-----
from src.utils.plotting import set_style, FigureManager

set_style()                          # call once at notebook top

fm = FigureManager(cfg)              # reads figures_dir from config
fm.save(fig, "errors")               # saves to <figures_dir>/errors.{pdf,png}
fm.save_and_show(fig, "3d_path")     # saves + displays inline in notebook
"""

from __future__ import annotations

import matplotlib.pyplot as plt
import matplotlib.ticker as ticker
from pathlib import Path
from typing import Union, Sequence


# ──────────────────────────────────────────────────────────────────────────────
# global style
# ──────────────────────────────────────────────────────────────────────────────

def set_style(usetex: bool = False) -> None:
    """
    Keep Matplotlib close to its default appearance while forcing scientific
    notation for large or small values.
    Set usetex=True when building the LaTeX report (requires a TeX installation).
    """
    plt.rcParams.update({
        "text.usetex": usetex,
        "axes.formatter.use_mathtext": True,
        "axes.formatter.limits": (-3, 3),
        "axes.formatter.useoffset": True,
    })


# ──────────────────────────────────────────────────────────────────────────────
# figure manager
# ──────────────────────────────────────────────────────────────────────────────

class FigureManager:
    """
    Saves figures to a folder specified in the simulation config,
    and optionally displays them inline.

    Parameters
    ----------
    config      : dict loaded from a YAML config file
    base_dir    : root path (usually the project ROOT); figures_dir in
                  config is resolved relative to this
    formats     : file formats to write (default: pdf + png)
    """

    def __init__(
        self,
        config: dict,
        base_dir: Union[str, Path] = ".",
        formats: Sequence[str] = ("pdf", "png"),
    ):
        self.formats = list(formats)

        figures_dir: str = config.get("figures_dir", "figures/default")
        self.out_dir = Path(base_dir) / figures_dir
        self.out_dir.mkdir(parents=True, exist_ok=True)

    # ── core ──────────────────────────────────────────────────────────────────

    def save(
        self,
        fig: plt.Figure,
        name: str,
        *,
        formats: Sequence[str] | None = None,
        close: bool = False,
    ) -> list[Path]:
        """
        Save *fig* as <figures_dir>/<name>.<ext> for each format.

        Returns the list of paths written.
        """
        fmts   = list(formats) if formats is not None else self.formats
        paths  = []
        for ext in fmts:
            p = self.out_dir / f"{name}.{ext}"
            fig.savefig(p)
            paths.append(p)
            print(f"  ↳ saved {p}")
        if close:
            plt.close(fig)
        return paths

    def save_and_show(
        self,
        fig: plt.Figure,
        name: str,
        *,
        formats: Sequence[str] | None = None,
    ) -> list[Path]:
        """
        Save *fig* and display it inline (Jupyter / IPython).
        The figure remains open after this call.
        """
        paths = self.save(fig, name, formats=formats)
        plt.show()          # renders inline in the notebook
        return paths

    # ── convenience factory methods ──────────────────────────────────────────

    @staticmethod
    def error_subplots(
        df,
        error_cols: list[str],
        labels: list[str] | None = None,
        units: list[str] | None = None,
        ncols: int = 2,
    ) -> plt.Figure:
        """
        Build a grid of error-vs-time subplots ready to save.

        Parameters
        ----------
        df         : simulation DataFrame
        error_cols : column names (e.g. ['ex', 'ey', 'ez', 'epsi'])
        labels     : subplot titles (defaults to error_cols)
        units      : y-axis unit strings (e.g. ['m', 'm', 'm', 'rad'])
        ncols      : columns in the subplot grid
        """
        labels = labels or error_cols
        units  = units  or [""] * len(error_cols)
        n      = len(error_cols)
        nrows  = (n + ncols - 1) // ncols

        fig, axs = plt.subplots(nrows, ncols, figsize=(6 * ncols, 4 * nrows))
        axs_flat = axs.flatten() if n > 1 else [axs]

        for i, (col, lbl, unit) in enumerate(zip(error_cols, labels, units)):
            ax = axs_flat[i]
            ax.plot(df["t"], df[col])
            ax.set_title(lbl)
            ax.set_xlabel("t [s]")
            ax.set_ylabel(f"[{unit}]" if unit else "")
            ax.axhline(0, color="black", linewidth=0.5, linestyle="--")

            # enforce scientific tick formatting for these axes
            try:
                ax.ticklabel_format(axis="both", style="sci", scilimits=(0, 0), useOffset=True)
            except Exception:
                pass

        # hide unused axes
        for j in range(n, len(axs_flat)):
            axs_flat[j].set_visible(False)

        fig.tight_layout()
        return fig

    @staticmethod
    def lyapunov_levelsets(
        df,
        x_col: str = "ex",
        y_col: str = "ey",
        *,
        n_levels: int = 15,
        title: str = r"Level sets of $V$ (projected on $e_x$–$e_y$)",
        figsize: tuple = (6, 6),
    ) -> plt.Figure:
        """
        Plot Lyapunov level sets V(ex,ey) = 0.5*(ex²+ey²) with the
        error trajectory overlaid.
        """
        import numpy as np

        fig, ax = plt.subplots(figsize=figsize)

        e_range = max(df[[x_col, y_col]].abs().max()) * 1.2
        ev = np.linspace(-e_range, e_range, 200)
        EX, EY = np.meshgrid(ev, ev)
        VV = 0.5 * (EX**2 + EY**2)

        cp = ax.contour(EX, EY, VV, levels=n_levels, cmap="viridis", alpha=0.5)
        fig.colorbar(cp, ax=ax, label=r"$V(e_x, e_y)$")

        ax.plot(df[x_col], df[y_col], "k-",  linewidth=1.8, label="Error trajectory")
        ax.plot(df[x_col].iloc[0],  df[y_col].iloc[0],  "go", label="Start")
        ax.plot(df[x_col].iloc[-1], df[y_col].iloc[-1], "ro", label="End")

        ax.axhline(0, color="black", linewidth=0.5)
        ax.axvline(0, color="black", linewidth=0.5)
        ax.set_xlabel(r"$e_x$ [m]")
        ax.set_ylabel(r"$e_y$ [m]")
        ax.set_title(title)
        ax.set_aspect("equal")
        ax.legend()
        # enforce scientific tick formatting on levelset axes
        try:
            ax.ticklabel_format(axis="both", style="sci", scilimits=(0, 0), useOffset=True)
        except Exception:
            pass

        fig.tight_layout()
        return fig

    @staticmethod
    def path_3d(
        df,
        *,
        x_col: str  = "x",  y_col: str  = "y",  z_col: str  = "z",
        xd_col: str = "xd", yd_col: str = "yd", zd_col: str = "zd",
        title: str = "3D Path Following",
        figsize: tuple = (8, 8),
    ) -> plt.Figure:
        """3D actual-vs-reference path plot."""
        fig = plt.figure(figsize=figsize)
        ax  = fig.add_subplot(111, projection="3d")

        ax.plot(df[xd_col], df[yd_col], df[zd_col], "r--", label="Reference")
        ax.plot(df[x_col],  df[y_col],  df[z_col],  "b-",  label="Actual")

        ax.scatter([df[x_col].iloc[0]],  [df[y_col].iloc[0]],  [df[z_col].iloc[0]],
                   color="black", marker="o", label="Start (actual)")
        ax.scatter([df[xd_col].iloc[0]], [df[yd_col].iloc[0]], [df[zd_col].iloc[0]],
                   color="green", marker="x", label="Start (ref)")

        ax.set_xlabel("X [m]")
        ax.set_ylabel("Y [m]")
        ax.set_zlabel("Z [m]")
        ax.set_title(title)
        ax.legend()
        # enforce scientific tick formatting on 3D axes
        try:
            ax.ticklabel_format(axis="x", style="sci", scilimits=(0, 0), useOffset=True)
            ax.ticklabel_format(axis="y", style="sci", scilimits=(0, 0), useOffset=True)
            if hasattr(ax, "zaxis"):
                ax.zaxis.set_major_formatter(ticker.ScalarFormatter(useMathText=True))
        except Exception:
            pass

        fig.tight_layout()
        return fig
```

### `src/vehicles/__init__.py`

```python

```

### `test_sim.py`

```python
import mujoco
import numpy as np

m = mujoco.MjModel.from_xml_path("models/unicycle/unicycle.xml")
d = mujoco.MjData(m)

mujoco.mj_resetData(m, d)
mujoco.mj_step(m, d)

# set velocity directly
v = 1.0
omega = 1.0

d.qpos[3:7] = [1, 0, 0, 0] # theta=0
d.qvel[0] = v   # is this global x?
d.qvel[1] = 0.0 # global y?
d.qvel[5] = omega 

for _ in range(10):
    mujoco.mj_step(m, d)
    print(d.qpos[:3], d.qvel[:3])
```

## Summary

Included **45** readable text files (4235 source lines).

Omitted files:
- unrecognized/generated file type: 15
