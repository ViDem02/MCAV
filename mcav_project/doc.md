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