### Iterative Lap Solver

The quasi-steady planner starts with a cornering speed ceiling, then alternates a forward acceleration pass and a backward braking pass. It lowers speeds where a local constraint is exceeded.

#### Cornering ceiling

For signed curvature $\kappa_i$ and candidate speed $v$, the lateral acceleration is $v^2\kappa_i$. The solver computes the current aero and normal loads at zero longitudinal acceleration. With lateral safety factor $\beta$, the candidate is feasible when

$$
m|v^2\kappa_i|\le
\beta F_{y,\mathrm{available}}(v,\kappa_i).
$$

The search bisects between zero and Speed Limit. It stops within $0.02\,\mathrm{m/s}$ or after 12 iterations. For $|\kappa_i|\le10^{-5}\,\mathrm{m^{-1}}$ the ceiling is Speed Limit. The optional yaw inertia correction is applied after this search.

#### Longitudinal capacity

At each trial speed, the planner uses $a_y=v^2\kappa$ and evaluates the remaining braking and drive force. In per-wheel mode, it bisects trial acceleration or deceleration over $[0,35]\,\mathrm{m/s^2}$ with a $0.1\,\mathrm{m/s^2}$ interval tolerance and a 10-iteration limit. Each trial updates the normal loads, available grip and rolling resistance. The drivetrain and brake distributions are scaled together on their respective branches to satisfy all active tires.

With drive force $F_{\mathrm{drive}}\ge0$, braking magnitude $B\ge0$, drag $D$ and rolling resistance $R$, the capacities checked against each trial are

$$
\begin{align*}
a_{\mathrm{cap}}&=\max(0,(F_{\mathrm{drive}}-D-R)/m),\\
b_{\mathrm{cap}}&=\max(0,(B+D+R)/m).
\end{align*}
$$

Point-mass mode instead evaluates the whole-vehicle grip and rounds the resulting capacity down to the search grid.

#### Forward and backward passes

Over sample spacing $\Delta s_i$, the forward pass clips the next speed by the reach of the current drive capacity $a_i^+$.

$$
v_{i+1}\leftarrow\min
\left(v_{i+1},\,\sqrt{v_i^2+2a_i^+\Delta s_i}\right).
$$

The backward pass searches for the largest entry speed $u$ up to the current ceiling with

$$
u^2-2b(u,\kappa_i)\Delta s_i\le v_{i+1}^2.
$$

A feasible upper bound is kept directly. Otherwise the search bisects to $0.06\,\mathrm{m/s}$ or 10 iterations. Each pass pair stops the iteration if the largest change is at most $0.05\,\mathrm{m/s}$, or the run ends at Maximum Passes with a convergence notice if needed. The first and last samples are processed independently, including for geometrically closed tracks. Initial Speed applies to the forward distance solver only.

#### Lap time

The solver divides each positive segment distance by the arithmetic mean of its endpoint speeds and sums the segment times.

$$
\Delta t_i=\frac{2\Delta s_i}{v_i+v_{i+1}},
\qquad t_{\mathrm{lap}}=\sum_i\Delta t_i.
$$

A positive-distance segment with effectively zero speed at both ends makes the lap infeasible. The Time Limit covers the solve from loading model data through preparing telemetry.
