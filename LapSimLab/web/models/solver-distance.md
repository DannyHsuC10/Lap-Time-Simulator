### Forward Distance Integration

This solver starts at the configured Initial Speed and advances once along the prescribed path. It uses the same cornering ceiling and quasi-steady tire, motor, brake and resistance limits as the iterative solver.

#### Local speed ceiling

The initial ceiling $V_i$ at each curved sample comes from testing $m v^2|\kappa_i|\le\beta F_{y,\mathrm{available}}$ by bisection, with lateral safety factor $\beta$. Straight samples use Speed Limit. Yaw inertia correction, when enabled, reduces this ceiling before integration.

**With single-pass braking lookahead**, the solver makes one backward sweep over the ceiling. At each segment it finds the largest admissible entry speed $u$ that can reach the downstream ceiling $C_{i+1}$ under the local braking capacity $b$.

$$
u^2-2b(u,\kappa_i)\Delta s_i\le C_{i+1}^2,\qquad
C_i=\min(V_i,u).
$$

The sweep begins with $C_{\mathrm{end}}=V_{\mathrm{end}}$. The forward pass uses $C_{i+1}^2$ as its target.

**Without lookahead**, the solver uses the next local minimum of the ceiling, or the final sample when no local minimum remains. With target index $j$ and current-speed braking capacity $b_i$, the target squared speed at the next sample is

$$
U_{i+1}=\min\!
\left(V_{i+1}^2,\,
V_j^2+2b_i(s_j-s_{i+1})\right).
$$

#### Reachable forward step

For a segment with positive $\Delta s_i$, the solver evaluates drive capacity $a_i^+$ at $v_i$ and $\kappa_i$. The maximum reachable squared speed is $v_i^2+2a_i^+\Delta s_i$. If the target squared speed $U_{i+1}$ is below $v_i^2$, braking sets the minimum reachable squared speed to $\max(v_i^2-2b_i\Delta s_i,0)$. Otherwise the minimum is zero.

$$
v_{i+1}=\sqrt{\operatorname{clip}
\left(U_{i+1},v_{\min}^2,v_{\max}^2\right)}.
$$

For effectively zero distance the solver carries forward $v_i$. If the vehicle cannot slow enough, the reachable speed is kept. A result sample above its lateral ceiling by more than $0.06\,\mathrm{m/s}$ is flagged.

#### Lap time

Each positive-length segment contributes $\Delta t_i=2\Delta s_i/(v_i+v_{i+1})$ and the lap time is their sum. A positive-length segment with effectively zero speed at both ends makes the lap infeasible. Maximum Passes applies to the iterative lap solver only.
