### Speed–Torque Table

The CSV supplies one motor's shaft torque $T$ in N·m against motor speed $N$ in RPM. The solver sorts rows by speed and uses the slope between adjacent samples. Duplicate speeds are rejected.

#### Interpolation

For the segment between tabulated speeds $N_i$ and $N_{i+1}$,

$$
T(N)=T_i+
\frac{N-N_i}{N_{i+1}-N_i}(T_{i+1}-T_i).
$$

Below or above the table, the first or last segment is extended linearly. Negative torque results produce zero available drive torque in the planner.

#### Conversion to wheel force

Let $G$ be the fixed reduction ratio, $\eta$ drivetrain efficiency as a fraction, and $r_0$ free tire radius. The solver uses the same free-radius speed estimate for all four wheels.

$$
\begin{align*}
N &= \frac{60G\,\max(v,0)}{2\pi r_0},\\
T_w^{\mathrm{base}} &= \max(0,T(N)G\eta\,c_w),\\
F_w^{\mathrm{base}} &= T_w^{\mathrm{base}}/r_w.
\end{align*}
$$

The configured front or rear torque limit factor supplies $c_w\in[0,1]$. Force uses the loaded radius $r_w$ in the per-wheel model and the free radius in point-mass mode. For per-wheel grip, the planner reduces all four drive forces by the same factor to respect the first active wheel to reach its available longitudinal tire force. Point-mass mode limits the sum against whole-vehicle tire grip.
