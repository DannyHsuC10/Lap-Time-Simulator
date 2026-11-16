### Aerodynamics Disabled

The solver bypasses the aerodynamic coefficient file. Downforce, drag, the reported coefficients and the center of pressure are set to zero.

$$
D=F_{\mathrm{drag}}=0,\qquad C_L=C_D=0.
$$

#### Effect on load and speed

The point-mass model's normal load becomes $mg$. In the per-wheel model the initial axle loads are determined by weight and static weight distribution, then adjusted for acceleration and cornering.

$$
\begin{align*}
F_{z,f}&=mg\,\frac{l_r}{L}-\frac{ma_xh}{L},\\
F_{z,r}&=mg\,\frac{l_f}{L}+\frac{ma_xh}{L}.
\end{align*}
$$

The per-wheel model still applies lateral load transfer and tire compression. Motor torque limits, brake torque limits and rolling resistance are still included. In the longitudinal balance the resistance term reduces to $F_{rr}$.
