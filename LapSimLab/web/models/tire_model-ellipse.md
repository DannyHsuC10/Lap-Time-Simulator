### Friction Ellipse

The ellipse represents the trade-off between cornering and longitudinal force. The JSON can define either fixed friction coefficients or the same load and camber fit used by the superellipse. The combined-force exponent is always two.

#### Pure force limits

With fixed coefficients `mu_x` and `mu_y`, let $\lambda$ be Tire Utilization divided by 100. The pure limits grow linearly with normal load.

$$
P_x=\lambda\mu_x F_z,\qquad
P_y=\lambda\mu_y F_z.
$$

For a fitted JSON, the solver clamps positive load and camber to their fit ranges. For $j\in\{x,y\}$, the same polynomial is evaluated with the corresponding coefficients, $d=(F_z-F_{z0})/F_{z0}$, and camber $\gamma$ in degrees.

$$
\begin{align*}
\mu_j(d)&=\mu_{0,j}(1+b_{1,j}d+b_{2,j}d^2),\\
G_j(\gamma)&=1+c_{2,j}\gamma^2+c_{4,j}\gamma^4,\\
P_j&=\lambda F_z\mu_j(d)G_j(\gamma).
\end{align*}
$$

#### Combined grip and rolling resistance

For $|F_y|\le P_y$ with positive $P_y$, the longitudinal interval is bounded by

$$
F_x^\pm=\pm P_x\sqrt{1-(F_y/P_y)^2}.
$$

When $|F_y|>P_y$, no longitudinal force can satisfy the requested cornering force. A wheel with zero normal load can supply only zero force. Fixed coefficients in point-mass mode are evaluated at full vehicle load and lateral demand. With fitted coefficients, the solver uses four identical tires at one-quarter of both quantities and zero nominal camber.

Rolling resistance is $C_{rr}F_z$ per wheel or $C_{rr}F_{z,\mathrm{tot}}$ in point-mass mode. It is separate from the grip envelope.
