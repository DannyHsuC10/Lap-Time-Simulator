### Fitted Superellipse

This model computes pure longitudinal and lateral force peaks for each loaded tire, then uses a superellipse to find how much longitudinal force remains at the wheel's assigned lateral force.

#### Load and camber fit

The JSON defines a reference load $F_{z0}$, valid load and camber ranges, and separate coefficients for the $x$ and $y$ axes. The solver clamps positive $F_z$ and camber $\gamma$ to the fit ranges. Use $\gamma$ in degrees. Let $\lambda$ be Tire Utilization divided by 100 and $d=(F_z-F_{z0})/F_{z0}$. For either direction $j\in\{x,y\}$,

$$
\begin{align*}
\mu_j(d)&=\mu_{0,j}(1+b_{1,j}d+b_{2,j}d^2),\\
G_j(\gamma)&=1+c_{2,j}\gamma^2+c_{4,j}\gamma^4,\\
P_j&=\lambda F_z\mu_j(d)G_j(\gamma).
\end{align*}
$$

#### Combined grip

The exponent $n\ge1$ comes from the tire JSON. The available forces satisfy

$$
\left|\frac{F_x}{P_x}\right|^n+
\left|\frac{F_y}{P_y}\right|^n\le1.
$$

For $|F_y|<P_y$, the two signed longitudinal bounds are

$$
F_x^\pm=\pm P_x
\left[1-\left(\frac{|F_y|}{P_y}\right)^n\right]^{1/n}.
$$

At $|F_y|=P_y$ the remaining longitudinal capacity is zero. A larger lateral demand makes the operating point infeasible. At zero utilization the only feasible force pair is zero.

The envelope is symmetric in drive and braking. The solver applies the motor and brake limits separately. Rolling resistance is $C_{rr}F_z$ per wheel using Fixed Rolling Resistance Coefficient. In the point-mass model, the fitted envelope is queried at one-quarter of the total normal load and lateral demand with zero nominal camber, then multiplied by four.
