### Equivalent Mechanical Advantage

The model represents the entire brake mechanism by an equivalent force multiplication. Let $F_p$ be pedal force, $A$ mechanical advantage, $r_0$ free tire radius, and $b$ the front brake bias.

#### From pedal force to brake torque

First compute the equivalent total braking force and total torque magnitude.

$$
F_{\mathrm{eq}}=F_pA,\qquad
T_{\mathrm{tot}}=r_0F_{\mathrm{eq}}.
$$

The front axle gets the fraction $b$ and the rear gets $1-b$. Each axle divides its share equally between its two wheels. Brake torques oppose forward motion.

$$
T_{f,w}=-\frac{bT_{\mathrm{tot}}}{2},\qquad
T_{r,w}=-\frac{(1-b)T_{\mathrm{tot}}}{2}.
$$

#### Tire-limited braking

The signed force at each wheel is $F_w=T_w/r_w$, using its loaded radius in the per-wheel model. The solver scales all brake forces together when a wheel cannot transmit its share at the requested lateral force. If $B_w$ is the available tire braking magnitude and $P_w=|T_w|/r_w>0$ is the full-pedal magnitude, the common multiplier is

$$
c=\operatorname{clip}
\left(\min_{w:P_w>0}\frac{B_w}{P_w},\,0,\,1\right).
$$

Under the point-mass model the same torques are converted using $r_0$, then the total is capped by whole-vehicle grip. Aerodynamic drag and rolling resistance add to deceleration in the solver.
