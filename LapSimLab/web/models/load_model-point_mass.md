### Point Mass (Total Vehicle Grip)

The solver uses a single mass following the sampled path. Aero is evaluated at zero heave, pitch and roll. With positive downforce $D$, the total normal load and signed lateral force demand are

$$
Z=\max(mg+D,0),\qquad
F_y=mv^2\kappa.
$$

#### Whole-vehicle grip

The fixed-coefficient friction ellipse uses $Z$ and $F_y$ directly. For fitted and tabulated tires, the solver uses four identical tires at load $Z/4$ and nominal zero camber. This preserves the nonlinear load sensitivity in the tire fit while leaving out longitudinal and lateral transfer.

$$
F_{x,\mathrm{tot}}^\pm=
4\,F_{x,\mathrm{tire}}^\pm(Z/4,F_y/4,0).
$$

For the cornering-only speed search, the pure lateral peak is summed in the same way. For a combined-force request outside the tire envelope, the longitudinal tire bounds are treated as zero.

#### Longitudinal acceleration

Let $P^+$ be total motor-limited drive force and $P^-$ the signed brake limit. Both use the free tire radius. With total resistance $R=D_{\mathrm{drag}}+F_{rr}$,

$$
\begin{align*}
a^+&=\max\!\left(0,\frac{\min(F_x^+,P^+)-R}{m}\right),\\
b&=\max\!\left(0,\frac{-\max(F_x^-,P^-)+R}{m}\right).
\end{align*}
$$

The implementation caps and rounds these capacities down on the same $35/512\,\mathrm{m/s^2}$ grid used by the four-wheel acceleration search. For the ellipse and superellipse $F_{rr}=C_{rr}Z$. With the table, $F_{rr}=4M_{rr}(Z/4)/r_0$ at positive load.

Per-wheel loads, individual wheel torques and suspension displacement channels are zero in this mode. Yaw inertia correction requires per-wheel loads and is unavailable.
