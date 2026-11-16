### Attitude-Dependent Aerodynamics

The modal model evaluates the JSON map at heave $q_h$ in metres and pitch $q_p$ and roll $q_r$ in radians. Heave is positive downward. Positive pitch raises the nose and positive roll raises the left side.

#### Coefficients and forces

The lift coefficient includes absolute-value and quadratic sensitivities. The drag coefficient includes absolute-value sensitivities. Subscripts on the coefficients correspond to the heave, pitch and roll keys in the JSON.

$$
\begin{align*}
C_L &= C_{L0}+K_h|q_h|+K_p|q_p|+K_r|q_r|\\
&\quad+K_{hh}q_h^2+K_{pp}q_p^2+K_{rr}q_r^2,\\
C_D &= C_{D0}+K_{d,h}|q_h|+K_{d,p}|q_p|\\
&\quad+K_{d,r}|q_r|.
\end{align*}
$$

For air density $\rho$, reference area $S$, and speed $v$, the solver uses $C_L<0$ for downforce.

$$
D=-\tfrac12\rho S v^2 C_L,\qquad
F_{\mathrm{drag}}=\tfrac12\rho S v^2 C_D.
$$

#### Balance and center of pressure

The front and left fractions are $f=f_0+K_{b,p}q_p$ and $\ell=0.5+K_{b,r}q_r$. The model uses them directly. Downforce is divided as follows.

$$
\begin{align*}
D_{FL}&=Df\ell,& D_{FR}&=Df(1-\ell),\\
D_{RL}&=D(1-f)\ell,& D_{RR}&=D(1-f)(1-\ell).
\end{align*}
$$

For nonzero $D$, the center of pressure is the downforce-weighted average of wheel positions measured from the vehicle midpoint. The longitudinal coordinates are $\pm L/2$ and the lateral coordinates are $\pm\bar t/2$, where $\bar t$ is the mean of front and rear track width.

$$
x_{\mathrm{COP}}=L(f-\tfrac12),\qquad
y_{\mathrm{COP}}=\bar t(\ell-\tfrac12).
$$

When $D=0$, both reported horizontal coordinates are zero. The vertical coordinate is `cop_z`.

#### Coupling with the lap model

The per-wheel solver starts at zero attitude, estimates loads and attitude using total downforce, and recomputes aero. It performs three updates by default. It distributes total downforce for load transfer using the static front weight fraction. The four aerodynamic loads and center of pressure are calculated from the aero map for inspection and telemetry. Point-mass lap and GGV calculations evaluate the aero map at zero attitude; the standalone aero sweep evaluates the supplied attitude directly.
