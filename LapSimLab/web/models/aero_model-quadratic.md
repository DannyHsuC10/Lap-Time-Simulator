### Constant-Coefficient Aerodynamics

The model fixes the aero map at zero heave, pitch and roll. The coefficients `CL0` and `CD0` apply at every attitude, so vehicle speed is the only changing input in the force equations.

#### Dynamic pressure and forces

Let $q=\rho v^2/2$ be dynamic pressure, $\rho$ air density and $S$ reference area.

$$
D=-qSC_{L0},\qquad
F_{\mathrm{drag}}=qSC_{D0}.
$$

Negative $C_{L0}$ gives positive downforce. Doubling speed multiplies both forces by four at fixed coefficients. If the JSON omits density, area, front fraction and center-of-pressure height, the defaults are $1.225\,\mathrm{kg/m^3}$, $1.2\,\mathrm{m^2}$, $0.45$ and $0\,\mathrm{m}$ respectively.

#### Balance and solver effect

With front fraction $f_0$, the axle downforces are $f_0D$ and $(1-f_0)D$, split equally left to right. For nonzero downforce, the center of pressure relative to the vehicle midpoint is

$$
x_{\mathrm{COP}}=L(f_0-\tfrac12),\qquad
y_{\mathrm{COP}}=0.
$$

The vertical coordinate is `cop_z`. The load solver adds total downforce to vehicle weight. In per-wheel mode that total is allocated with the static front weight fraction, independently of the reported aero balance. Drag subtracts from available drive acceleration and adds to braking deceleration.
