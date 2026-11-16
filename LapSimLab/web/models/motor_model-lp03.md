### LP03 Voltage Envelope

The CSV columns `vdc_V`, `rpm` and `T_shaft_Nm` define maximum shaft torque versus RPM at discrete DC bus voltages. The selected voltage stays fixed throughout the lap.

#### Interpolation across RPM and voltage

For voltage $V$ between table voltages $V_a$ and $V_b$, interpolate the torque separately along the RPM axis of each voltage curve. The RPM grids may differ. On each curve, queries below its first speed keep its first torque, and queries above its last speed return zero. A voltage outside the tabulated range is invalid.

$$
\begin{align*}
w&=\frac{V-V_a}{V_b-V_a},\\
T(N,V)&=(1-w)T_a(N)+wT_b(N).
\end{align*}
$$

At an exact table voltage the solver uses that voltage curve directly. The envelope is a steady input without state of charge, thermal derating or voltage sag updates.

#### Conversion to wheel force

With free tire radius $r_0$, reduction ratio $G$, drivetrain efficiency $\eta$, and axle torque factor $c_w$, the planner calculates

$$
\begin{align*}
N &= \frac{60G\,\max(v,0)}{2\pi r_0},\\
F_w^{\mathrm{base}} &=
\frac{\max(0,T(N,V)G\eta\,c_w)}{r_w}.
\end{align*}
$$

Here $r_w$ is the loaded radius for an individual wheel and $r_0$ in point-mass mode. In the per-wheel model, one common scale factor reduces the drive forces to satisfy all tires while retaining the torque split. In point-mass mode, the sum is capped by the whole-vehicle longitudinal grip.
