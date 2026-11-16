### Hydraulic Brake

Pedal leverage, balance-bar split, master cylinders, caliper pistons and effective disc radius determine the brake torque independently for each axle. Let $F_p$ be pedal force, $A_p$ pedal ratio, and $b_i$ the fraction of master-cylinder input force for axle $i$. The front axle uses the configured bias $b_f$ and the rear uses $b_r=1-b_f$.

#### Hydraulic pressure and wheel torque

With master cylinder diameter $d_m$, the axle line pressure is

$$
p_i=\frac{F_pA_p b_i}{\pi d_m^2/4}.
$$

One wheel on axle $i$ has $n_i$ caliper pistons of diameter $d_i$. The model multiplies pressure by the total piston area, then applies the pad friction coefficient $\mu_p$ and effective disc radius $d_{\mathrm{disc}}/2$.

$$
\begin{align*}
F_{\mathrm{clamp},i}&=p_i n_i\frac{\pi d_i^2}{4},\\
T_{i,w}&=-F_{\mathrm{clamp},i}\,\mu_p\,
\frac{d_{\mathrm{disc}}}{2}.
\end{align*}
$$

The piston count is per wheel, so each of the two wheels on an axle receives the calculated torque. The balance-bar setting splits master-cylinder input force. Different piston sizes or counts can therefore change the final front-to-rear torque split.

#### Tire-limited brake force

The solver divides $T_{i,w}$ by loaded wheel radius to get signed longitudinal force. It then applies a common factor between zero and one so no active wheel exceeds its available braking force at the assigned lateral force. Point-mass mode uses the free radius and caps the sum against the whole-vehicle tire envelope.
