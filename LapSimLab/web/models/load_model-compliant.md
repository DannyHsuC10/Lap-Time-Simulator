### Rigid Load Transfer with Tire Deflection

At each trial speed and acceleration, the model estimates four normal loads and body attitude. Let $L=l_f+l_r$ be wheelbase, $h$ CG height, $t_i$ axle track widths, and $D$ aerodynamic downforce. Axle index $i$ is front $f$ or rear $r$.

#### Normal loads

The model distributes the sum of weight and downforce using the static front fraction $p=l_r/L$. Positive $a_x$ transfers load to the rear.

$$
\begin{align*}
W&=mg+D,\qquad M_p=ma_xh,\\
Z_f&=pW-M_p/L,\\
Z_r&=(1-p)W+M_p/L.
\end{align*}
$$

Positive $a_y$ is a left turn and loads the right tires. Roll moment $M_r=ma_yh$ is split in proportion to the configured per-wheel roll stiffnesses $k_{r,i}$.

$$
\begin{align*}
\Delta Z_i&=\frac{M_r}{t_i}
\frac{k_{r,i}}{k_{r,f}+k_{r,r}},\\
Z_{i,L}&=Z_i/2-\Delta Z_i,\\
Z_{i,R}&=Z_i/2+\Delta Z_i.
\end{align*}
$$

The rear-to-front stiffness ratio scales both rear heave and rear roll stiffness. The superellipse branch floors each load at $1\,\mathrm{N}$. The ellipse and table branches set negative loads to zero and rescale the remaining loads to the original nonnegative sum.

#### Body attitude and tire radius

With axle heave stiffnesses $k_{h,i}$ and tire vertical stiffness $K_t$, suspension heave is positive downward. Positive roll raises the left side and positive pitch raises the nose.

$$
\begin{align*}
q_h &= D/(k_{h,f}+k_{h,r}),\\
K_\phi &=\sum_i k_{r,i}(t_i/2)^2,\\
K_\theta &= L^2/(1/k_{h,f}+1/k_{h,r}).
\end{align*}
$$

Define raw tire compression $\delta_{i,w}=\max(Z_{i,w},0)/K_t$ before load post-processing and let $\bar\delta_i$ be the axle mean. Tire compression adds roll and a static-referenced pitch correction.

$$
\begin{align*}
q_r&=M_r/K_\phi+
\tfrac12\sum_i\operatorname{atan2}
(\delta_{i,R}-\delta_{i,L},t_i),\\
\Delta\delta_0&=mg(1-2p)/(2K_t),\\
q_p&=M_p/K_\theta+\operatorname{atan2}
(\bar\delta_r-\bar\delta_f-\Delta\delta_0,L).
\end{align*}
$$

Tire camber for the force lookup is the roll angle in degrees, negative on the left and positive on the right, with zero static camber. The loaded radius in the force conversion is $\max(r_0-F_z/K_t,10^{-9})$ using the post-processed load.

#### Lateral force allocation

Yaw moment balance about the CG gives front and rear lateral-force shares $l_r/L$ and $l_f/L$. Each axle splits its assigned force in proportion to its wheel loads, or equally when the axle load is effectively zero. With fractions $s_w$ and pure lateral peaks $P_{y,w}$, the vehicle's cornering-only capacity is $\min_{s_w>0}(P_{y,w}/s_w)$. During braking or acceleration each wheel's assigned lateral force is passed to its combined-force envelope.

If yaw inertia correction is enabled, the solver estimates yaw-moment demand from the sampled curvature gradient. With $Q=\sum_w l_{\mathrm{axle}(w)}P_{y,w}$, the cornering speed is reduced when $Q$ is positive.

$$
v_{\mathrm{corr}}=v\,\operatorname{clip}
\left(1-0.25\,\frac{I_{zz}v^2|\mathrm{d}\kappa/\mathrm{d}s|}{Q},
\,0.05,\,1\right).
$$

This is a speed-limit correction based on the available tire moments. The core speed planner remains quasi-steady.
