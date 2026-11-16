### Pacejka Table Interpolation

The NPZ contains precomputed force limits from a Pacejka tire map at zero camber. Its load axis indexes signed lateral bounds, a pure lateral peak, signed brake and drive limits, and rolling-resistance torque.

#### Combined-force envelope

Let $\lambda$ be Tire Utilization divided by 100. At the requested positive normal load $F_z$, the solver interpolates the lower and upper combined-force lateral bounds $Y_-(F_z)$ and $Y_+(F_z)$ and multiplies both by $\lambda$. Loads outside the table use the closest endpoint. If the requested $F_y$ is inside the scaled interval, its normalized position is

$$
u=\frac{F_y-\lambda Y_-}{\lambda(Y_+-Y_-)}.
$$

An out-of-range $F_y$ makes the operating point infeasible. A collapsed interval produces zero longitudinal force at its sole feasible lateral force.

Let $F_{z,i}$ and $F_{z,i+1}$ bracket the clamped load, and let $w$ be the load fraction. For each signed bound, interpolate its row along `fy_fraction`, then blend the two rows.

$$
\begin{align*}
w&=\frac{F_z-F_{z,i}}{F_{z,i+1}-F_{z,i}},\\
B^\pm(F_z,F_y)&=\lambda
\big[(1-w)B_i^\pm(u)+wB_{i+1}^\pm(u)\big].
\end{align*}
$$

The cornering-only speed search uses $\lambda$ times the separately interpolated `fy_pure`, while the combined-force search uses the full signed interval. Different drive and brake bounds can therefore apply at the same $F_y$.

#### Rolling resistance and slip estimates

For a loaded wheel the solver interpolates the zero-slip rolling torque $M_{rr}$ from the table and uses $F_{rr}=M_{rr}/r_{\mathrm{loaded}}$. Point-mass mode queries the envelope and rolling torque at one-quarter of total load, scales force back up by four, and uses the free tire radius for rolling resistance.

If the NPZ includes slip grids, telemetry reconstructs the slip angle and ratio after the speed plan. It interpolates the grids in load, mirrors the lateral sign for the right-side wheels, and picks the sampled pair with the smallest normalized squared force residual.

$$
E=\left(\frac{\widehat F_x-F_x}{\max(R_x,1)}\right)^2
+\left(\frac{\widehat F_y-F_y}{\max(R_y,1)}\right)^2.
$$

Here $R_x,R_y$ are the ranges of the scaled forces in the interpolated grid, in newtons. The grid point also gives the moment estimates. These diagnostics are separate from the force-envelope solve.
