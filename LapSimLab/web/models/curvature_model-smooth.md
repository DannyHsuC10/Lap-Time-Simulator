### Gaussian Distance-Based Smoothing

This option smooths the signed curvature from the LINE/ARC CSV in metres of traveled distance. Centerline coordinates, track edges, sample distances and timing geometry come from the original sampled path.

#### Resampling and convolution

For $N$ samples over total distance $s_{\mathrm{end}}$, the solver creates a uniform grid with spacing $h=s_{\mathrm{end}}/(N-1)$ and linearly interpolates the raw curvature onto it. The input Curvature Smoothing Scale is the Gaussian standard deviation $\sigma_m$ in metres. If $\sigma_m/h<0.125$, the solver uses the original curvature.

Otherwise the discrete radius is $r=\min(\lceil4\sigma_m/h\rceil,N-1)$. On the uniform grid, the normalized weights and filtered curvature are

$$
\begin{align*}
w_j&=\exp\!\left[-\tfrac12(jh/\sigma_m)^2\right],\\
\widetilde\kappa_i &=
\frac{\sum_{j=-r}^{r}w_j\,\kappa_{i+j}}
{\sum_{j=-r}^{r}w_j}.
\end{align*}
$$

Indices outside the grid use the closest endpoint's curvature. The filtered grid is then linearly interpolated back to the original sample distances.

#### Effect on the solver

The filtered curvature drives $a_y=v^2\widetilde\kappa$ and the optional yaw inertia correction. Smoothing spreads sharp changes across neighboring samples. A curved section may therefore have lower peak demand, and a nearby straight can acquire nonzero demand. The filter keeps the spatial route and its sampled length fixed.
