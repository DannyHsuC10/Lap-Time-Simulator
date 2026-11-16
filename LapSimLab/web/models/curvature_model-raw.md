### Raw Segment Curvature

The solver follows the centerline defined by the track CSV. Each row contributes a straight `LINE` or circular `ARC` with constant curvature.

#### Geometry and turn direction

For a line, sample interpolation parameter $u\in[0,1]$ uniformly between endpoints $\mathbf p_0$ and $\mathbf p_1$.

$$
\mathbf p(u)=(1-u)\mathbf p_0+u\mathbf p_1,
\qquad \kappa=0.
$$

For an arc with center $(x_c,y_c)$ and radius $R$, the CSV gives start and end angles in degrees. The solver converts them to radians and samples the angular sweep in the requested direction.

$$
\begin{align*}
x&=x_c+R\cos\theta,\qquad y=y_c+R\sin\theta,\\
\kappa&=\sigma/R,
\end{align*}
$$

where $\sigma=1$ for counterclockwise and $\sigma=-1$ for clockwise. The angular sweep is

$$
\Delta\theta=\sigma\big((\sigma(\theta_1-\theta_0))\bmod 2\pi\big).
$$

A sweep of zero is invalid, so a full circle can be split into multiple arcs. Positive curvature requires positive lateral acceleration, which is a left turn.

#### Sampling and distance

For CSV length $L_{\mathrm{row}}$ and configured sample spacing $\Delta s_{\mathrm{set}}$, the number of points in a row is

$$
n=1+\max(1,\lceil L_{\mathrm{row}}/\Delta s_{\mathrm{set}}\rceil).
$$

The first point of every row after the first is dropped when concatenating segments. The solver accumulates chord distances from the sampled points.

$$
s_i=s_{i-1}+
\sqrt{(x_i-x_{i-1})^2+(y_i-y_{i-1})^2}.
$$

The reported path length uses $s_{\mathrm{last}}$. For circular arcs, chord sampling can make it slightly shorter than the corresponding arc length. Track width is used to offset the centerline into track edges.

The cornering demand at a sample is $a_y=v^2\kappa$. Raw curvature can jump at segment boundaries. If yaw inertia correction is enabled, it uses the gradient of this sampled signal.
