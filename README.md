# Kinect Fusion

## Input

frames RGB + depth map

## k Frame Processing

* Depth Map preprocessing $\mathbf{D}_k$

$$
d \xrightarrow{B} D
$$

​		$B$ means BilateralFilter

* Camera Intrinsic + DepthMap $\rightarrow$ Points Cloud $\mathbf{V}_k$

$$
\begin{bmatrix} x \\y \\ z\end{bmatrix} = \mathbf{M}^{-1}\begin{bmatrix} u \\v \\ d\end{bmatrix}
$$

* Normal Map $\mathbf{N}_k$

$$
N(u,v) = [\mathbf{V}(u+1,v)-\mathbf{V}(u,v)] \times [\mathbf{V}(u,v+1)-\mathbf{V}(u,v)]
$$

## Pose Estimation

point-plane ICP
$$
\mathbf{E}\left(\mathrm{T}_{g, k}\right)=\sum_{\mathbf{u} \in \mathscr{U} \atop \Omega_{k}(\mathbf{u}) \neq \mathrm{null}}\left\|\left(\mathrm{T}_{g, k} \dot{\mathbf{V}}_{k}(\mathbf{u})-\hat{\mathbf{V}}_{k-1}^{g}(\hat{\mathbf{u}})\right)^{\top} \hat{\mathbf{N}}_{k-1}^{g}(\hat{\mathbf{u}})\right\|_{2}
$$


point-point ICP 
$$
\begin{align*}
& \forall \quad \textrm{point-pair } P_i \leftrightarrow Q_i \\
        Q_i & = R P_i + t \\
        (R,t) &= \underset{R,t}{argmin} \sum_{i=1}^{n} \omega_i\| R P_i + t -Q_i\|_2^2 \\
        & \frac{\partial F}{\partial t} = 2\sum_{i=1}^{n}\omega_i(R P_i + t -Q_i) \\
        &= 2t\sum_{i=1}^{n}\omega_i + 2R(\sum_{i=1}^{n}\omega_iP_i) - 2\sum_{i=1}^{n}\omega_iQ_i = 0 \\
        t &= \bar{Q} - R\bar{P} \\
        A &= PQ^T \\
        U\Lambda V^T &= A \quad(SVD) \\
        & \left\{ \begin{aligned}
    R &= VU^T \\
    t &= \bar{Q} - R\bar{P}
    \end{aligned} \right.
\end{align*}
$$

## Point_to_Plane ICP

$$
Trans_{opt} = \underset{Trans}{argmin} || \sum_{i}(Trans \cdot \mathbf{s}_i-\mathbf{d}_i)\cdot \mathbf{n}_i ||_2^2 \\
$$

$\mathbf{s}$ is source points cloud, $\mathbf{d}$ is destination points cloud, $\mathbf{n}$ are normal vectors of destination



## Find the Surface

Find the surface via **Ray Casting** if we choose point-plane ICP

* step = $\mu$ instead of 1 voxel

## TSDF

Truncated signed distance $F_k(p)$ , $tsdf(p)$
$$
sdf(p) = D(x) - dist(p)
$$


```
dist(p) is the distance from voxel p to camera O

x is the pixel which voxel p reprojects to image plane 

D(x) is the depth value of x
```

$$
tsdf(p) = \left\{ 
\begin{aligned}
& sdf(p)/\mu \quad ,|\frac{sdf(p)}{\mu}|<1\\
& 1 \quad ,\frac{sdf(p)}{\mu} > 1\\
& -1\quad ,\frac{sdf(p)}{\mu} <-1
\end{aligned}
\right.
$$

Weight $W_k(p)$
$$
W(p) = \frac{cos(\theta)}{dist(p)} \\
$$
$\theta$ is angle between normal vector and pixel ray

## Volume

New frame arives

For each voxel in volume, compute $ W_k(p)$ and $ F_k(p)$ in this frame, called $\mathrm{W}_{\mathrm{R}_{k}}(\mathbf{p}) \mathrm{F}_{\mathrm{R}_{k}}(\mathbf{p})$
$$
\begin{aligned}
\mathrm{F}_{k}(\mathbf{p}) &=\frac{\mathrm{W}_{k-1}(\mathbf{p}) \mathrm{F}_{k-1}(\mathbf{p})+\mathrm{W}_{\mathrm{R}_{k}}(\mathbf{p}) \mathrm{F}_{\mathrm{R}_{k}}(\mathbf{p})}{\mathrm{W}_{k-1}(\mathbf{p})+\mathrm{W}_{\mathrm{R}_{k}}(\mathbf{p})} \\
\mathrm{W}_{k}(\mathbf{p}) &=\mathrm{W}_{k-1}(\mathbf{p})+\mathrm{W}_{\mathrm{R}_{k}}(\mathbf{p})
\end{aligned}
$$
What's more, restrict the $\mathrm{W}_{k}(\mathbf{p})$ by $\mathrm{W}_{k}(\mathbf{p}) \leftarrow \min \left(\mathrm{W}_{k-1}(\mathbf{p})+\mathrm{W}_{\mathrm{R}_{k}}(\mathbf{p}), \mathrm{W}_{\eta}\right)$



## Results

Marching Cubes to mesh
https://scikit-image.org/docs/dev/auto_examples/edges/plot_marching_cubes.html