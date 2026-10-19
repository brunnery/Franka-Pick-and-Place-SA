# Theory and Methods – Collection for the Report

> Living document. Every formula, model or principle that the code relies on and that belongs in the report is written down here, in report-ready English, with symbols defined once.
> Status per section: **used** (in the code), **planned** (decided, not implemented yet), **dropped** (kept for the discussion chapter).
> Last update: 18.10.2026 (gripper clocking and fixed orientation corrected, Section 0; base-frame target and motion, Sections 3 and 5; stud detection, 2.6c)

---

## 0. Notation and frames

| Symbol | Meaning |
|---|---|
| $\{B\}$ | robot base frame (FR3 `fr3_link0`) |
| $\{F\}$ | flange frame: origin at the centre of the flange contact face, **axes of the Franka hand frame**, i.e. `fr3_link8` turned by −45° about $z$ (the robot's end-effector frame with the Franka Hand set in Desk). **Not** the axes of `fr3_link8`. |
| $\{C\}$ | camera frame, OpenCV convention: $x_C$ right in the image, $y_C$ down in the image, $z_C$ along the optical axis into the scene |
| $\{T\}$ | tool frame (TCP): centre of the gripper bottom plate |
| ${}^{A}\mathbf{T}_{B} \in SE(3)$ | homogeneous transform that maps coordinates in $\{B\}$ to $\{A\}$ |
| $(u, v)$ | pixel coordinates (column, row), origin top left |
| $\mathbf{K}$ | camera intrinsic matrix |
| $d$ | distance from the camera centre to the object plane along $z_C$ |

Fixed transforms (CAD of the new mount, 16.10.2026, mm, in the flange frame $\{F\}$ with $z_F$ pointing along the tool):

$$
{}^{F}\mathbf{t}_{C} = (44.4,\ 0,\ 57.223)^\top, \qquad {}^{F}\mathbf{t}_{T} = (0,\ 0,\ 118.823)^\top
$$

The flange origin is the centre of the contact face on the FR3 flange (a centring cylinder of the adapter reaches into the arm). ${}^{F}\mathbf{t}_{C}$ points to the centre of the lowest face of the camera housing; the lens lies 5 mm higher, and the projection centre higher still (Section 2.8). ${}^{F}\mathbf{t}_{T}$ points to the centre of the gripper bottom plate, which rests on the table during the grasp. CAD components below 0.2 mm are set to zero. With the gripper resting on the table, the housing is 61.6 mm above the table.

A second, independent CAD measurement (19.10.2026, `cad/export/Configuration_1.*`) gives the camera housing → TCP vector ${}^{F}\mathbf{t}_{CT} = (-44.505,\ 0.105,\ 61.6)^\top$ (Fusion, base axes: $(-44.505,\ -0.105,\ -61.60)$). The chain must close,

$$
{}^{F}\mathbf{t}_{C} + {}^{F}\mathbf{t}_{CT} = {}^{F}\mathbf{t}_{T},
$$

and it does to $(-0.105,\ 0.105,\ 0)$ mm, below the 0.2 mm CAD rounding (the two Fusion measurements picked the camera centre at slightly different points). The motion uses only ${}^{F}\mathbf{t}_{C}$ and ${}^{F}\mathbf{t}_{T}$; the closure is checked every time `tasks.yaml` is loaded, so a CAD update of one offset without the other fails loudly. Previous mount (08.10., written in other axes): $\Delta z = 60.223$ mm and $(0,\ 0,\ 113.823)^\top$.

The camera lies on $+x_F$. Expressed in the axes of `fr3_link8` the same camera vector is $(31.4,\ -31.4,\ 57.223)^\top$; reading ${}^{F}\mathbf{t}_{C}$ in `fr3_link8` axes puts the camera 45° off. The camera axes in $\{F\}$ are $x_C = +y_F$, $y_C = -x_F$, $z_C = +z_F$, i.e. ${}^{F}\mathbf{R}_{C} = \mathbf{R}_z(+90°)$.

**Commanded orientation, the only one used:** the end-effector quaternion $(x, y, z, w) = (1,\ 0,\ 0,\ 0)$, which is also the controller start pose ($q_7 = 0.80$ rad). The tool points down and $x_F = +x_B$, $y_F = -y_B$, $z_F = -z_B$; `fr3_link8` has a yaw of −45°. In this pose the camera lies in front of the flange, ${}^{B}\mathbf{R}_{F}\,{}^{F}\mathbf{t}_{C} = (44.4,\ 0,\ -57.223)^\top$. No pose of the task turns the tool about its axis, so the pixel-to-base mapping below is constant and no adjustment is needed at the start of a run.

With the gripper pointing down and the camera in front of the flange, the camera axes map to the base axes as $x_B = -y_C$, $y_B = -x_C$, $z_B = -z_C$.

---

## 1. Camera model – **used**

### 1.1 Pinhole projection
A point $\mathbf{P}_C = (X_C, Y_C, Z_C)^\top$ in the camera frame projects to the normalised image plane as

$$
x = \frac{X_C}{Z_C}, \qquad y = \frac{Y_C}{Z_C}
$$

and to pixels through the intrinsic matrix

$$
\begin{pmatrix} u \\ v \\ 1 \end{pmatrix} = \mathbf{K} \begin{pmatrix} x_d \\ y_d \\ 1 \end{pmatrix},
\qquad
\mathbf{K} = \begin{pmatrix} f_x & 0 & c_x \\ 0 & f_y & c_y \\ 0 & 0 & 1 \end{pmatrix}
$$

$f_x, f_y$: focal lengths in pixels; $(c_x, c_y)$: principal point. Skew is assumed zero.

### 1.2 Lens distortion (Brown–Conrady)
With $r^2 = x^2 + y^2$, the distorted normalised coordinates are

$$
\begin{aligned}
x_d &= x\,(1 + k_1 r^2 + k_2 r^4 + k_3 r^6) + 2 p_1 x y + p_2 (r^2 + 2x^2) \\
y_d &= y\,(1 + k_1 r^2 + k_2 r^4 + k_3 r^6) + p_1 (r^2 + 2y^2) + 2 p_2 x y
\end{aligned}
$$

$k_i$: radial, $p_i$: tangential coefficients. Only the detected centre point is undistorted (iteratively, `cv2.undistortPoints`), not the whole image. This keeps the pipeline fast and avoids interpolation artefacts in the segmentation.

### 1.3 Intrinsic calibration (Zhang's method)
A planar checkerboard with known corner positions $\mathbf{M}_j$ is imaged in $n$ poses. $\mathbf{K}$, the distortion vector and the board poses $(\mathbf{R}_i, \mathbf{t}_i)$ are estimated by minimising the reprojection error

$$
\min_{\mathbf{K},\,\mathbf{k},\,\mathbf{R}_i,\,\mathbf{t}_i}\ \sum_{i=1}^{n} \sum_{j} \left\| \mathbf{m}_{ij} - \pi(\mathbf{K}, \mathbf{k}, \mathbf{R}_i, \mathbf{t}_i, \mathbf{M}_j) \right\|^2
$$

with Levenberg–Marquardt, initialised from the closed-form homography solution [Zhang 2000]. The RMS reprojection error is reported as the calibration quality measure. Tool: ROS 2 `camera_calibration`.

### 1.4 Calibration result – **used**
14.10.2026, 1280 × 720 px, board with 8 × 11 inner corners and 30 mm squares, 68 views. File: `calibration/hd_camera.yaml`.

| Parameter | Value |
|---|---|
| $f_x,\ f_y$ | 2077.2 px, 2083.7 px |
| $c_x,\ c_y$ | 695.4 px, 405.6 px |
| $k_1, k_2, p_1, p_2, k_3$ | 0.137, −0.749, −0.0022, 0.0012, 0 |
| Field of view (h × v) | $2\arctan\frac{W}{2f_x} = 34.2°$ × $2\arctan\frac{H}{2f_y} = 19.6°$ |

Two calibrations from overlapping sample sets differed by less than 0.4 % in $f$ and less than 1 px in $(c_x, c_y)$, which indicates a stable estimate.

Consequences for the method:
- **Ground resolution** $d/f$: 0.096 mm/px at $d = 200$ mm, 0.144 mm/px at 300 mm, 0.193 mm/px at 400 mm. The M8 test nut (13 mm across flats) spans about 90 px at 300 mm, an M4 nut (7 mm) still about 48 px.
- **Narrow field of view:** at $d = 300$ mm the image covers only about 185 × 104 mm on the table. Each work area must fit into this footprint from its observation pose.
- **Principal point offset:** the optical axis meets the image at $(695, 406)$, i.e. 55 px right of and 46 px below the image centre $(640, 360)$. At $d = 300$ mm this is about 8 mm and 7 mm on the table. Centring an object on the geometric image centre, as in the BA, therefore does not place it on the optical axis; the back-projection in Section 3 uses $(c_x, c_y)$ and avoids this bias.

---

## 2. Object detection – **used**: simple finder (2.0, `simple_vision`); full pipeline 2.1–2.8 (`sa_vision`) kept for later

Goal: the centre $(u, v)$ of the nut (pick) and of the stud base (place), from a single observation pose, on a white background, without painting the parts (in the BA they were painted matt black). The nut detection does not assume a nut size; it measures it.

**Appearance of the bare nut (test images of 15.10.2026, hand-held, 27 images).** The zinc-plated nut reflects specularly. Most of its body is *darker* than the white table (ratio to the table about 0.5–0.8), a few highlights are *brighter* (up to 1.2), and some facets are as bright as the table. The thread hole is dark, not bright: through the bore the camera sees the table in the nut's own shadow (ratio about 0.45–0.7). On a locknut the blue nylon ring merges with the hole into one dark disc. Each nut casts a soft shadow that is 20–40 % darker than the table. The previous segmentation (inverted Otsu threshold, hole as a bright region, Sections 2.2–2.4 of the earlier version) found 0 of 27 nuts on these images; it is **dropped** (see 2.2).

### 2.0 Simple object finder – **used** (default since 18.10.2026, `simple_vision`)
For the first runs at the robot, the detection assumes exactly one object on a blank background and checks neither size nor shape. The background is estimated by heavy Gaussian smoothing, $B = G_{\sigma} * I$ with $\sigma = 60$ px (much larger than the object, about 60 px across at 400 mm), computed on an image downscaled by 8. Foreground is where the relative deviation $|I - B| / B$ exceeds $0.15$, in either direction, so both dark and specular parts count. After closing and opening, the largest blob that does not touch the image border is taken (parts of the robot reaching into the view and objects cut by the border are excluded).

**Hexagon fit.** For an orientation $\theta$, the convex hull $\{\mathbf p_i\}$ is enclosed by three pairs of parallel lines with normals $\mathbf n_k = (\cos(\theta + k\,60°), \sin(\theta + k\,60°))$, $k = 0, 1, 2$: the widths are $w_k = \max_i \mathbf n_k^\top \mathbf p_i - \min_i \mathbf n_k^\top \mathbf p_i$ and the midlines $\mathbf n_k^\top \mathbf c = m_k$ with $m_k$ the mean of the two extremes. For a regular hexagon the three widths are equal at every $\theta$ by symmetry, so the criterion is the tightest enclosing hexagon, $\min_\theta \sum_k w_k$ (1° steps over 0–60°), which touches the flats. Its centre is the least-squares solution of the three midline equations. For any centrally symmetric outline (hexagon, square base of the stud) the midlines meet in the centre of symmetry.

**Result on the bench set** (16.10., no reference image): 26 of 26 grid nuts, position error RMS 1.36 mm, max 2.25 mm (Section 2.8 method; the full pipeline below: 1.40 mm). Empty sheet and nuts cut by the border: nothing found. The width across flats is too large (17 mm instead of 13 mm), because the soft shadow is part of the blob; the centre is not affected. With several objects in view the largest wins; the pipeline of Sections 2.1–2.7 (edges, reference image, size, hole and shape filters, temporal median) remains available in `sa_vision` and its parts can be brought back one at a time.

### 2.1 Image acquisition and pre-processing – **used**
1. **Fixed exposure and white balance** (`software/config/camera.yaml`, set with `v4l2-ctl` before the camera starts). With automatic exposure the brightness depends on the scene content (a bright table drives the gain down), so a reference image of the empty work area and a live image with a nut would not share the same grey levels. Trade-off measured in office light: the automatic mode reaches a table grey value of about 190 at 24 fps; manual exposure needs about 120 ms (4 fps) for the same brightness, 50 ms gives about 110 at 20 fps. The scene is static during the observation, so a low frame rate is acceptable. The exposure must not clip the table: a clipped table (grey value 255) hides the highlights of the nut and makes the division by the reference image meaningless. Bench setup of 16.10.2026 (diffuse lamp, $d \approx 420$ mm): 10 ms gives a table median of 180 without clipping, 15 ms clips 6 % and 25 ms clips 60–94 % of the pixels. Target: table grey value 170–200, no clipping.
2. Greyscale conversion.
3. Gaussian smoothing, $G_\sigma(x,y) = \frac{1}{2\pi\sigma^2} e^{-(x^2+y^2)/2\sigma^2}$, to suppress sensor noise before thresholding.
4. Region of interest = known work area projected into the image, so the gripper and the table edge cannot produce false detections.

### 2.2 Background normalisation and two-sided threshold – **used** (replaces Otsu, **dropped**)
A global threshold cannot separate an object that is partly brighter and partly darker than the background. In addition, the nut covers only about 1 % of the image, so Otsu's criterion [Otsu 1979] splits the illumination gradient of the table instead of separating the nut (observed on the test images).

The image is therefore divided by the local table brightness $B(u,v)$:

$$
r(u,v) = \frac{I(u,v)}{B(u,v)}
$$

With the multiplicative image model $I = \rho\,L$ (reflectance times illumination), $r$ is the reflectance relative to the table and does not depend on the illumination level or on vignetting. The bare table gives $r \approx 1$. A pixel is foreground if

$$
r > 1 + \tau_b \quad\text{or}\quad r < 1 - \tau_d \qquad (\tau_b = 0.06,\ \tau_d = 0.25)
$$

$B$ is computed in two steps:
1. **Reference image** (method A'): if an image $R$ of the empty work area exists, taken from the same observation pose with the same fixed exposure, the image is first divided by it, $I \leftarrow I / R$. This removes everything static: the gradient of the illumination, the table edge, marks on the table, parts of the robot in view. Recorded with `ros2 run sa_vision capture_reference` (mean of 30 frames).
   The reference is only valid for the lighting it was recorded with. If the median of $I/R$ over the ROI leaves $[0.8,\ 1.25]$ (`reference_match_range`), the lighting has changed; the reference is then ignored (method A, warning in `vision_node`), because dividing by it would add structure instead of removing it.
2. **Local median** (method A, and applied after step 1 in A'): $B$ is the median of the (divided) image over a square window of side $w = k\, s_\text{max}\, f / d$, with $s_\text{max}$ the largest accepted width across flats and $k = 1.25$. The median ignores objects that cover less than half of the window, so the nut does not enter $B$; the window still follows slow variations of the table brightness. Near the nut, the window also contains the nut's soft shadow, which lowers $B$ there and removes most of the shadow from $r$. A larger window ($k = 2$) lets the shadow through: it then becomes part of the silhouette and the nut fails the shape filter (18 instead of 20 of 23 M8 nuts found). The value $k = 1.25$ was chosen on the same 27 images. For speed the median runs on an image downscaled by 8 and is interpolated back.

The step 2 suppression is partial. On synthetic images a shadow of the measured shape is removed up to a depth of about 0.25; the real shadows of up to 0.4 on the test images were removed as well, because they are softer.

### 2.2a Gradient-based foreground – **used** (default since 16.10.2026, `segmentation: edges`)
The level threshold of 2.2 assumes that the nut differs from the table in brightness. Under the diffuse lamp of the bench setup this is wrong: the flats of the zinc-plated nut reflect the lamp and reach 0.85–1.0 of the table brightness, while the soft shadow beside the nut is the darkest area after the thread hole. The level threshold then segments hole plus shadow and misses the flats, and the silhouette fails the shape filter (4 of 26 nuts found on the bench set, Section 2.8).

The foreground is therefore taken from the **gradient magnitude** of the ratio image,

$$
g(u,v) = \sqrt{(S_x * r)^2 + (S_y * r)^2} > \tau_g \qquad (\tau_g = 0.35)
$$

with the $3 \times 3$ Sobel kernels $S_x, S_y$ (they include a smoothing factor, so a step of height $\Delta r$ gives a peak of about $4\,\Delta r$ before the blur). The physical argument: the outline of the nut, its facet edges, the chamfer and the hole are sharp edges, whereas a shadow cast by an extended (diffuse) light source has a penumbra that is wide compared with the pixel size, so its gradient stays small. Because $g$ is computed on $r = I/B$, the threshold is relative and independent of the illumination level. The edge pixels are then **grouped by proximity**: pixels closer than $k_g\, s_\text{px,max}$ ($k_g = 0.1$, dilation with a disc and connected components) form one candidate, whose silhouette is the convex hull of its edge pixels only (the dilation does not widen it). An outline with a gap, e.g. a flank without contrast against its own shadow, therefore still yields the whole nut; a closing followed by filling the outline (first version) lost such nuts. Bench set: $k_g = 0.05$ → 25/26, 0.1 and 0.2 → 26/26; hand-held images 18/27 instead of 12/27 with closing + filling.

Known bias: the gradient exceeds $\tau_g$ about 1.5 px beyond the true outline on each side (blur and Sobel support), so $s_\text{px}$ is about 3 px too large on synthetic images (0.5 mm at $d = 340$ mm). On the bench set the measured width across flats was $13.30 \pm 0.34$ mm for the nominal 13 mm. The centre is not affected, because the widening is symmetric.

Trade-off: on the 27 hand-held images of 15.10. (no lamp, no reference image, varying height) the edge method finds 18 of 27 nuts against 20 of 27 with the level threshold; there the nut is dark on a bright table and the level threshold works well. The level threshold remains available (`segmentation: threshold`). The edge method is the default because the bench conditions (fixed camera, fixed exposure, reference image, lamp) are those of the robot.

### 2.3 Morphology – **used**
Closing ($A \bullet B = (A \oplus B) \ominus B$, two iterations, elliptic $7 \times 7$ element) first joins highlights, dark facets and the hole into one region; opening ($A \circ B = (A \ominus B) \oplus B$) then removes small specks. The edge foreground (2.2a) uses grouping instead of morphology.

### 2.4 Silhouette and dark core – **used**
Outer contours are extracted by border following [Suzuki & Abe 1985]. The **silhouette** of a candidate is the convex hull of its contour; the hull fills concavities where a facet happens to have the brightness of the table. The **dark core** is the largest connected region inside the silhouette with $r < 1 - \tau_c$ ($\tau_c = 0.28$). It contains the thread hole, the nylon ring of a locknut and the shaded inner chamfer.

### 2.5 Centre from image moments – **used**
For a region $R$, the raw moments are $m_{pq} = \sum_{(x,y)\in R} x^p y^q$, and the centroid is

$$
\bar u = \frac{m_{10}}{m_{00}}, \qquad \bar v = \frac{m_{01}}{m_{00}}
$$

with $m_{00}$ the area in pixels. The reported nut centre is the centroid of the **silhouette**. The centroid of the dark core is not used: its shape depends on how the light falls into the bore, and it lay up to $0.18\,s$ away from the silhouette centroid on the test images (about 2 mm for the large nut).

Known biases, to be measured at the robot (Section 4):
- A facet with the brightness of the table is missing from the silhouette; edges with little contrast are cut more than edges with high contrast. On the test images the lowest facet was often cut, which moves the centre by a few pixels (estimate 0.5–1 mm) away from that side. Synthetic test: a nut with one half just beyond the dark threshold gives about 1 px offset.
- With perspective the silhouette is the union of the top face, the visible side faces and the bottom face. For a nut at lateral distance $X$ from the optical axis the top and bottom face differ by $X h / d$ on the object plane (≈ 1.2 mm at $X = 50$, $h = 8$, $d = 340$ mm), so the silhouette centroid lies roughly half of this outside the top-face centre.

### 2.6 Shape filters (scale-invariant) – **used**
- **Width across flats and across corners** by rotating calipers [Toussaint 1983] on the silhouette: the minimum width of a convex polygon is attained with one caliper flush with an edge; for a hexagon it is the width across flats $s_\text{px}$. The maximum width (the diameter) is the width across corners $e_\text{px}$.
- **Width ratio** $s/e$: regular hexagon $\sqrt3/2 \approx 0.866$, square $1/\sqrt2 \approx 0.707$, circle 1. Accepted window 0.78–0.95. On the test images, accepted nuts had 0.82–0.91, the parts of the stud (square base, thread) 0.39–0.67. This replaces the compactness filter: compactness $C = 4\pi A / P^2$ (hexagon $\pi/(2\sqrt3) \approx 0.907$, square $\pi/4 \approx 0.785$) did not separate the blurred square stud base ($C = 0.87$) from the nuts ($C = 0.92$–$0.96$); it is still reported for debugging. A round washer is rejected by $s/e \approx 1$.
- **Dark core present and concentric:** the core area divided by the silhouette area must lie in 0.15–0.85, and the core centroid must lie within $0.25\,s_\text{px}$ of the silhouette centroid. The window is deliberately wide, see 2.6a.
- **Uniqueness:** a detection is valid only if exactly one silhouette passes all filters (decision 14.10.). Two candidates in the work area are an error, not a guess.
- **Polygon approximation** for the stud base (Douglas–Peucker, 4 corners, side ratio ≈ 1) – **planned**.

### 2.6a Size-independent nut detection – **used**
1. **Metric size from the image.** With the distance $d$ known from the robot pose, the measured width across flats is
$$
\hat s = \frac{s_\text{px}\, d}{f}
$$
The size is an output of the detection, not an assumption, and is always reported in mm. Relative errors in $d$ map one to one: $\delta \hat s / \hat s = \delta d / d$. On the synthetic test images $\hat s$ is about 2.5 px too large (anti-aliased drawing and blur widen high-contrast edges); the bias on real nuts is to be calibrated at the robot.

2. **Size window and target size.** Without a target, a candidate is accepted if $\hat s$ lies in $[s_\text{min}(1 - t),\ s_\text{max}(1 + t)]$ with $s_\text{min} = 7$ mm (M4), $s_\text{max} = 19$ mm (M12, DIN 934) and $t = 0.15$; then exactly one nut of any size may be in view. With a target $s^\ast$ (parameter `target_across_flats_mm`, 13 mm for the M8 nut and stud of the task), only $\hat s \in [s^\ast(1 - t),\ s^\ast(1 + t)]$ is accepted, so nuts of other sizes are ignored. The nearest other sizes (10 and 16 mm) differ by 23 %, so $t = 0.15$ leaves margin for the bias of $\hat s$ and for errors in $d$.

3. **Thread size only if unambiguous.** The standard widths across flats are

| Size | ISO 4032 $s$ [mm] | DIN 934 $s$ [mm] |
|---|---|---|
| M4 | 7 | 7 |
| M5 | 8 | 8 |
| M6 | 10 | 10 |
| M8 | 13 | 13 |
| M10 | 16 | 17 |
| M12 | 18 | 19 |

A thread size is assigned only if all standard values within $\pm 5\,\%$ of $\hat s$ belong to the same thread size, otherwise the size is left empty. M10 DIN (17 mm) and M12 ISO (18 mm) differ by only 5.6 %, so for example $\hat s = 17.5$ mm stays unassigned. The large plain nut in the test images measured about 1.46 times the M8 nut, i.e. about 19 mm (M12, DIN 934).

4. **Hole ratio: theory and measurement.** For the thread minor diameter $D_1 = d_\text{nom} - 1.0825\,P$ the area ratio $\frac{\pi D_1^2 / 4}{A_\text{hex}}$, with $A_\text{hex} = \frac{\sqrt3}{2} s^2$, stays within a narrow band over the common sizes:

| Size | $s$ [mm] | $D_1$ [mm] | hole / hexagon area (theory) |
|---|---|---|---|
| M4 | 7 | 3.24 | 0.20 |
| M5 | 8 | 4.13 | 0.24 |
| M6 | 10 | 4.92 | 0.22 |
| M8 | 13 | 6.65 | 0.24 |
| M10 | 16 | 8.38 | 0.25 |
| M12 | 18 | 10.11 | 0.29 |

The plan was one ratio window of about 0.15–0.35 for all sizes. The **measured** ratio of dark core to silhouette is different, because the dark core is not just the bore: it also contains the nylon ring, the shaded chamfer and dark facets, and the silhouette contains side faces:

| Nut (test images 15.10., runs with and without target) | dark core / silhouette (measured) |
|---|---|
| M8 locknut with nylon insert | 0.58–0.70 |
| M8 plain hex nut | 0.34–0.65 |
| large plain nut (≈ M12) | 0.21–0.30 |

The ratio is therefore not a usable size-independent constant. It is only checked loosely (0.15–0.85, "core present"), and the size is taken from $\hat s$ alone.

**Result on the 27 test images** (hand labels: 23 images with an M8 nut, 5 of them with an additional large nut, 4 empty images; $d = 340$ mm assumed for all, background estimated, no reference image): with target M8, 20 of 23 M8 nuts found and no false detection; without a target, 25 of 28 nuts accepted as candidates and no false candidate (the 5 images with two nuts are then correctly invalid). Missed: a nut in an overexposed corner, a nut at the image border whose true distance (≈ 270 mm) lies outside the size window at the assumed 340 mm, and a nut seen obliquely with its side faces ($s/e$ = 0.77). **These thresholds were chosen on the same images**, so the rates are optimistic; a fair rate needs a second, independent image set from the observation pose at the robot.

### 2.6c Stud detection – **used, not yet validated on images** (`detect_stud`)
The stud is fixed in its area and seen from directly above (observation pose over it). The same pre-processing (2.1–2.2a) yields silhouettes; the stud is the one whose width $s$ matches the 10 mm square base ($\pm 20\,\%$, $d$ to the top face of the base) and whose width ratio is that of a square, $s/e \in [0.62, 0.80]$ around $1/\sqrt2 = 0.707$; a nut ($s/e \ge 0.82$) never passes. The threaded rod (8 mm) lies inside the square and does not change the hull when the camera is above the stud. Seen off-axis, the tip of the rod (19 mm higher) is displaced outwards by $X h / d$, which is why the stud observation pose is taught directly above it. Exactly one candidate must pass. Each observation pose has its own reference image (`reference_image`, `reference_image_stud`), because the empty nut area and the stud area look different.

### 2.6b Source of the distance $d$ – **used** (`d_source` in `vision.yaml`)
The metric quantities need the distance $d$ from the camera to the nut top face.
- **`yaml`**: $d$ is a known value, `object_plane_distance_mm`; at the robot it will come from forward kinematics (Section 3). The size $\hat s = s_\text{px} d / f$ is then *measured*.
- **`from_size`** (hand-held tests without the robot, at any height): if the nut size $s^\ast$ is known (`target_across_flats_mm`), the pinhole relation is inverted,
$$
\hat d = \frac{f\, s^\ast}{s_\text{px}}
$$
The size is then *assumed*, not measured: a size filter on $\hat s$ would be circular ($\hat s = s^\ast$ for every candidate). The size filter is therefore relaxed to a plausibility check of the implied distance, $\hat d \in [d_\text{min}, d_\text{max}]$ (`d_from_size_range_mm`, 200–500 mm), i.e. $s_\text{px} \in [f s^\ast (1-t)/d_\text{max},\ f s^\ast (1+t)/d_\text{min}]$. Consequences: a nut of another size is taken for an M8 at another distance (an M12 next to an M8 gives "2 candidates" instead of being rejected), and every error in $s_\text{px}$ goes directly into $\hat d$: $\delta \hat d / \hat d = -\delta s_\text{px} / s_\text{px}$. The bias of about +2.5 px found for $s_\text{px}$ (Section 2.6a) gives about 3 % too small $\hat d$ at 340 mm, and through Section 4 a lateral error of $X \cdot 0.03$ (0.9 mm at $X = 30$ mm). The debug image states "d estimated from size". On the 27 test images the implied distances were 273–390 mm, consistent with the estimate of 15.10.; 19 of 23 M8 nuts passed all filters, 14 images gave a valid single detection (the 5 images with the large nut give "2 candidates").

**Debug image** (`/vision/debug`): the camera frame ($x_\text{cam}$ right, $y_\text{cam}$ down) is drawn as a legend at the top left, the robot frame ($x_\text{robo} = -y_\text{cam}$, $y_\text{robo} = -x_\text{cam}$, gripper pointing down) at the bottom right; the principal point $(c_x, c_y)$ is marked with a cross and joined to the nut centre. At the top right, for a detected nut, it shows $\hat s$, $d$ and the offset of the nut from the optical axis in the robot frame, $(\Delta x_\text{robo}, \Delta y_\text{robo}) = (-Y_C, -X_C)$ with $(X_C, Y_C)$ from Section 3.2, i.e. the distance the robot would have to move to bring the optical axis over the nut.

### 2.7 Temporal robustness – **used** (`sa_vision/temporal.py`)
Pixels at the rim of the nut lie close to the threshold of Section 2.2, so they flip between foreground and background from frame to frame, even with a still camera (sensor noise, JPEG compression). Each flip at the outer boundary moves the convex hull and with it the centre. Measured on 60 frames of a still camera with an M8 locknut ($s_\text{px} \approx 145$, $d \approx 180$ mm): the single-frame centre jitters with a standard deviation of 0.9 px (range 3.4 px, about 0.08 mm and 0.3 mm on the table).

Spatial remedies were tried and **dropped**: smoothing the binary mask (Gaussian blur and threshold at one half, $\sigma$ up to $0.08\,s_\text{px}$) left the jitter at 0.8–0.9 px and rounds the hexagon corners, which raises $s/e$ towards the circle; a least-squares fit of a regular hexagon to the outline reached 0.6 px. The flipping pixels change the outline itself, which no spatial model can restore.

The published centre is therefore the **median over the last $n$ valid frames** ($n = 5$, `centre_median_frames`), separately for $u$, $v$ and $d$:

$$
\hat u_t = \operatorname{median}(u_{t-n+1}, \dots, u_t)
$$

On the same 60 frames this gives a standard deviation of 0.08 px (range 0.3 px). Unlike the exponential filter of the BA ($\alpha = 0.25$), the median ignores single-frame outliers instead of averaging them in. The result is valid only once the window holds $n$ frames. The window starts over after an invalid frame and when the new centre lies more than $0.25\,s_\text{px}$ from the current median (`centre_reset_jump_ratio`), so a moved camera or nut is never mixed with the old position. Cost: a delay of $n$ frames (0.25 s at 20 fps), irrelevant at the robot, where the camera stands still during the observation.

### 2.8 Bench evaluation on a printed grid – **used** (`sa_vision/grid_fit.py`, `evaluate_grid`)
Without the robot, the observation conditions are reproduced on a stand: camera fixed above the table looking down, fixed exposure, reference image, $d$ known. A printed sheet carries a grid of $7 \times 4$ marks with a pitch of 30 mm (`tools/make_grid_sheet.py`, scale checked with a ruler). The nut is placed on each mark in turn (random rotation, one image per mark).

**Fit.** For a camera looking straight down, the undistorted normalised image coordinates $\mathbf n_i = ((u_i - c_x)/f_x,\ (v_i - c_y)/f_y)$ of the nut centres and the sheet coordinates $\mathbf p_i$ of the marks follow a similarity

$$
\mathbf n_i = \frac{1}{d}\,\mathbf R(\theta)\,\mathbf p_i + \mathbf t
$$

with unknown sheet rotation $\theta$, offset $\mathbf t$ and scale $1/d$. It is fitted in closed form by least squares [Umeyama 1991]. The residuals multiplied by $d$ are the position errors on the object plane in mm; $d$ itself comes out of the fit. A full plane homography (8 parameters, DLT [Hartley & Zisserman]) additionally absorbs a tilt between the sheet and the optical axis; its residuals are the part of the error that no plane mapping explains.

**Results, bench set of 16.10.2026** (diffuse lamp, exposure 10 ms, reference image, M8 locknut, marks 24 and 25 hidden by the gripper):

| Quantity | Level threshold (2.2) | Edges (2.2a) |
|---|---|---|
| Nuts detected on the grid | 4 / 26 | **26 / 26** |
| Extra images as expected (3 empty, 2 stud only, 2 stud + M8, 2 M8 + large nut, 2 M8 cut by the border) | 7 / 11 | **11 / 11** |
| Position error, similarity (RMS / max) | – | 1.45 / 2.55 mm |
| Position error, homography (RMS) | – | 1.18 mm |
| Width across flats at the fitted $d$ | – | 13.30 ± 0.34 mm |

**Lighting is a requirement, not a detail.** With a low, point-like light source instead of the diffuse lamp (live test 16.10., table half as bright, long hard shadow), the flank of the nut next to its shadow has no contrast: the nut is as dark as the shadow, no edge exists there, and the visible part measures only 9.8 mm. No segmentation can recover an outline that is not in the image. The setup therefore needs diffuse light from above (or around the camera); at the robot the lamp position is part of the observation pose.

The error vectors show no systematic pattern (no radial or one-sided trend), so the remaining error is dominated by the placement of the nut by hand on a mark hidden under it, and the 1.45 mm RMS is an upper bound of the detection error.

**Calibration check with the printed crosses.** The crosses of the reference image have exact sheet positions and need no manual placement. Fitting 19 of them (intensity-weighted centroids): homography residual 0.06 mm RMS, so intrinsics and distortion model are consistent over the field of view; similarity residual 0.50 mm RMS (max 1.0 mm), i.e. about 0.5 mm of the nut error comes from a slight tilt of the stand camera against the sheet.

**Effective distance.** The fit gives $d = 419.4$ mm from the projection centre to the sheet, whereas the ruler gave 410 mm from the lowest point of the camera housing. The nut fit is consistent with this ($d = 415.8$ mm to the nut centres, i.e. about 4 mm below the top face of the 8 mm nut). The projection centre therefore lies about 9 mm above the lowest point of the housing (about 4 mm above the lens, which sits 5 mm higher), or the focal length is about 2 % off; both have the same effect on the target. The CAD offset of the camera (Section 0, $\Delta z = 57.223$ mm) refers to the lowest point of the camera housing, the same point the ruler was placed on, so $d$ from forward kinematics would be about 9 mm too small, an error of 2 % of the lateral distance from the optical axis (2 mm at 100 mm, Section 4). It is calibrated at the robot with one image of the grid sheet from the observation pose (fitted $d$ against $d$ from forward kinematics) and stored as a constant offset.

---

## 3. From pixel to robot target – **used** (`sa_vision/geometry.py`, `sa_tasks/frames.py`, `sa_tasks/sequences.py`)

### 3.1 Back-projection as ray–plane intersection
After undistortion, the pixel defines a viewing ray in the camera frame:

$$
\mathbf{r}_C = \mathbf{K}^{-1} (u, v, 1)^\top
$$

The camera pose in the base frame follows from forward kinematics and the fixed hand–camera transform:

$$
{}^{B}\mathbf{T}_{C} = {}^{B}\mathbf{T}_{F}(\mathbf{q})\ {}^{F}\mathbf{T}_{C}
= \begin{pmatrix} \mathbf{R} & \mathbf{o} \\ \mathbf{0}^\top & 1 \end{pmatrix}
$$

The ray in the base frame is $\mathbf{p}(\lambda) = \mathbf{o} + \lambda\, \mathbf{R}\,\mathbf{r}_C$. Intersecting it with the horizontal object plane $z = z_\text{obj}$ (table height plus object height) gives

$$
\lambda^* = \frac{z_\text{obj} - o_z}{(\mathbf{R}\,\mathbf{r}_C)_z}, \qquad \mathbf{p}_\text{obj} = \mathbf{o} + \lambda^* \mathbf{R}\,\mathbf{r}_C
$$

This is valid for any camera orientation, as long as the ray is not parallel to the plane and $\lambda^* > 0$ (plane in front of the camera).

**Frames in the implementation.** The robot state gives the pose of the libfranka end-effector frame $\{E\}$ (configured in Desk) and its transform relative to `fr3_link8` $\{L\}$ (libfranka's "flange" frame). The flange frame $\{F\}$ of Section 0 has the same origin as $\{L\}$ and is turned by −45° about $z$: ${}^{L}\mathbf{T}_{F} = (\mathbf{R}_z(-45°), \mathbf{0})$. Hence ${}^{B}\mathbf{T}_{F} = {}^{B}\mathbf{T}_{E}\,({}^{L}\mathbf{T}_{E})^{-1}\,{}^{L}\mathbf{T}_{F}$, and the controller is commanded with ${}^{B}\mathbf{T}_{E}^{d} = {}^{B}\mathbf{T}_{F}^{d}\,({}^{L}\mathbf{T}_{F})^{-1}\,{}^{L}\mathbf{T}_{E}$. The task code therefore works in $\{F\}$ and does not depend on the end-effector setting in Desk; with the Franka Hand set in Desk, ${}^{F}\mathbf{T}_{E}$ is a pure translation and the commanded end-effector quaternion is exactly $(1, 0, 0, 0)$. ${}^{F}\mathbf{T}_{C}$ has the rotation ${}^{F}\mathbf{R}_{C} = \mathbf{R}_z(+90°)$ (Section 0) and the translation ${}^{F}\mathbf{t}_{C} - (0, 0, \Delta_c)^\top$, where $\Delta_c = 9$ mm is the height of the projection centre above the lowest point of the housing (Section 2.8; to be calibrated at the robot).

**Gripper mount tilt – used, optional (`use_gripper_tilt`).** A printed adapter can hold the gripper slightly tilted against the flange. The gripper frame $\{G\}$ is then $\{F\}$ rotated by a small ${}^{F}\mathbf{R}_{G}$ about the common origin, and the CAD offsets (camera, TCP) are defined in $\{G\}$. ${}^{F}\mathbf{R}_{G}$ is measured with the bottom plate pressed flat on the table, where $\{G\}$ is level: ${}^{F}\mathbf{R}_{G} = ({}^{B}\mathbf{R}_{F})^\top\,\operatorname{lev}({}^{B}\mathbf{R}_{F})$, with $\operatorname{lev}$ keeping the yaw and setting the tool axis to $-z_B$; several samples are averaged as rotation vectors. With the switch on, every motion commands $\{G\}$ to the fixed orientation, i.e. ${}^{B}\mathbf{R}_{F} = {}^{B}\mathbf{R}_{G}^{d}\,({}^{F}\mathbf{R}_{G})^\top$, so the bottom plate lands flat and the camera on the gripper looks straight down; camera and TCP are computed as ${}^{B}\mathbf{T}_{F}\,{}^{F}\mathbf{T}_{G}\,{}^{G}\mathbf{T}_{C}$. A mount tilt $\varepsilon$ explains a constant target offset of $d \tan\varepsilon$ (Section 4); the hand-guided reference points of 18.10. showed flange tilts of 0.9–3.2° with the plate on the table.

**Object planes.** The plane must be the height of the point whose image is the detected centre. For the nut this is the centroid of the silhouette, which lies at about half the nut height (bench fit, Section 2.8): $z_\text{obj} = z_\text{table} + 4$ mm. For the stud it is the top face of the square base, $z_\text{table} + 20$ mm. The distance used by the size filter of the detection is computed separately for the face whose size is checked (nut top face, base top face): $d = (o_z - z_\text{face}) / (-\mathbf{R}_{:,3})_z$.

### 3.2 Special case: camera looking straight down
If the optical axis is vertical, $\lambda^* = d$ and the result reduces to the familiar pinhole relation

$$
X_C = \frac{(u - c_x)\, d}{f_x}, \qquad Y_C = \frac{(v - c_y)\, d}{f_y}
$$

The vision node publishes this point $(X_C, Y_C, d)$ in the camera frame for debugging; the task code uses the general intersection of 3.1 with the measured pose, so a small tilt of the camera is handled.

### 3.3 Tool target
The commanded TCP position is $\mathbf{p}_\text{obj}$ in $x, y$ plus a constant correction $\Delta\mathbf{p}$ from the verification measurement (Section 4); $z$ is set per phase. The orientation is fixed to the quaternion $(1, 0, 0, 0)$, ${}^{B}\mathbf{R}_{F} = \operatorname{diag}(1, -1, -1)$ (gripper down, camera towards $+x_B$, Section 0); taught poses contribute only their position. The flange target that puts the TCP at $\mathbf{p}_T$ is

$$
{}^{B}\mathbf{t}_{F} = \mathbf{p}_T - {}^{B}\mathbf{R}_{F}\,{}^{F}\mathbf{t}_{T}
$$

- **Pick (step 1):** $\mathbf{p}_T = (x_\text{nut}, y_\text{nut}, z_\text{table} - \delta_p)$ with a press depth $\delta_p = 2$ mm, so the bottom plate rests flat on the table with a small preload ($k_z \delta_p = 1500\ \text{N/m} \cdot 2\ \text{mm} = 3$ N) and the iris, centred on the TCP axis, surrounds the nut.
- **Place (step 2):** the nut sits in the iris with its bottom face in the TCP plane. Approach: $\mathbf{p}_T = (x_\text{stud}, y_\text{stud}, z_\text{table} + 39\ \text{mm} + 5\ \text{mm})$, i.e. 5 mm above the tip of the thread; then a vertical contact search (Section 5).
- **Move per image:** from the observation pose, the horizontal TCP move splits into the part the image gives (object relative to the point where the optical axis meets the object plane) and the fixed part of the mount (TCP → that point, $\approx (+44.4,\ 0)$ mm in $x_B, y_B$ with a level camera):
  $\Delta\mathbf{p}_{xy} = (\mathbf{p}_\text{obj} - \mathbf{p}_\text{axis})_{xy} + (\mathbf{p}_\text{axis} - \mathbf{p}_{T,\text{obs}})_{xy}$.
  The target comes from the frame chain above, the split is only logged and stored in the run record (`tcp_move`), so a wrong mount offset shows up as a wrong second term.
- **Plausibility:** a target farther than 70 mm from the camera axis (measured on the object plane) is rejected as outside the work area, before the robot moves.

### 3.4 Comparison with the BA approach
The BA used image-based visual servoing in its simplest form: a proportional law on the normalised pixel error, $\Delta\mathbf{x} = K_p\, \mathbf{e}_\text{px}$, iterated until $\|\mathbf{e}_\text{px}\|$ fell below a threshold. This needs no calibration but converges slowly and oscillated in one of eight runs. The direct (position-based) method needs $\mathbf{K}$, ${}^{F}\mathbf{T}_{C}$ and $z_\text{obj}$, but reaches the target with a single motion.

---

## 4. Error budget – **planned**

Differentiating 3.2 gives the sensitivity of the target position:

$$
\delta X \approx \underbrace{\frac{d}{f}\,\delta u}_{\text{pixel error}} \;+\; \underbrace{\frac{X}{d}\,\delta d}_{\text{height error}} \;+\; \underbrace{\frac{X}{f}\,\delta f}_{\text{focal error}}
$$

- One pixel corresponds to $d/f$ millimetres on the object plane (e.g. $d = 300$ mm, $f = 1000$ px → 0.3 mm/px; actual $f$ after calibration).
- Height and focal-length errors scale with the lateral distance $X$ from the optical axis. Objects near the image centre are therefore less sensitive, which is an argument for an observation pose above the middle of each work area.
- The height error includes the position of the projection centre inside the camera, which is not the lowest point of the camera housing assumed by the CAD offset (about 9 mm higher on the bench, Section 2.8).
- Systematic errors (CAD offset ${}^{F}\mathbf{T}_{C}$, small camera tilt) appear as a constant offset and are removed by the verification measurement: place the object at a known position, drive to the computed target, measure the residual, store it as $\Delta\mathbf{p}$. A camera tilt $\varepsilon$ against the flange shifts the target by about $d \tan\varepsilon$ (7 mm per degree at $d = 400$ mm), the dominant term for a 3D-printed mount; it is constant for a fixed observation pose.
- **Verification procedure – used** (`sa_tasks/verification.py`): for each of $n$ nut positions, the gripper is guided by hand flat onto the table with the iris centred over the nut, and the TCP position is stored as the truth $\mathbf{p}_i^\ast$ (`teach_poses`, command `truth`); `pick --locate-only` gives the computed position $\hat{\mathbf p}_i$ from the observation pose. The correction is the mean $\Delta\mathbf p = \frac1n \sum_i (\mathbf p_i^\ast - \hat{\mathbf p}_i)$; the spread of the residuals $\mathbf p_i^\ast - \hat{\mathbf p}_i - \Delta\mathbf p$ is the error a constant correction cannot remove (it would grow with the distance from the camera axis for a scale or tilt error). **First result (18.10., 4 nut positions 51–115 mm from the axis):** robot execution error (landed TCP − commanded target) 0.2–0.7 mm; computation error nearly constant, $\Delta\mathbf p = (-9.0,\ +8.2)$ mm, which corresponds to a camera tilt of about 1.5° on the mount; residual after the constant correction 2.5–4.5 mm. A similarity fit (scale 1.025, rotation −1.6°) leaves 1.4–3.2 mm but has as many parameters as points.

---

## 5. Motion – **used** (`sa_tasks/trajectory.py`, `sa_tasks/sequences.py`)

The Cartesian impedance controller tracks a reference pose; the task code streams the reference at 100 Hz (the controller additionally low-pass filters it and limits its speed). Every move is **one continuous minimum-jerk motion** from the measured pose to the goal,

$$
\sigma(s) = 10 s^3 - 15 s^4 + 6 s^5, \qquad s \in [0, 1],
$$

which starts and ends with zero velocity and acceleration (peak speed $1.875\times$ the mean speed). To arrive vertically over the nut without sweeping the gripper over it, the horizontal and vertical components use different time scales: going down, $x, y$ follow $\sigma(t / \lambda T)$ and $z$ follows $\sigma(t / T)$ with $\lambda = 0.6$, so the horizontal motion is finished at $0.6\,T$ and the rest of the descent is vertical (from an observation height of 350 mm, the last $\approx 55$ mm). Going up, $z$ leads instead, so the gripper leaves the table vertically. The orientation is interpolated by spherical linear interpolation with $\sigma(t/T)$. The duration $T$ is the smallest value for which no axis group exceeds the peak speed (100 mm/s, 20 °/s in the first robot runs).

- **Optional intermediate stop (pick):** the move ends 30 mm above the table straight over the nut; the task waits until the measured orientation is within 0.2° of the commanded one (at most 5 s) and then descends vertically. The measured tilt at the observation pose, at the stop and after landing is recorded.
- **Arrival:** the goal is reached when the arm stands still (flange speed below 2 mm/s for 0.5 s) within 8 mm of the goal (plus the press depth for the pick, where the table stops the arm before the reference). A tight position tolerance is not needed: the impedance controller keeps a steady-state error of a few millimetres (3.6 mm in the first robot run, 18.10.), and the target computation uses the measured pose, not the commanded one. The remaining error is logged per axis.
- **Contact search (step 2):** from the approach pose the reference moves straight down at 5 mm/s; contact is declared when the estimated external force along $z_B$ differs by more than 4 N from its median before the descent (the difference removes the bias of an unmodelled tool mass). The reference is then set to the measured pose.
- **Screwing (step 2):** the controller switches to its compliant preset (low stiffness in all axes), the reference is set 10 mm below the contact pose, and the gripper rotates the nut for a fixed time. The low stiffness lets the arm follow the thread down and centre itself on the stud, with a small downward preload. Before switching back to the stiff preset the reference is reset to the measured pose, so the arm does not jump.

---

## References (to complete)

- Z. Zhang, "A flexible new technique for camera calibration," *IEEE TPAMI*, 22(11), 2000.
- N. Otsu, "A threshold selection method from gray-level histograms," *IEEE Trans. SMC*, 9(1), 1979.
- S. Suzuki, K. Abe, "Topological structural analysis of digitized binary images by border following," *CVGIP*, 30(1), 1985.
- G. T. Toussaint, "Solving geometric problems with the rotating calipers," *Proc. IEEE MELECON*, 1983.
- D. C. Brown, "Decentering distortion of lenses," *Photogrammetric Engineering*, 32(3), 1966.
- S. Umeyama, "Least-squares estimation of transformation parameters between two point patterns," *IEEE TPAMI*, 13(4), 1991.
- R. Hartley, A. Zisserman, *Multiple View Geometry in Computer Vision*, 2nd ed., Cambridge University Press, 2004.
- F. Chaumette, S. Hutchinson, "Visual servo control, Part I: Basic approaches," *IEEE Robotics & Automation Magazine*, 13(4), 2006.
