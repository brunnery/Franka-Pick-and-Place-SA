# Theory and Methods – Collection for the Report

> Living document. Every formula, model or principle that the code relies on and that belongs in the report is written down here, in report-ready English, with symbols defined once.
> Status per section: **used** (in the code), **planned** (decided, not implemented yet), **dropped** (kept for the discussion chapter).
> Last update: 14.10.2026 (detection and back-projection implemented in `sa_vision`)

---

## 0. Notation and frames

| Symbol | Meaning |
|---|---|
| $\{B\}$ | robot base frame (FR3 `fr3_link0`) |
| $\{F\}$ | flange frame (`fr3_link8`) |
| $\{C\}$ | camera frame, OpenCV convention: $x_C$ right in the image, $y_C$ down in the image, $z_C$ along the optical axis into the scene |
| $\{T\}$ | tool frame (TCP): centre of the gripper bottom plate |
| ${}^{A}\mathbf{T}_{B} \in SE(3)$ | homogeneous transform that maps coordinates in $\{B\}$ to $\{A\}$ |
| $(u, v)$ | pixel coordinates (column, row), origin top left |
| $\mathbf{K}$ | camera intrinsic matrix |
| $d$ | distance from the camera centre to the object plane along $z_C$ |

Fixed transforms (CAD, 08.10.2026, mm):

$$
{}^{F}\mathbf{t}_{C} = (0,\ 44.4,\ 60.223)^\top, \qquad {}^{F}\mathbf{t}_{T} = (0,\ 0,\ 113.823)^\top
$$

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

## 2. Object detection – **used** (nut, `sa_vision/detection.py`; stud: planned)

Goal: the centre $(u, v)$ of the nut (pick) and of the stud base (place), from a single observation pose, on a white background, without painting the parts (in the BA they were painted matt black). The nut detection does not assume a nut size; it measures it.

**Appearance of the bare nut (test images of 15.10.2026, hand-held, 27 images).** The zinc-plated nut reflects specularly. Most of its body is *darker* than the white table (ratio to the table about 0.5–0.8), a few highlights are *brighter* (up to 1.2), and some facets are as bright as the table. The thread hole is dark, not bright: through the bore the camera sees the table in the nut's own shadow (ratio about 0.45–0.7). On a locknut the blue nylon ring merges with the hole into one dark disc. Each nut casts a soft shadow that is 20–40 % darker than the table. The previous segmentation (inverted Otsu threshold, hole as a bright region, Sections 2.2–2.4 of the earlier version) found 0 of 27 nuts on these images; it is **dropped** (see 2.2).

### 2.1 Image acquisition and pre-processing – **used**
1. **Fixed exposure and white balance** (`software/config/camera.yaml`, set with `v4l2-ctl` before the camera starts). With automatic exposure the brightness depends on the scene content (a bright table drives the gain down), so a reference image of the empty work area and a live image with a nut would not share the same grey levels. Trade-off measured in office light: the automatic mode reaches a table grey value of about 190 at 24 fps; manual exposure needs about 120 ms (4 fps) for the same brightness, 50 ms gives about 110 at 20 fps. The scene is static during the observation, so a low frame rate is acceptable.
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
2. **Local median** (method A, and applied after step 1 in A'): $B$ is the median of the (divided) image over a square window of side $w = k\, s_\text{max}\, f / d$, with $s_\text{max}$ the largest accepted width across flats and $k = 1.25$. The median ignores objects that cover less than half of the window, so the nut does not enter $B$; the window still follows slow variations of the table brightness. Near the nut, the window also contains the nut's soft shadow, which lowers $B$ there and removes most of the shadow from $r$. A larger window ($k = 2$) lets the shadow through: it then becomes part of the silhouette and the nut fails the shape filter (18 instead of 20 of 23 M8 nuts found). The value $k = 1.25$ was chosen on the same 27 images. For speed the median runs on an image downscaled by 8 and is interpolated back.

The step 2 suppression is partial. On synthetic images a shadow of the measured shape is removed up to a depth of about 0.25; the real shadows of up to 0.4 on the test images were removed as well, because they are softer.

### 2.3 Morphology – **used**
Closing ($A \bullet B = (A \oplus B) \ominus B$, two iterations, elliptic $7 \times 7$ element) first joins highlights, dark facets and the hole into one region; opening ($A \circ B = (A \ominus B) \oplus B$) then removes small specks.

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

### 2.6b Source of the distance $d$ – **used** (`d_source` in `vision.yaml`)
The metric quantities need the distance $d$ from the camera to the nut top face.
- **`yaml`**: $d$ is a known value, `object_plane_distance_mm`; at the robot it will come from forward kinematics (Section 3). The size $\hat s = s_\text{px} d / f$ is then *measured*.
- **`from_size`** (hand-held tests without the robot, at any height): if the nut size $s^\ast$ is known (`target_across_flats_mm`), the pinhole relation is inverted,
$$
\hat d = \frac{f\, s^\ast}{s_\text{px}}
$$
The size is then *assumed*, not measured: a size filter on $\hat s$ would be circular ($\hat s = s^\ast$ for every candidate). The size filter is therefore relaxed to a plausibility check of the implied distance, $\hat d \in [d_\text{min}, d_\text{max}]$ (`d_from_size_range_mm`, 200–500 mm), i.e. $s_\text{px} \in [f s^\ast (1-t)/d_\text{max},\ f s^\ast (1+t)/d_\text{min}]$. Consequences: a nut of another size is taken for an M8 at another distance (an M12 next to an M8 gives "2 candidates" instead of being rejected), and every error in $s_\text{px}$ goes directly into $\hat d$: $\delta \hat d / \hat d = -\delta s_\text{px} / s_\text{px}$. The bias of about +2.5 px found for $s_\text{px}$ (Section 2.6a) gives about 3 % too small $\hat d$ at 340 mm, and through Section 4 a lateral error of $X \cdot 0.03$ (0.9 mm at $X = 30$ mm). The debug image states "d estimated from size". On the 27 test images the implied distances were 273–390 mm, consistent with the estimate of 15.10.; 19 of 23 M8 nuts passed all filters, 14 images gave a valid single detection (the 5 images with the large nut give "2 candidates").

**Debug image** (`/vision/debug`): the camera frame ($x_\text{cam}$ right, $y_\text{cam}$ down) and the robot frame ($x_\text{robo} = -y_\text{cam}$, $y_\text{robo} = -x_\text{cam}$, gripper pointing down) are drawn in the principal point $(c_x, c_y)$. For a detected nut it shows $\hat s$, $d$ and the offset of the nut from the optical axis in the robot frame, $(\Delta x_\text{robo}, \Delta y_\text{robo}) = (-Y_C, -X_C)$ with $(X_C, Y_C)$ from Section 3.2, i.e. the distance the robot would have to move to bring the optical axis over the nut.

### 2.7 Temporal robustness – **planned**
The centre is the **median** over $n$ consecutive frames. The median rejects single-frame outliers from reflections, which the exponential filter of the BA ($\alpha = 0.25$) only damps.

---

## 3. From pixel to robot target – **used** (camera frame, `sa_vision/geometry.py`); base frame and tool target: planned

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

### 3.2 Special case: camera looking straight down
If the optical axis is vertical, $\lambda^* = d$ and the result reduces to the familiar pinhole relation

$$
X_C = \frac{(u - c_x)\, d}{f_x}, \qquad Y_C = \frac{(v - c_y)\, d}{f_y}
$$

The vision node currently publishes this point $(X_C, Y_C, d)$ in the camera frame, with $d$ taken from the configuration (observation height). The transformation into $\{B\}$ with Section 3.1 follows once the forward kinematics are available.

### 3.3 Tool target
The commanded TCP position is $\mathbf{p}_\text{obj}$ in $x, y$; $z$ is set per phase (approach height, contact). The orientation is fixed (gripper down, camera towards $+x_B$). A constant correction $\Delta\mathbf{p}$ from the verification measurement (Section 4) is added in $x, y$.

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
- Systematic errors (CAD offset ${}^{F}\mathbf{T}_{C}$, small camera tilt) appear as a constant offset and are removed by the verification measurement: place the object at a known position, drive to the computed target, measure the residual, store it as $\Delta\mathbf{p}$.

---

## References (to complete)

- Z. Zhang, "A flexible new technique for camera calibration," *IEEE TPAMI*, 22(11), 2000.
- N. Otsu, "A threshold selection method from gray-level histograms," *IEEE Trans. SMC*, 9(1), 1979.
- S. Suzuki, K. Abe, "Topological structural analysis of digitized binary images by border following," *CVGIP*, 30(1), 1985.
- G. T. Toussaint, "Solving geometric problems with the rotating calipers," *Proc. IEEE MELECON*, 1983.
- D. C. Brown, "Decentering distortion of lenses," *Photogrammetric Engineering*, 32(3), 1966.
- R. Hartley, A. Zisserman, *Multiple View Geometry in Computer Vision*, 2nd ed., Cambridge University Press, 2004.
- F. Chaumette, S. Hutchinson, "Visual servo control, Part I: Basic approaches," *IEEE Robotics & Automation Magazine*, 13(4), 2006.
