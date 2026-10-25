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
19.10.2026, 1280 × 720 px, board with 8 × 11 inner corners and 30 mm squares; mean of three independent runs (73, 74 and 57 views, new images in each run, camera fixed on the robot, board moved by hand). File: `calibration/hd_camera.yaml`.

| Parameter | Value |
|---|---|
| $f_x,\ f_y$ | 2096.0 px, 2092.3 px |
| $c_x,\ c_y$ | 695.4 px, 404.1 px |
| $k_1, k_2, p_1, p_2, k_3$ | 0.077, −0.479, −0.0026, 0.0020, 0 |
| Field of view (h × v) | $2\arctan\frac{W}{2f_x} = 34.0°$ × $2\arctan\frac{H}{2f_y} = 19.5°$ |

**Repeatability.** Between the three runs the standard deviation is 2.9 / 4.8 px in $f_x$ / $f_y$ and 7.9 / 5.1 px in $c_x$ / $c_y$ (range 15.8 px in $c_x$). Mapped onto a plane at $d = 416$ mm (undistortion plus pinhole), a single run deviates from the mean by at most 2.1 mm anywhere in the image, two runs from each other by at most 3.5 mm. The first calibration (14.10., 68 views: $f_x = 2077.2$, $f_y = 2083.7$, $c_x = 695.4$, $c_y = 405.6$ px) lies within 0.75 mm of the mean, 0.3 mm at the image centre. An earlier check with two overlapping sample sets (< 1 px in $(c_x, c_y)$) was therefore too optimistic. The intrinsics contribute about ±2 mm to the target error and cannot explain the constant offset of about (−9, +8) mm found at the robot (Section 3.3): that would need a principal point error of about 45 px.

Consequences for the method:
- **Ground resolution** $d/f$: 0.095 mm/px at $d = 200$ mm, 0.143 mm/px at 300 mm, 0.191 mm/px at 400 mm. The M8 test nut (13 mm across flats) spans about 90 px at 300 mm, an M4 nut (7 mm) still about 48 px.
- **Narrow field of view:** at $d = 300$ mm the image covers only about 185 × 104 mm on the table. Each work area must fit into this footprint from its observation pose.
- **Principal point offset:** the optical axis meets the image at $(695, 406)$, i.e. 55 px right of and 46 px below the image centre $(640, 360)$. At $d = 300$ mm this is about 8 mm and 7 mm on the table. Centring an object on the geometric image centre, as in the BA, therefore does not place it on the optical axis; the back-projection in Section 3 uses $(c_x, c_y)$ and avoids this bias.

---

### 1.5 Camera rotation in the flange from pure translations – **used** (25.10.2026, `sa_tasks/hand_eye.py`, `hand_eye_solver.py`)

Until 25.10.2026 the camera's rotation in the flange came from the CAD plus a tilt of 1.25° about $x_F$ measured once by centring the camera over the stud; the component about $y_F$ was never measured. A full hand-eye calibration ($AX = XB$, e.g. Tsai–Lenz) needs flange rotations about non-parallel axes, which the fixed tool orientation of this project excludes. With **pure translations** the rotation can still be found. A ChArUco board (9 × 13 squares of 20 mm; corners are found even when only part of the board is in view, which matters with a field of view of about 250 × 140 mm) lies flat on the table. The flange moves over a grid of $n$ positions $\mathbf{p}_i$ (here $3 \times 3 \times 3$: ±40 mm, ±50 mm, ±80 mm around the stud observation pose) with measured orientations $R_i$ (nominally constant, in practice within about 1°), and at each one `solvePnP` on the board corners gives the board origin $\mathbf{o}_i$ in the camera frame. With the camera position $\mathbf{t}$ in the flange (CAD) and the unknown rotation $R_{FC}$, every view must put the board at the same base position $\mathbf{b}$:
$$
\mathbf{p}_i + R_i\,(\mathbf{t} + R_{FC}\,\mathbf{o}_i) = \mathbf{b}.
$$
Writing $R_{FC} = \exp([\boldsymbol{\delta}]_\times)\,R_k$ and $\mathbf{v}_i = R_k \mathbf{o}_i$, the equations are linear in the small rotation $\boldsymbol{\delta}$ and in $\mathbf{b}$, $R_i [\mathbf{v}_i]_\times \boldsymbol{\delta} + \mathbf{b} = \mathbf{p}_i + R_i(\mathbf{t} + \mathbf{v}_i)$; a few Gauss–Newton steps from the CAD rotation converge (synthetic test: exact without noise, < 0.15° with 0.5 mm noise per view). Because $R_i$ is almost constant, $\mathbf{t}$ and $\mathbf{b}$ cannot be separated, so $\mathbf{t}$ stays at the CAD value: an error in it only shifts all targets by a constant, while a rotation error $\varepsilon$ shifts a target by about $d\,\varepsilon$ ($d \approx 400$ mm: 7 mm per degree), which is why the rotation matters. The result is expressed as $R_{FC} R_\text{CAD}^{-1} = R_x(a) R_y(b) R_z(c)$, i.e. the existing `camera_tilt_in_flange_deg` $(a, b)$ plus a rotation $c$ about the optical axis. **Result 25.10.2026** (two runs, board shifted slightly in between): $(a, b, c)$ = (1.53°, 0.76°, 1.19°) and (1.54°, 0.81°, 1.40°), residual RMS 0.96 and 1.12 mm; adopted the mean (1.54°, 0.79°, 1.30°), replacing (1.25°, 0°) set by eye on 22.10.2026. The previously unmeasured $b$ alone moves a target by about $400\ \text{mm} \cdot \tan 0.79° \approx 5.5$ mm. The camera's distance scale differed from the robot's by 1–2 % (focal length), and the board normal came out about 2° from vertical in both runs (a tilted sheet or a principal-point error; the vision uses the same intrinsics, so the fit stays consistent with it). Quality measures: the RMS of the per-view residual $\lVert \mathbf{p}_i + R_i(\mathbf{t} + R_{FC}\mathbf{o}_i) - \mathbf{b} \rVert$, and the angle between the board normal $R_i R_{FC} R_{C,\text{board},i}\,\mathbf{e}_z$ and the vertical (zero for a flat board on a level table, an independent check of the tilt).

## 2. Object detection – **used**: simple finder (2.0, `sa_vision/vision_node.py`); the earlier full pipeline 2.1–2.8 is **dropped** (code removed 19.10.2026, text kept for the report)

Goal: the centre $(u, v)$ of the nut (pick) and of the stud base (place), from a single observation pose, on a white background, without painting the parts (in the BA they were painted matt black). The nut detection does not assume a nut size; it measures it.

**Appearance of the bare nut (test images of 15.10.2026, hand-held, 27 images).** The zinc-plated nut reflects specularly. Most of its body is *darker* than the white table (ratio to the table about 0.5–0.8), a few highlights are *brighter* (up to 1.2), and some facets are as bright as the table. The thread hole is dark, not bright: through the bore the camera sees the table in the nut's own shadow (ratio about 0.45–0.7). On a locknut the blue nylon ring merges with the hole into one dark disc. Each nut casts a soft shadow that is 20–40 % darker than the table. The previous segmentation (inverted Otsu threshold, hole as a bright region, Sections 2.2–2.4 of the earlier version) found 0 of 27 nuts on these images; it is **dropped** (see 2.2).

### 2.0 Simple object finder – **used** (since 18.10.2026; package `simple_vision`, renamed to `sa_vision` on 19.10.2026)
For the first runs at the robot, the detection assumes exactly one object on a blank background and checks neither size nor shape. The background is estimated by heavy Gaussian smoothing, $B = G_{\sigma} * I$ with $\sigma = 60$ px (much larger than the object, about 60 px across at 400 mm), computed on an image downscaled by 8. The ratio image $r = I / B$ removes the illumination (table $r \approx 1$, black 0, specular edges above 1). The segmentation is a band-stop around the table: foreground where $|r - 1| > 0.18$, in either direction, so both the dark body and the specular edges count. The half-width 0.18 was chosen at the robot on 19.10.2026 (live frames: table 0.9–1.1, soft shadow below the nut 0.8–0.9, nut body 0.45–0.85, edges 1.0–2.0); 0.10 kept too much shadow. Tried and **dropped** on 19.10.: an inverse binary threshold $r < 0.75$ (dark parts only; bench RMS 1.30 vs 1.35 mm, size closer to 13 mm, but no gain at the robot) and the hole method of 2.0b. After closing and opening, the largest blob that does not touch the image border is taken (parts of the robot reaching into the view and objects cut by the border are excluded).

**Hexagon fit.** For an orientation $\theta$, the convex hull $\{\mathbf p_i\}$ is enclosed by three pairs of parallel lines with normals $\mathbf n_k = (\cos(\theta + k\,60°), \sin(\theta + k\,60°))$, $k = 0, 1, 2$: the widths are $w_k = \max_i \mathbf n_k^\top \mathbf p_i - \min_i \mathbf n_k^\top \mathbf p_i$ and the midlines $\mathbf n_k^\top \mathbf c = m_k$ with $m_k$ the mean of the two extremes. For a regular hexagon the three widths are equal at every $\theta$ by symmetry, so the criterion is the tightest enclosing hexagon, $\min_\theta \sum_k w_k$ (1° steps over 0–60°), which touches the flats. Its centre is the least-squares solution of the three midline equations. For any centrally symmetric outline (hexagon, square base of the stud) the midlines meet in the centre of symmetry.

**Stud (step 2)** – **used** since 20.10.2026, not yet validated on real images. The M8 stud stands on a square base ($10 \times 10 \times 20$ mm, thread 19 mm, 39 mm in total). The same finder is used; the vision node's parameter `target` (set by `sa_tasks`) selects the polygon: for the stud the hull is enclosed by two pairs of parallel lines with normals $\theta, \theta + 90°$ ($\theta \in [0°, 90°)$), i.e. the tightest enclosing square; in general a regular polygon with $2m$ sides uses $m$ pairs $180°/m$ apart, with circumradius $R = (s/2)/\cos(\pi/2m)$. Seen from above, the thread (8 mm) lies inside the square, so the outline stays square-like. The multi-object filter uses its own band for the width ratio ($1/\sqrt2 = 0.707$ for a square; accepted $[0.60, 0.82]$, which rejects a nut at 0.866) and target size $s = 10$ mm, $d$ measured to the top face of the base.

*Stud search by centring* – **used** since 20.10.2026. First test at the robot (20.10.): seen off-axis, the 39 mm stud does not give a square outline at all (walls and thread produce shapes that depend on the viewing angle), so its shape cannot be judged from the observation pose. Therefore: from the observation pose, the objects in view are listed and ranked. A loose filter first drops specks ($s < 6$ mm, dirt on the table) and lines (width ratio $< 0.4$, table marks); a size-based ranking failed at the robot (20.10.), because the stud seen off-axis appears much larger than 10 mm (17.7 mm at 46 mm from the axis) and a 4 mm speck was closer to the target size. The remaining objects are sorted by the area of their hull, largest first: the stud is by far the largest dark object, and the order is stable from frame to frame. (Sorting clean squares first, then by the distance from the image centre, let a speck that happened to look square jump to rank 1 in some frames, 20.10.; with the stud off-axis no square, the shape is only judged once the camera is centred.) The speck limit was raised from 6 to 8 mm.

*Edge refinement of the stud centre* – **used** since 21.10.2026. From directly above, only a rim of about 1 mm of the dark base is visible around the bright thread, and the soft shadow of the 20 mm high base is about as dark as the base itself under the ceiling light (ratio to the blurred background: base 0.4–0.75, shadow 0.6–0.8, depending on the room light), so no fixed threshold separates them. The dark blob then contains part of the shadow on one side: the hull becomes a rectangle (e.g. 11.0 × 9.3 mm for the 10 mm base), and the midlines of the enclosing square move towards the shadow by half its width. The base edges, however, are sharp and the shadow edge is soft. With the square's orientation $\theta$ from the coarse fit and its axes $\mathbf n_1 = (\cos\theta, \sin\theta)$, $\mathbf n_2 = (-\sin\theta, \cos\theta)$, the centre is refined per axis: a pair of parallel lines $L$ apart, with $L = s^\ast f / d$ the known base size in pixels, is shifted by $\delta \in [-0.25L, 0.25L]$ (0.25 px steps), and
$$\delta_k^\ast = \arg\max_\delta \sum_{\sigma = \pm 1/2} \frac{1}{N} \sum_{j=1}^{N} \left| \nabla I\big(\mathbf c_0 + (\delta + \sigma L)\,\mathbf n_k + t_j\,\mathbf n_{3-k}\big)^\top \mathbf n_k \right|,$$
sampled at $N = 25$ points $t_j$ over the inner 80 % of each side (Sobel gradient of the smoothed image, bilinear). The refined centre is $\mathbf c = \mathbf c_0 + \delta_1^\ast \mathbf n_1 + \delta_2^\ast \mathbf n_2$; size and orientation stay as measured (they feed the shape check). Dry test (21.10., 12 images, camera 425 mm above the base, ceiling light): with the stud near the image centre the correction was 0.2–0.7 mm and the square lies on the visible base edges; where the threshold kept only part of the base (7.3 mm), the refinement still found the whole base (1.8 mm shift). Far off-axis (> 50 mm) the thread, 19 mm higher, projects beyond the base and can attract the lines, which is harmless because the position is only taken after centring.

**Tracking with a temporal median** – **used** since 20.10.2026 (for nut and stud; camera at 30 fps, 10 fps to be tested). Objects are followed from image to image: each object of a new image is assigned to the track whose last centre is nearest, within 40 px (≈ 7.5 mm at $d = 400$ mm), nearest pairs first. A track's centre, width across flats and width ratio are the medians over the last 5 images in which it was found, and only a track with 5 such images counts; a track not found in more than 3 images in a row ends. The median ignores single outliers, and specks that flicker in and out of the segmentation never reach 5 images (the stud search had locked on such specks in 2 of 3 runs). `sa_tasks` sends each request (target, distance) only after the settle time, and the request starts the tracks over, so no image taken during the motion enters a median. Only the first view of the stud search, at the observation pose where the candidate is chosen, uses the lock; after the centring move and for the nut, the medians of all tracks are taken once 5 images have been tracked, without confirmation or lock (this had worked well; with the lock also after centring the stud was lost once, 20.10.). With the lock: after 10 tracked images the result is **locked**: the medians of the confirmed tracks (the stud: ranked by area) stay fixed until the next request, while the live image runs on; `sa_tasks` only uses locked detections. The matching radius is 40 px (≈ 7.5 mm): with 15 px the stud's centre jumped farther than the radius between images, its track kept breaking and the stud was never confirmed (20.10.). A per-object median over the last 5 frames without tracks, and freezing the overlay on the image of each decision, were tried the same day and replaced by this.

*Which plane for the centre (single off-axis view)* – **dropped** 20.10.2026 (the off-axis outline was unusable at the robot). With $D = 400$ mm, $X = 100$ mm, on the base top plane: base bottom edges $[90.3, 99.8]$ mm, base top $[95, 105]$ mm, thread tip $[100.8, 109.2]$ mm; hull centre 99.7 mm, an error of $-0.3$ mm; without the walls about $+2$ mm.

**Several objects in view** (added 19.10.2026, live scene with a lying bolt and two nuts of different size, where the largest blob, the bolt, was taken). With one object the finder stays filter-free. With several, each blob's convex hull gets (1) a **shape filter** on the width ratio $w_\text{min}/w_\text{max}$ over all directions (rotating calipers [Toussaint 1983]): $\sqrt3/2 = 0.866$ for a regular hexagon, about 1 for a round washer, low for an elongated bolt; accepted $[0.60, 0.95]$, because the shadow lowers the ratio of a real nut (bench 0.62–0.82). If exactly one object passes, it is the nut. If several pass, (2) a **size filter** takes the one whose width across flats $s = s_\text{px}\, d / f$ is closest to the target (13 mm for M8), if within ±40 %; the tolerance is wide because the shadow makes $s$ about 30 % too large (2.0 result below). The overlay marks shape rejects red (with the width ratio), nut-shaped objects not chosen orange (with $s$ in mm) and the chosen nut green.

**Result on the bench set** (16.10., no reference image): 26 of 26 grid nuts, position error RMS 1.36 mm, max 2.25 mm (Section 2.8 method; the full pipeline below: 1.40 mm). Comparison of the segmentations on the same set (19.10.): two-sided RMS 1.35 mm, max 2.28 mm, $s$ = 17.1 ± 0.7 mm; inverse binary $r < 0.75$ RMS 1.30 mm, max 2.12 mm, $s$ = 14.2 ± 0.4 mm (0.65: 12.8 mm, 0.85: 15.6 mm, RMS 1.33–1.34 mm); extra images 9 of 11 correct for both. The centre barely changes, the measured size gets much closer to the nominal 13 mm. Empty sheet and nuts cut by the border: nothing found. The width across flats is too large (17 mm instead of 13 mm), because the soft shadow is part of the blob; the centre is not affected. With several objects in view the largest wins. The pipeline of Sections 2.1–2.7 (edges, reference image, size, hole and shape filters, temporal median) was removed from the code on 19.10.2026; it is in the git history (before commit "sa_vision: replace the full pipeline by the simple finder") if parts are needed again.

### 2.0b Nut centre from the hole – **dropped** (tried 19.10.2026; back to 2.0 for simplicity, the circle thresholds did not carry over from the bench to the live camera)
The outline of the nut changes with the light: specular facets drop out of the dark mask, the soft shadow is added to it. The one feature that looks the same in every bench image is the bore together with the nylon ring of the locknut: a black disc with a sharp, round edge, concentric with the nut and independent of its rotation about the axis. Its centre is therefore taken as the nut centre. Measured on the 26 bench nuts (percentiles 5 / 50 / 95 % of $r$): disc 0.28 / 0.35 / 0.58, nut body 0.56 / 0.87 / 1.17, around the nut (shadow and table) 0.73 / 1.05 / 1.13.

Two filters on $r$ separate the parts: **black** $r < 0.4$ (only the disc) and **band** $0.4 \le r < 0.9$ (the grey nut body without the disc). Their union, closed and opened as in 2.0, gives the blobs; the specular facets ($r > 1$) are in neither, as in the inverse binary mask. For each blob (largest first, border blobs excluded), the black pixels inside its convex hull are closed and opened once (5 px disc), and the convex hull of the largest black region is taken. The hull matters with the strict black threshold: a reflection on the bore wall often lifts one side of the bore above 0.4, so only a black crescent remains, open to that side; the outer edge of the nylon ring stays black, and the hull closes the crescent to the disc (without the hull, black < 0.4 passed only 10 of 26 bench nuts). Where the crescent is open, the hull edge is a chord that shifts the centre slightly towards the black side (synthetic worst case 2.4 px; fitting only the hull points on the black edge did not improve the bench result). A circle is fitted to the hull outline, sampled every pixel, by the algebraic least-squares fit [Kåsa 1976]: $u^2 + v^2 = 2au + 2bv + c$ is linear in $(a, b, c)$, the centre is $(a, b)$ and the radius $\rho = \sqrt{c + a^2 + b^2}$.

Two checks reject what is not a nut, without any size assumption:
1. **Clear circle.** The radial deviation $\operatorname{std}(\lVert \mathbf p_i - \mathbf c\rVert) / \rho \le 0.2$, the region fills at least 85 % of the fitted circle, its centre lies within $0.35\,s_\text{px}$ of the hull's hexagon centre (2.0), and its diameter is at most $0.85\,s_\text{px}$ (a solid dark object is not a hole). A lying stud fails here (deviation about 0.4).
2. **Hexagon, not round.** The ratio of the smallest to the largest width of the hull over all directions, $w_\text{min}/w_\text{max}$, is $\sqrt3/2 = 0.866$ for a regular hexagon (across flats / across corners), $1$ for a circle and $1/\sqrt2 = 0.707$ for a square. Accepted: $[0.65, 0.93]$, so a washer (round outline with a dark hole) is rejected. Bench nuts: 0.75–0.86, the shadow lowers the ratio.

The first blob that passes both checks is the nut, so a larger washer or a lying stud next to the nut no longer hides it. The hexagon of 2.0 is drawn concentric to the circle; its size and orientation are only for the overlay.

**Result** (bench set of 16.10., same evaluation as 2.0), black threshold swept: 0.30 → 11 of 26 grid nuts, 0.35 → 20, **0.40 → 26 of 26, RMS 1.36 mm**, 0.45 and 0.50 → 26 of 26 (RMS 1.39 / 1.36 mm); hull centre of 2.0 on the same set: 1.30 mm. Extra images 11 of 11 correct from 0.35 up (2.0: 9 of 11, both lying studs were taken for a nut). The band limits hardly matter: 0.7–0.9 gives the same result as 0.4–0.9 at black 0.4, but leaves the darker quarter of the body (0.48–0.7) in neither filter, so the contiguous band is used. 0.4 is the strictest threshold that keeps all bench nuts. **Limitation:** on the hand-held images of 15.10. (strong side light, nut held at an angle) only 3 of 27 pass at 0.4 and 15 of 27 at 0.5: the bore is often bright there and the long shadow lowers the width ratio below 0.65. The method assumes the diffuse light of the bench and the robot; 0.5 is the more tolerant setting.

### 2.1 Image acquisition and pre-processing – **partly used** (fixed exposure via `camera_controls` and smoothing are used; the region of interest is dropped)
1. **Fixed exposure and white balance** (`software/config/camera.yaml`, set with `v4l2-ctl` before the camera starts). With automatic exposure the brightness depends on the scene content (a bright table drives the gain down), so a reference image of the empty work area and a live image with a nut would not share the same grey levels. Trade-off measured in office light: the automatic mode reaches a table grey value of about 190 at 24 fps; manual exposure needs about 120 ms (4 fps) for the same brightness, 50 ms gives about 110 at 20 fps. The scene is static during the observation, so a low frame rate is acceptable. The exposure must not clip the table: a clipped table (grey value 255) hides the highlights of the nut and makes the division by the reference image meaningless. Bench setup of 16.10.2026 (diffuse lamp, $d \approx 420$ mm): 10 ms gives a table median of 180 without clipping, 15 ms clips 6 % and 25 ms clips 60–94 % of the pixels. Target: table grey value 170–200, no clipping.
2. Greyscale conversion.
3. Gaussian smoothing, $G_\sigma(x,y) = \frac{1}{2\pi\sigma^2} e^{-(x^2+y^2)/2\sigma^2}$, to suppress sensor noise before thresholding.
4. Region of interest = known work area projected into the image, so the gripper and the table edge cannot produce false detections.

### 2.2 Background normalisation and two-sided threshold – **dropped** 19.10.2026 (replaced Otsu, also **dropped**; 2.0 keeps the idea with a blurred background instead of a reference image)
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

### 2.2a Gradient-based foreground – **dropped** 19.10.2026 (code removed with the full pipeline)
The level threshold of 2.2 assumes that the nut differs from the table in brightness. Under the diffuse lamp of the bench setup this is wrong: the flats of the zinc-plated nut reflect the lamp and reach 0.85–1.0 of the table brightness, while the soft shadow beside the nut is the darkest area after the thread hole. The level threshold then segments hole plus shadow and misses the flats, and the silhouette fails the shape filter (4 of 26 nuts found on the bench set, Section 2.8).

The foreground is therefore taken from the **gradient magnitude** of the ratio image,

$$
g(u,v) = \sqrt{(S_x * r)^2 + (S_y * r)^2} > \tau_g \qquad (\tau_g = 0.35)
$$

with the $3 \times 3$ Sobel kernels $S_x, S_y$ (they include a smoothing factor, so a step of height $\Delta r$ gives a peak of about $4\,\Delta r$ before the blur). The physical argument: the outline of the nut, its facet edges, the chamfer and the hole are sharp edges, whereas a shadow cast by an extended (diffuse) light source has a penumbra that is wide compared with the pixel size, so its gradient stays small. Because $g$ is computed on $r = I/B$, the threshold is relative and independent of the illumination level. The edge pixels are then **grouped by proximity**: pixels closer than $k_g\, s_\text{px,max}$ ($k_g = 0.1$, dilation with a disc and connected components) form one candidate, whose silhouette is the convex hull of its edge pixels only (the dilation does not widen it). An outline with a gap, e.g. a flank without contrast against its own shadow, therefore still yields the whole nut; a closing followed by filling the outline (first version) lost such nuts. Bench set: $k_g = 0.05$ → 25/26, 0.1 and 0.2 → 26/26; hand-held images 18/27 instead of 12/27 with closing + filling.

Known bias: the gradient exceeds $\tau_g$ about 1.5 px beyond the true outline on each side (blur and Sobel support), so $s_\text{px}$ is about 3 px too large on synthetic images (0.5 mm at $d = 340$ mm). On the bench set the measured width across flats was $13.30 \pm 0.34$ mm for the nominal 13 mm. The centre is not affected, because the widening is symmetric.

Trade-off: on the 27 hand-held images of 15.10. (no lamp, no reference image, varying height) the edge method finds 18 of 27 nuts against 20 of 27 with the level threshold; there the nut is dark on a bright table and the level threshold works well. The level threshold remains available (`segmentation: threshold`). The edge method is the default because the bench conditions (fixed camera, fixed exposure, reference image, lamp) are those of the robot.

### 2.3 Morphology – **dropped** 19.10.2026 (code removed with the full pipeline)
Closing ($A \bullet B = (A \oplus B) \ominus B$, two iterations, elliptic $7 \times 7$ element) first joins highlights, dark facets and the hole into one region; opening ($A \circ B = (A \ominus B) \oplus B$) then removes small specks. The edge foreground (2.2a) uses grouping instead of morphology.

### 2.4 Silhouette and dark core – **dropped** 19.10.2026 (code removed with the full pipeline)
Outer contours are extracted by border following [Suzuki & Abe 1985]. The **silhouette** of a candidate is the convex hull of its contour; the hull fills concavities where a facet happens to have the brightness of the table. The **dark core** is the largest connected region inside the silhouette with $r < 1 - \tau_c$ ($\tau_c = 0.28$). It contains the thread hole, the nylon ring of a locknut and the shaded inner chamfer.

### 2.5 Centre from image moments – **dropped** 19.10.2026 (code removed with the full pipeline)
For a region $R$, the raw moments are $m_{pq} = \sum_{(x,y)\in R} x^p y^q$, and the centroid is

$$
\bar u = \frac{m_{10}}{m_{00}}, \qquad \bar v = \frac{m_{01}}{m_{00}}
$$

with $m_{00}$ the area in pixels. The reported nut centre is the centroid of the **silhouette**. The centroid of the dark core is not used: its shape depends on how the light falls into the bore, and it lay up to $0.18\,s$ away from the silhouette centroid on the test images (about 2 mm for the large nut).

Known biases, to be measured at the robot (Section 4):
- A facet with the brightness of the table is missing from the silhouette; edges with little contrast are cut more than edges with high contrast. On the test images the lowest facet was often cut, which moves the centre by a few pixels (estimate 0.5–1 mm) away from that side. Synthetic test: a nut with one half just beyond the dark threshold gives about 1 px offset.
- With perspective the silhouette is the union of the top face, the visible side faces and the bottom face. For a nut at lateral distance $X$ from the optical axis the top and bottom face differ by $X h / d$ on the object plane (≈ 1.2 mm at $X = 50$, $h = 8$, $d = 340$ mm), so the silhouette centroid lies roughly half of this outside the top-face centre.

### 2.6 Shape filters (scale-invariant) – **dropped** 19.10.2026 (code removed with the full pipeline)
- **Width across flats and across corners** by rotating calipers [Toussaint 1983] on the silhouette: the minimum width of a convex polygon is attained with one caliper flush with an edge; for a hexagon it is the width across flats $s_\text{px}$. The maximum width (the diameter) is the width across corners $e_\text{px}$.
- **Width ratio** $s/e$: regular hexagon $\sqrt3/2 \approx 0.866$, square $1/\sqrt2 \approx 0.707$, circle 1. Accepted window 0.78–0.95. On the test images, accepted nuts had 0.82–0.91, the parts of the stud (square base, thread) 0.39–0.67. This replaces the compactness filter: compactness $C = 4\pi A / P^2$ (hexagon $\pi/(2\sqrt3) \approx 0.907$, square $\pi/4 \approx 0.785$) did not separate the blurred square stud base ($C = 0.87$) from the nuts ($C = 0.92$–$0.96$); it is still reported for debugging. A round washer is rejected by $s/e \approx 1$.
- **Dark core present and concentric:** the core area divided by the silhouette area must lie in 0.15–0.85, and the core centroid must lie within $0.25\,s_\text{px}$ of the silhouette centroid. The window is deliberately wide, see 2.6a.
- **Uniqueness:** a detection is valid only if exactly one silhouette passes all filters (decision 14.10.). Two candidates in the work area are an error, not a guess.
- **Polygon approximation** for the stud base (Douglas–Peucker, 4 corners, side ratio ≈ 1) – **planned**.

### 2.6a Size-independent nut detection – **dropped** 19.10.2026 (code removed with the full pipeline)
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

### 2.6c Stud detection – **dropped** 19.10.2026 (code removed with the full pipeline)
The stud is fixed in its area and seen from directly above (observation pose over it). The same pre-processing (2.1–2.2a) yields silhouettes; the stud is the one whose width $s$ matches the 10 mm square base ($\pm 20\,\%$, $d$ to the top face of the base) and whose width ratio is that of a square, $s/e \in [0.62, 0.80]$ around $1/\sqrt2 = 0.707$; a nut ($s/e \ge 0.82$) never passes. The threaded rod (8 mm) lies inside the square and does not change the hull when the camera is above the stud. Seen off-axis, the tip of the rod (19 mm higher) is displaced outwards by $X h / d$, which is why the stud observation pose is taught directly above it. Exactly one candidate must pass. Each observation pose has its own reference image (`reference_image`, `reference_image_stud`), because the empty nut area and the stud area look different.

### 2.6b Source of the distance $d$ – **dropped** 19.10.2026 (code removed with the full pipeline)
The metric quantities need the distance $d$ from the camera to the nut top face.
- **`yaml`**: $d$ is a known value, `object_plane_distance_mm`; at the robot it will come from forward kinematics (Section 3). The size $\hat s = s_\text{px} d / f$ is then *measured*.
- **`from_size`** (hand-held tests without the robot, at any height): if the nut size $s^\ast$ is known (`target_across_flats_mm`), the pinhole relation is inverted,
$$
\hat d = \frac{f\, s^\ast}{s_\text{px}}
$$
The size is then *assumed*, not measured: a size filter on $\hat s$ would be circular ($\hat s = s^\ast$ for every candidate). The size filter is therefore relaxed to a plausibility check of the implied distance, $\hat d \in [d_\text{min}, d_\text{max}]$ (`d_from_size_range_mm`, 200–500 mm), i.e. $s_\text{px} \in [f s^\ast (1-t)/d_\text{max},\ f s^\ast (1+t)/d_\text{min}]$. Consequences: a nut of another size is taken for an M8 at another distance (an M12 next to an M8 gives "2 candidates" instead of being rejected), and every error in $s_\text{px}$ goes directly into $\hat d$: $\delta \hat d / \hat d = -\delta s_\text{px} / s_\text{px}$. The bias of about +2.5 px found for $s_\text{px}$ (Section 2.6a) gives about 3 % too small $\hat d$ at 340 mm, and through Section 4 a lateral error of $X \cdot 0.03$ (0.9 mm at $X = 30$ mm). The debug image states "d estimated from size". On the 27 test images the implied distances were 273–390 mm, consistent with the estimate of 15.10.; 19 of 23 M8 nuts passed all filters, 14 images gave a valid single detection (the 5 images with the large nut give "2 candidates").

**Debug image** (`/vision/debug`): the camera frame ($x_\text{cam}$ right, $y_\text{cam}$ down) is drawn as a legend at the top left, the robot frame ($x_\text{robo} = -y_\text{cam}$, $y_\text{robo} = -x_\text{cam}$, gripper pointing down) at the bottom right; the principal point $(c_x, c_y)$ is marked with a cross and joined to the nut centre. At the top right, for a detected nut, it shows $\hat s$, $d$ and the offset of the nut from the optical axis in the robot frame, $(\Delta x_\text{robo}, \Delta y_\text{robo}) = (-Y_C, -X_C)$ with $(X_C, Y_C)$ from Section 3.2, i.e. the distance the robot would have to move to bring the optical axis over the nut.

### 2.7 Temporal robustness – **dropped** 19.10.2026 (code removed with the full pipeline)
Pixels at the rim of the nut lie close to the threshold of Section 2.2, so they flip between foreground and background from frame to frame, even with a still camera (sensor noise, JPEG compression). Each flip at the outer boundary moves the convex hull and with it the centre. Measured on 60 frames of a still camera with an M8 locknut ($s_\text{px} \approx 145$, $d \approx 180$ mm): the single-frame centre jitters with a standard deviation of 0.9 px (range 3.4 px, about 0.08 mm and 0.3 mm on the table).

Spatial remedies were tried and **dropped**: smoothing the binary mask (Gaussian blur and threshold at one half, $\sigma$ up to $0.08\,s_\text{px}$) left the jitter at 0.8–0.9 px and rounds the hexagon corners, which raises $s/e$ towards the circle; a least-squares fit of a regular hexagon to the outline reached 0.6 px. The flipping pixels change the outline itself, which no spatial model can restore.

The published centre is therefore the **median over the last $n$ valid frames** ($n = 5$, `centre_median_frames`), separately for $u$, $v$ and $d$:

$$
\hat u_t = \operatorname{median}(u_{t-n+1}, \dots, u_t)
$$

On the same 60 frames this gives a standard deviation of 0.08 px (range 0.3 px). Unlike the exponential filter of the BA ($\alpha = 0.25$), the median ignores single-frame outliers instead of averaging them in. The result is valid only once the window holds $n$ frames. The window starts over after an invalid frame and when the new centre lies more than $0.25\,s_\text{px}$ from the current median (`centre_reset_jump_ratio`), so a moved camera or nut is never mixed with the old position. Cost: a delay of $n$ frames (0.25 s at 20 fps), irrelevant at the robot, where the camera stands still during the observation.

### 2.8 Bench evaluation on a printed grid – **dropped** 19.10.2026 (code removed with the full pipeline)
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

## 3. From pixel to robot target – **used** (`sa_tasks/frames.py`, `sa_tasks/sequences.py`)

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

**Camera tilt – used since 22.10.2026.** The optical axis need not be square to the housing (the sensor of a low-cost USB camera can sit a degree or two off; the intrinsic calibration does not see this), so ${}^{F}\mathbf{R}_{C} = \mathbf{R}_x(\alpha)\,\mathbf{R}_y(\beta)\,\mathbf{R}_z(+90°)$ with small tilts $\alpha, \beta$ about $x_F, y_F$. With the camera centred over an object, height, focal-length and distortion errors vanish (Section 4: they scale with the distance from the axis), and what remains is the axis direction: a tilt $\varepsilon$ moves the computed point by $d\tan\varepsilon$. Measurement 22.10.2026: camera centred over the stud ($d = 393$ mm, after the robot positioning errors of Section 5 were removed), iris centred over the stud by hand and the TCP read from the robot: the stud lay 8.6 mm further in $+y_B$ than computed, $\alpha = \arctan(8.6/393) = 1.25°$ ($x_F = x_B$ in the fixed pose; positive $\alpha$ turns the axis towards $+y_B$). On 23.10.2026 briefly 1.33° (= 0.5 mm further in $+y_B$), then back to 1.25°. $\beta$ is not measured yet (only $y$ was checked by hand), $\beta = 0$. Unlike a constant target correction, the tilt is part of the frame chain and holds for every pose and height.

**Gripper mount tilt – dropped** 20.10.2026 (code removed: the rotating lower half of the gripper has play and adapts to the table when pressed flat, so the measurement shows the pressed, not the working state; the mount is reprinted instead and the residual measured end to end). A printed adapter can hold the gripper slightly tilted against the flange. The gripper frame $\{G\}$ is then $\{F\}$ rotated by a small ${}^{F}\mathbf{R}_{G}$ about the common origin, and the CAD offsets (camera, TCP) are defined in $\{G\}$. ${}^{F}\mathbf{R}_{G}$ is measured with the bottom plate pressed flat on the table, where $\{G\}$ is level: ${}^{F}\mathbf{R}_{G} = ({}^{B}\mathbf{R}_{F})^\top\,\operatorname{lev}({}^{B}\mathbf{R}_{F})$, with $\operatorname{lev}$ keeping the yaw and setting the tool axis to $-z_B$; several samples are averaged as rotation vectors. With the switch on, every motion commands $\{G\}$ to the fixed orientation, i.e. ${}^{B}\mathbf{R}_{F} = {}^{B}\mathbf{R}_{G}^{d}\,({}^{F}\mathbf{R}_{G})^\top$, so the bottom plate lands flat and the camera on the gripper looks straight down; camera and TCP are computed as ${}^{B}\mathbf{T}_{F}\,{}^{F}\mathbf{T}_{G}\,{}^{G}\mathbf{T}_{C}$. A mount tilt $\varepsilon$ explains a constant target offset of $d \tan\varepsilon$ (Section 4); the hand-guided reference points of 18.10. showed flange tilts of 0.9–3.2° with the plate on the table.

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
  The target comes from the frame chain above (the split was logged until 20.10.2026, then dropped).
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
- **Verification procedure** – the hand-guided truth (`teach_poses truth`, `compare_runs`) was **dropped** 20.10.2026 (code removed). **Used instead:** the end-to-end check of `place_screw --approach-only`: the stud position seen 1) from the observation pose and 2) with the camera centred, 3) the TCP reached above it, and the remaining offset of the bottom plate against the stud measured by hand; vision and robot were repeatable to ±0.2 mm (3 runs, 20.10.), so the offset is constant and goes into $\Delta\mathbf{p}$ (`target_correction_mm`). **Since 22.10.2026 $\Delta\mathbf{p} = 0$** (23.10.2026 briefly (0, −1) mm, back to 0 together with the camera tilt): the hand-tuned values (+4 … +8 mm in $y$) never held on both sides of the table, because they mixed two effects that are now modelled separately: the direction-dependent stopping error of the arm (fine positioning, Section 5) and the tilt of the camera axis (Section 3.1). After both, the approach check gave TCP − stud = (−0.2, +0.3) mm.

---

## 5. Motion – **used** (`sa_tasks/trajectory.py`, `sa_tasks/sequences.py`)

The Cartesian impedance controller tracks a reference pose; the task code streams the reference at 100 Hz (the controller additionally low-pass filters it and limits its speed). Every move is **one continuous minimum-jerk motion** from the measured pose to the goal,

$$
\sigma(s) = 10 s^3 - 15 s^4 + 6 s^5, \qquad s \in [0, 1],
$$

which starts and ends with zero velocity and acceleration (peak speed $1.875\times$ the mean speed). To arrive vertically over the nut without sweeping the gripper over it, the horizontal and vertical components use different time scales: going down, $x, y$ follow $\sigma(t / \lambda T)$ and $z$ follows $\sigma(t / T)$ with $\lambda = 0.6$, so the horizontal motion is finished at $0.6\,T$ and the rest of the descent is vertical (from an observation height of 350 mm, the last $\approx 55$ mm). Going up, $z$ leads instead, so the gripper leaves the table vertically. The orientation is interpolated by spherical linear interpolation with $\sigma(t/T)$. The duration $T$ is the smallest value for which no axis group exceeds the peak speed (100 mm/s, 20 °/s in the first robot runs; 150 mm/s until 21.10.2026, 250 mm/s since, controller limit 300 mm/s). **Since 25.10.2026 $z$ has its own limit of 150 mm/s** (x, y stay at 250 mm/s): with the jitter measure below, vertical moves at 250 mm/s jittered with 0.2–0.9 mm RMS along $z$ (horizontal ones 0.01–0.07 mm), at 150 mm/s only about 0.01 mm. Likely reason: the controller damps the absolute velocity, $\mathbf{F} = -K\mathbf{e} - D\dot{\mathbf{x}}$, so during a move the spring must also overcome $D\dot x$ (about 20 N at 250 mm/s), and in $z$ it carries the uncompensated tool weight as well (4–6 mm sag at 1500 N/m, 6–9 N); together this exceeds the wrench clamp of 18 N, and a clamped command does not damp. At 150 mm/s in $x, y$ the arm instead stuck and jumped (static friction at low speed), so only $z$ is slowed.

- **Full run in one motion (21.10.2026) – used:** the iris opens while the arm moves to the nut observation pose (the gripper command runs in parallel with the move). After the iris has closed on the table, the full run goes straight to the stud observation pose instead of stopping at the nut observation pose first: one move with $z$ leading, so the nut leaves the table vertically and the horizontal travel blends in.

- **Intermediate stop (pick) – dropped (19.10.):** the move ended 30 mm above the table straight over the nut, the task waited until the measured orientation was within 1.2° of the commanded one (at most 5 s) and then descended vertically. It did not help where the misses happen (towards the edge of the work area the arm itself does not hold the pose well) and cost several seconds per pick.
- **Spiral search descent (pick) – used:** one move (xy leading) to $h_s = 10$ mm above the table straight over the computed nut position $\mathbf{c}$ and on without stopping (since 23.10.2026; on 22.10.2026 the arm stopped there for fine positioning, which made it visibly hover before the descent), then an outward Archimedean spiral while the reference descends to $z_\text{table} - \delta_p$:
  $$
  \mathbf{p}(s) = \mathbf{c} + \begin{pmatrix} r\,s\cos(2\pi n s) \\ r\,s\sin(2\pi n s) \\ z_\text{table} + h_s - (h_s + \delta_p)\,s \end{pmatrix}, \qquad s = \sigma(t / T_s),
  $$
  with final radius $r = 6$ mm, $n = 2$ turns ($r/n = 3$ mm between turns, well below the nut size) and $T_s = 2.5$ s (4 s until 21.10.2026). Radius, angle and depth share the minimum-jerk parameter, so the spiral starts and ends at rest. If the target is off, the bottom plate first lands on the nut (the impedance controller lets it rest there with a force $k_z \Delta z$) and slides over it until the open iris passes over the nut, which then drops in; once inside, the iris carries the nut along. The search covers a target error of about $r$ plus the capture range of the open iris. The ending pose is $r$ from $\mathbf{c}$; the landed TCP and the spiral end are recorded.
- **Jitter measure (`motion_test`) – used since 25.10.2026:** to compare controller settings without a full run, the measured end-effector position $\mathbf{p}(t_i)$ is sampled during each move (about 500 Hz) and split into a smooth part, the centred moving average $\bar{\mathbf{p}}_i = \frac{1}{n}\sum_{j=-m}^{m} \mathbf{p}_{i+j}$ over $n = 2m+1$ samples spanning 50 ms, and the residual. The jitter is the RMS of the residual per axis, $J_a = \sqrt{\tfrac{1}{N}\sum_i (p_{a,i} - \bar{p}_{a,i})^2}$: the moving average passes the minimum-jerk motion almost unchanged, so $J_a$ measures what changes faster than about 20 Hz. The lowest `control_command_success_rate` of libfranka (share of valid commands in the last 100 cycles) is logged per move as well. Lost cycles were the main cause of the jitter on 25.10.2026 (0.95–0.99 on large moves; no real-time scheduling allowed for the user, fixed by real-time priority rights, after which only single dips to 0.99 remained).
- **Arrival:** the goal is reached when the arm stands still (flange speed below 3 mm/s for 0.15 s; 0.3 s until 21.10.2026, 2 mm/s for 0.5 s before 19.10.) within 8 mm of the goal (plus the press depth for the pick, where the table stops the arm before the reference). For the observation poses a tight tolerance is not needed: the impedance controller keeps a steady-state error of a few millimetres (3.6 mm in the first robot run, 18.10.), and the target computation uses the measured pose, not the commanded one. The remaining error is logged per axis.
- **Fine positioning (stud approach; pick start only on 22.10.2026) – used since 22.10.2026:** the arm stops short of the goal along its last direction of travel. A reach test (22.10.2026, 10 points around the nut and stud areas, TCP 80 mm above the table) gave 1.5–4.5 mm, always opposite to the preceding motion, with an estimated external force of 2–4 N against it, and all joints at least 14° from their limits. This is static friction against a finite stiffness ($K = 2.5$ N/mm in $x, y$): the arm sticks once $K\,e$ falls below the friction force, and the controller's integral term (anti-windup limit 2.5 N) only lets it creep on slowly (about 40 % less error after 2 s). Earlier full runs had looked like an error towards $y_B = 0$ only because the final moves came from the middle of the table; the sign therefore flipped with the side, which is why a constant target correction worked on one side only. Where the position matters, the task closes the loop around the controller: with the TCP goal $\mathbf{g}$, the measured TCP position $\mathbf{p}_k = T_{BF,k}\,{}^F\mathbf{t}$ (from the measured flange pose, so a tilt of the arm is included) and the flange reference position $\mathbf{r}_k$ ($\mathbf{r}_0$ = the planned flange position),
  $$
  \mathbf{r}_{k+1} = \mathbf{r}_k - \kappa\,P_{xy}\,(\mathbf{p}_k - \mathbf{g}), \qquad \text{until } \lVert P_{xy}(\mathbf{p}_k - \mathbf{g}) \rVert \le 0.5 \text{ mm or } k = 3,
  $$
  each step a short minimum-jerk move from $\mathbf{r}_k$ to $\mathbf{r}_{k+1}$ followed by the arrival wait ($P_{xy}$ keeps the $x, y$ components). This is a discrete integral action with gain $\kappa = 0.8$; it converges in two or three steps (remaining error $(1-\kappa)^k$ of the first one if the friction error stays the same). With $\kappa = 1$ the reach test reached $\le 0.5$ mm at 8 of 10 points, while at 2 points the ongoing creep caused 0.9–1.3 mm of overshoot; $\kappa < 1$ leaves room for that creep. The TCP rather than the flange is corrected because the arm also keeps an orientation error (rotational stiffness 300 Nm/rad, effectively 150): on 22.10.2026 the flange stood within 0.1 mm of its goal while the TCP, 119 mm below, was 2.7 mm off. $z$ is left out: the pick presses on the table and the place searches the contact by force. The following spiral or descent starts from the corrected reference.
- **Approach from a fixed direction (step 2) – used since 25.10.2026:** the gripper's bottom part hangs on a turning disk with some play, and the plate also looks slightly tilted. After a move the plate stays displaced towards the direction of travel, and neither the camera (it sees the stud, not the plate) nor the kinematics (it sees the flange) measures that. The direction of the last move depended on where the stud lay relative to the observation pose, so the plate ended up 1–2.5 mm off to one side or the other, and a constant target correction could only fit one side. The approach therefore first goes to a point offset by $\Delta = (0, -10)$ mm in the base frame from the approach pose, and the last short move (and the fine positioning after it) always runs in $-\Delta$. The play then always ends up on the same side, so one constant correction fits everywhere. The side is $-y$ because the plate, seen in the start configuration, hangs lower on its $-y$ side (direction chosen by Yannick from that tilt) (the same idea as approaching from one side to take up backlash in a machine tool).
- **Fine positioning without integral action – used since 25.10.2026:** the controller resets its integral term on every reference message. While the arm stood still after the fine positioning, no messages came, the integral built up (up to 2.5 N in $x, y$) and helped hold the TCP on the target; the descent streams references at 100 Hz, which wiped it, and the arm gave way by up to $2.5\ \text{N} / 2500\ \text{N/m} = 1$ mm against the last direction of travel, so the nut sat off-centre and was tilted by the thread. During the fine positioning the reference is therefore re-sent with every arrival poll (50 ms), so the arm stands on the spring alone, exactly as during the following descent, and the correction $\kappa\,(\mathbf{p}_k - \mathbf{g})$ is measured in that state.
- **Position-dependent stud correction – dropped (25.10.2026):** an empirical correction linear in the stud's $y_B$ was added the same day for the plate being right on one side of the table and off on the other, and removed again in favour of the fine positioning without integral action (above), which addresses the cause. A swinging-plate model with a tilt gain was tried and dropped before that, because the flange tilt varied from run to run at the same place.
- **Target correction linear in $y_B$ – used since 25.10.2026 (empirical):** after the fixed approach direction and the fine positioning without integral action, the plate was still off in $+y$, by more with the stud far to one side of the table than to the other, so a constant correction was too much on one side and too little on the other. The single correction of every target is therefore linear in its $y_B$ between two calibration points and constant beyond them, $\Delta\mathbf{p}(y) = \mathbf{c}_1 + \frac{y - y_1}{y_2 - y_1}(\mathbf{c}_2 - \mathbf{c}_1)$ (`frames.target_correction`); the nut, at $y_B > 0$, gets $\mathbf{c}$ of the nearer point. The cause of the residual is not known (candidates: the unmeasured second component of the camera tilt, the play of the turning disk).
- **Contact search (step 2):** from the approach pose the reference moves straight down at 5 mm/s; contact is declared when the estimated external force along $z_B$ differs by more than 4 N from its median before the descent (the difference removes the bias of an unmodelled tool mass). The reference is then set to the measured pose. If no contact is found within 17 mm (12 mm until 21.10.2026), the nut is screwed at the lowest point anyway (`screw_without_contact`, 21.10.2026: a nut slid onto the thread without reaching 4 N).
- **Screwing (step 2) – used since 23.10.2026:** at contact the controller switches to a dedicated *screw* preset (`control_mode` 4): translational stiffness $k_{xy}$ = 2000 N/m (first 30, then 400 N/m: the flange moved only 0.2–0.5 mm on a 4 mm reference circle, friction decided where it stood; now the arm goes where the reference is and the loose bottom part of the gripper provides the lateral compliance), $k_z$ = 150 N/m, tilt stiffness 4 Nm/rad about $x$ and $y$, and **no integral action** in this mode. Reason: the thread can only exert a small aligning moment on the nut. With the preload $F$ acting at roughly the thread radius $r \approx 6.5$ mm the moment is $M \approx F r \approx 7.5\ \text{N} \cdot 0.0065\ \text{m} \approx 0.05$ Nm, while tilting the nut by $\theta$ against the previous compliant preset needs $M = k_{\text{eff}}\,\theta$ with $k_{\text{eff}} \approx K/2$ (the orientation error is the quaternion vector part $\sin(\theta/2)$): $20\ \text{Nm/rad} \cdot 0.035\ \text{rad} \approx 0.7$ Nm for 2°, plus up to 1.5 Nm from the integrator. The nut was therefore held tilted on the thread tip and jammed. With $K$ = 4 Nm/rad the same 2° need about 0.07 Nm, of the order of what the thread provides; the price is that any unmodelled tool load (gripper mass not entered as payload) now tilts the arm by itself. The reference is set $d$ below the contact pose, giving a preload $F \approx k_z d + F_0$, where $F_0$ is the part of the tool weight the controller does not compensate (there is no integral action in $z$; in the stiff preset it shows as about 3.5 mm steady-state sag at 1500 N/m, i.e. a few newtons). Because $F_0$ is not known well, the record also holds the measured contact force $F_z - F_{z,\text{free}}$ (estimated external force minus its value in the air just before the descent) at the start, after the catch and at the end. **Sign (checked 25.10.2026 by pressing on the gripper by hand: $F_z$ rises):** a downward push on the tool reads positive, so the nut pressing on the stud (the stud pushes the tool up) reads **negative**; positive values in the records of 23.10.2026 and the morning of 25.10.2026 were offsets of the estimate, not preload, and the estimate is only good to about a newton. **Thread-catch detection:** the nut is first turned slowly in short timed steps (250 ms at speed 100 of 255, $d$ = 3 mm, $k_z d \approx 0.5$ N). After each step the TCP height $z_k$ (from the measured flange pose) is compared with the height $z_0$ before the first step; once $z_0 - z_k \ge 0.6$ mm (about half a turn of the M8 thread, pitch $P$ = 1.25 mm, since a caught nut advances $P$ per turn) the thread counts as caught. Then the nut is tightened faster with more preload (5.0 s at full speed 255, 3.0 s before; speed 200 for 3–5 s turned too weakly, $d$ = 15 mm, $k_z d \approx 2.3$ N; first tried 23.10.2026 with 10 / 50 mm, which pressed visibly too hard). If the drop is not reached within 12 steps (3 s) it is tightened anyway and the run record notes `caught: false`; the record also holds the drop after every step and the flange tilt at the start, after the catch and at the end. The rotation itself stays time-based (no sensor on the gripper); stopping a turn early is not possible without releasing the iris, hence the short steps. History: until 22.10.2026 the nut was turned at full speed for 4 s in the compliant preset, the last 1.5 s with $d$ = 50 mm; it always ended up resting on the stud tip, while a bench test without the robot (stud held by hand under the nut) screwed it on easily even with the weaker N20 1:100 motor, which first suggested that the drive torque is sufficient. On the morning of 23.10.2026 two fixed timed phases (2 s at speed 100, 3 s at 200) still jammed with the nut visibly tilted. **Wobble:** during the catch steps the reference moves one point further on a circle of radius $r_w$ = 2 mm (1 mm before; 2 and 4 mm with $k_{xy}$ = 400 N/m) around the contact position after each step (4 points per circle). The controller's reference low-pass (time constant about 0.5 s) smooths the jumps; because a 250 ms step is only half a time constant, the filtered reference covers only about 40 % of each jump, so the effective lateral pull is well below $k_{xy} r_w$ = 4 N; so the nut is rocked over the tip until it drops into the start of the thread. **Result 23.10.2026 (20 runs):** positioning on the stud was practically exact, the nut caught the thread without the stud tilting and the parts swung only slightly, yet the nut stopped after at most about 1.5 turns (TCP drop 0–1.9 mm at the end; the height-based catch detection never triggered during the slow steps, at most 0.35 mm). The conclusion is that the remaining limit is the drive torque at the nut: the M4 drive is geared up (gear on the motor 44 teeth → pinion on the rotating body 17 teeth, ratio $i = 17/44 \approx 0.39$), so the body turns $1/i \approx 2.6\times$ faster than the motor but gets only $M_\text{nut} = i\,\eta\,M_\text{motor} \approx 0.39\,\eta\,M_\text{motor}$. The hand-held bench test did not show this because a hand-held stud centres and aligns itself and needs much less torque than a fixed stud with the friction of the preload and of a slightly tilted nut. **25.10.2026, first runs with the 1:1 drive:** the preload was too small for the nut to catch (records of the morning: $F_z - F_{z,\text{free}}$ between +1.7 and −0.7 N, i.e. at most 0.7 N of preload), so $d$ was raised to 8 mm for the catch ($k_z d \approx 1.2$ N) and 30 mm for tightening (≈ 4.5 N). The next run measured only about 1 N ($-1.0$, $-0.3$, $-1.0$ N) although the nut advanced 2.0 mm (about 1.6 turns, the best so far); to see whether the force scales with $d$ at all, tightening was then set to 60 mm (≈ 9 N on paper). **Planned:** a redesign with a 1:1 gear pair ($M_\text{nut} = \eta\,M_\text{motor}$, about 2.6× the torque). After the catch the reference returns to the contact position for tightening, and the controller switches to a *tighten* preset (`control_mode` 5): as *screw*, but $k_{xy}$ = 400 N/m, so the nut can swing on the thread while it turns, since nut, iris and stud are not perfectly concentric (23.10.2026: with the stud centred by eye the nut still jammed with $k_{xy}$ = 2000 N/m). Tried and dropped on 23.10.2026: moving back with the nut gripped (blocked by the nut on the stud: −1 mm commanded, −0.36 mm reached), and a regrip (iris open, arm back over the contact, iris closed; even after a short fast engage turn the nut did not hold reliably). **Planned:** a visual check that the gripper's bottom part really turns: while screwing, the bottom plate covers the whole camera image (observed 23.10.2026), so a turning plate shows as image change, a jammed one as a still image. **Release:** before the iris opens, the reference is set to the measured pose raised by 3 mm, the task waits 2.5 s in the soft preset until the controller's low-passed reference has risen from below the thread (time constant about 0.5 s, from 15 mm below: $15 e^{-5} \approx 0.1$ mm), then the stiff preset is switched on and only then the iris opens. Without the wait the stiff preset pulled the arm towards the still lagging reference below the thread. The lift is roughly the steady-state sag of the stiff preset under the uncompensated tool weight (no integral action in $z$), so the arm neither drops nor jumps when the stud stops carrying it. On the first try (23.10.2026) the iris opened first and the soft arm fell towards the reference below the thread.

---

## References (to complete)

- Z. Zhang, "A flexible new technique for camera calibration," *IEEE TPAMI*, 22(11), 2000.
- N. Otsu, "A threshold selection method from gray-level histograms," *IEEE Trans. SMC*, 9(1), 1979.
- S. Suzuki, K. Abe, "Topological structural analysis of digitized binary images by border following," *CVGIP*, 30(1), 1985.
- G. T. Toussaint, "Solving geometric problems with the rotating calipers," *Proc. IEEE MELECON*, 1983.
- I. Kåsa, "A circle fitting procedure and its error analysis," *IEEE Trans. Instrumentation and Measurement*, IM-25(1), 1976.
- D. C. Brown, "Decentering distortion of lenses," *Photogrammetric Engineering*, 32(3), 1966.
- S. Umeyama, "Least-squares estimation of transformation parameters between two point patterns," *IEEE TPAMI*, 13(4), 1991.
- R. Hartley, A. Zisserman, *Multiple View Geometry in Computer Vision*, 2nd ed., Cambridge University Press, 2004.
- F. Chaumette, S. Hutchinson, "Visual servo control, Part I: Basic approaches," *IEEE Robotics & Automation Magazine*, 13(4), 2006.
