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
- **Ground resolution** $d/f$: 0.096 mm/px at $d = 200$ mm, 0.144 mm/px at 300 mm, 0.193 mm/px at 400 mm. The M6 nut (10 mm across flats) spans about 69 px at 300 mm.
- **Narrow field of view:** at $d = 300$ mm the image covers only about 185 × 104 mm on the table. Each work area must fit into this footprint from its observation pose.
- **Principal point offset:** the optical axis meets the image at $(695, 406)$, i.e. 55 px right of and 46 px below the image centre $(640, 360)$. At $d = 300$ mm this is about 8 mm and 7 mm on the table. Centring an object on the geometric image centre, as in the BA, therefore does not place it on the optical axis; the back-projection in Section 3 uses $(c_x, c_y)$ and avoids this bias.

---

## 2. Object detection – **used** (nut, `sa_vision/detection.py`; stud: planned)

Goal: the centre $(u, v)$ of the nut (pick) and of the stud base (place), from a single observation pose, on a white background, without painting the parts (in the BA they were painted matt black).

### 2.1 Pre-processing
1. Greyscale conversion.
2. Gaussian smoothing, $G_\sigma(x,y) = \frac{1}{2\pi\sigma^2} e^{-(x^2+y^2)/2\sigma^2}$, to suppress sensor noise before thresholding.
3. Region of interest = known work area projected into the image, so the gripper and the table edge cannot produce false detections.

### 2.2 Segmentation: Otsu threshold
The threshold $t^*$ is chosen automatically by maximising the between-class variance of the grey-level histogram [Otsu 1979]:

$$
t^* = \arg\max_t\ \sigma_B^2(t), \qquad \sigma_B^2(t) = \omega_0(t)\,\omega_1(t)\,\big(\mu_0(t) - \mu_1(t)\big)^2
$$

$\omega_{0,1}$: class probabilities, $\mu_{0,1}$: class means. Unlike the fixed threshold of the BA (grey value 80), this adapts to lighting changes. The binary image is inverted, so the darker object is foreground.

### 2.3 Morphology
Opening ($A \circ B = (A \ominus B) \oplus B$) removes small specks; closing ($A \bullet B = (A \oplus B) \ominus B$) fills small gaps from specular highlights on the bare metal.

### 2.4 Contours and the nut hole
Contours are extracted with border following [Suzuki & Abe 1985] in a two-level hierarchy (outer boundaries and holes). The nut is the only object whose outer contour contains a hole of the expected size, because the white table is visible through the thread. This makes the hole a robust feature, and its centre coincides with the nut axis.

### 2.5 Centre from image moments
For a contour region $R$, the raw moments are $m_{pq} = \sum_{(x,y)\in R} x^p y^q$, and the centroid is

$$
\bar u = \frac{m_{10}}{m_{00}}, \qquad \bar v = \frac{m_{01}}{m_{00}}
$$

with $m_{00}$ the area in pixels.

### 2.6 Shape filters
- **Compactness** $C = \dfrac{4\pi A}{P^2}$ ($A$ area, $P$ perimeter). Reference values: circle 1, regular hexagon $\pi / (2\sqrt3) \approx 0.907$, square $\pi/4 \approx 0.785$.
- **Polygon approximation** (Douglas–Peucker, tolerance as a fraction of $P$): 4 corners and side ratio ≈ 1 for the square stud base.
- **Expected size from geometry** instead of hand-tuned pixel windows: an object of area $A_\text{mm}$ at distance $d$ appears with

$$
A_\text{px} \approx A_\text{mm} \left(\frac{f}{d}\right)^2
$$

For the M6 nut (across flats $s = 10$ mm): $A_\text{hex} = \frac{\sqrt3}{2} s^2 \approx 86.6\ \text{mm}^2$. The accepted area window is a tolerance band around this value, so it stays valid when the observation height changes.
- **Hole ratio:** the area ratio hole/outer is expected near $\frac{\pi D^2/4}{A_\text{hex}}$ with the visible hole diameter $D \approx 5$ mm (≈ 0.23). This rejects washers and rings.
- **Concentricity:** the centroids of the outer contour and of the hole must lie within a fraction of $s\,f/d$ of each other.
- **Uniqueness:** a detection is valid only if exactly one contour passes all filters. Two candidates in the work area are treated as an error rather than guessed.
- The reported centre is the hole centroid by default (option: outer centroid). With perspective, the visible hole is the overlap of the top and bottom openings, so its centroid corresponds to roughly half the nut height; for a nut at lateral distance $X$ from the optical axis the top and bottom planes differ by $X h / d$ in the image (≈ 0.8 mm at $X = 50$, $h = 5$, $d = 300$ mm), i.e. the error against the top-face centre is about half of this.

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
- D. C. Brown, "Decentering distortion of lenses," *Photogrammetric Engineering*, 32(3), 1966.
- R. Hartley, A. Zisserman, *Multiple View Geometry in Computer Vision*, 2nd ed., Cambridge University Press, 2004.
- F. Chaumette, S. Hutchinson, "Visual servo control, Part I: Basic approaches," *IEEE Robotics & Automation Magazine*, 13(4), 2006.
