# One object that may be the stud (Detection.candidates, ranked: largest first).

float64 u_undistorted   # centre of the fitted polygon, undistorted pixel coordinates
float64 v_undistorted
float64 size_mm         # width across flats of the fitted polygon, s = s_px * d / f [mm]
float64 width_ratio     # smallest / largest width of the convex hull
bool shape_ok           # width ratio and size within the strict band of the target
