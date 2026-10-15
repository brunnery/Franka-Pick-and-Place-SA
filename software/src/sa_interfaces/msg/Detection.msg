# Result of one object detection in one camera image.
# header.stamp = image stamp, header.frame_id = camera optical frame.

std_msgs/Header header

bool valid              # true if exactly one object passed all filters
string status           # "ok" or the reason why the detection is invalid

# Object centre in pixels (column, row), as measured in the raw image
float64 u
float64 v
# Same point after removing lens distortion (pixel coordinates of the ideal pinhole camera)
float64 u_undistorted
float64 v_undistorted

# Intersection of the viewing ray with the object plane, in the camera frame [m]
geometry_msgs/Point point_camera
float64 plane_distance  # distance camera -> object plane along z_C used for point_camera [m]

# Measured size of the object
float64 across_flats_mm   # width across flats s = s_px * d / f [mm]
string size               # thread size (e.g. "M8") if s matches it unambiguously, else ""

# Debug information
float64 area_px           # area of the silhouette (convex hull) [px^2]
float64 hole_area_px      # area of the dark core (thread hole) [px^2]
float64 width_ratio       # s / e, width across flats / across corners (hexagon 0.866)
float64 compactness       # 4*pi*A/P^2 of the silhouette
int32 num_candidates      # silhouettes that passed all filters
