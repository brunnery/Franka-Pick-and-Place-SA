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

# Debug information
float64 area_px           # area of the outer contour [px^2]
float64 hole_area_px      # area of the inner hole [px^2]
float64 expected_area_px  # expected outer area from geometry [px^2]
float64 compactness       # 4*pi*A/P^2 of the outer contour
int32 num_candidates      # contours that passed all filters
