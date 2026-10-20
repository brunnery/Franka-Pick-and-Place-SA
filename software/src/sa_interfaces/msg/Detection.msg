# Result of one object detection. Objects are tracked over the images: only objects found
# in frames_window images count, with the median of those positions, sizes and width ratios.
# header.stamp = stamp of the newest image, header.frame_id = camera optical frame.

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

# All objects in view (not only the chosen one), ranked: most likely target first.
# Used by sa_tasks for the stud search (centre the camera over each until one is a stud).
Candidate[] candidates

# Images since the tracks were started over (each request of sa_tasks sets target and
# distance, which starts them over) and images a track needs to count.
int32 frames_since_reset
int32 frames_window
# Locked: the result is fixed (medians of the confirmed tracks after lock_after_frames
# images) until the next request. sa_tasks only uses locked detections.
bool locked
