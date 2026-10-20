# Result of the object detection in one camera image (sa_vision/vision_node). Objects are
# tracked over the images, positions are medians. header.stamp = image stamp.

std_msgs/Header header

bool valid                # the target was found (with lock: confirmed)
string status             # what was found, or why nothing
float64 u_undistorted     # centre of the target, undistorted pixel (nan if not valid)
float64 v_undistorted
Candidate[] candidates    # stud: objects that may be the stud, most likely first

# Images since the last request of sa_tasks (it sets target, distance and lock, which starts
# the tracks over), images a track needs to count, and whether the result is locked (fixed
# until the next request; only with lock).
int32 frames_since_reset
int32 frames_window
bool locked
