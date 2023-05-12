std_msgs/Header header

int32 width      # disparity width
int32 height     # disparity height

int32 num_objects # number of objects
ObjectPos[]      obj_pos             # objects' pos (bounding box) array

DriveSpace       drv_space           # drivable space

int32            fs_boundary_length   # down-sample factor in detection algorithm
int32[]          fs_boundary         # freespace boundary
