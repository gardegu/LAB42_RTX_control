std_msgs/Header header

int32 width              # disparity width
int32 height             # disparity height

int32 min_disp            # minimum disparity
int32 max_disp            # maximum disparity

int32   step             # full row length in bytes
uint16[] data            # actual data 
