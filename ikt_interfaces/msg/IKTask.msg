# One Cartesian IK task: place `frame` at `target` with per-DOF `stiffness`.
# The solver is advisory only; a solution is not a command.

string frame                 # any frame in the model (link, tip or tool frame)
geometry_msgs/Pose target    # target pose in the solve/base frame (or frame_id)
float64[6] stiffness         # diag(Wt): [x y z rx ry rz]; 0 lets that DOF float
string frame_id              # optional TF source frame for `target` ("" = base)
