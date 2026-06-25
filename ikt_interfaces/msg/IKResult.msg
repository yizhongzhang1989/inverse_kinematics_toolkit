# Result of an IK solve. Advisory only.

bool reachable               # true if all active pose tasks met their tolerance
string reason                # ok | joint_limit | singular | task_conflict | max_iters | tf_unavailable
int32 iters
string[] joint_names         # order of `q`
float64[] q                  # solved joint angles (rad)
float64 max_pos_err          # metres
float64 max_ori_err          # rad
string[] blocking_joints     # joints at a limit (when reason == joint_limit)
float64 manipulability       # sqrt(det(J J^T)) of the (first) task
float64 sigma_min            # smallest singular value (singularity proximity)
float64 delta_norm           # ||q - seed|| (continuity / motion size)
