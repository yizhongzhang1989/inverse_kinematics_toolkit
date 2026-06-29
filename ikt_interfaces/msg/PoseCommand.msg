# Unified command for ikt_pose_commander: place `control_link` at `pose`,
# with `pose` expressed in `frame_link`.
#
# All three fields are OPTIONAL. Any field left empty/unset reuses the value
# from the previous command. If a field is NEVER set, the commander default is:
#   frame_link   -> the first link of the model (the base / root link)
#   control_link -> the last link of the model (the tip link)
# so a bare pose still controls the tip in base coordinates.
std_msgs/Header header        # stamp only; header.frame_id is ignored (use frame_link)
string frame_link             # frame the pose is defined in ("" = reuse previous)
string control_link           # link to control ("" = reuse previous)
bool has_pose                 # true: `pose` is set this message; false: reuse previous
geometry_msgs/Pose pose       # target position + orientation of control_link in frame_link
