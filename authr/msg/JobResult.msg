# Response
string id
bool error
string msg

string agent

string type
# tof_pose
# tof_gripper
# set_pose
# set_gripper
# validate_pose
# validate_gripper

string result
# {
#   // if tof_pose or tof_gripper
#   tof: number
#
#   // if validate_pose or validate_gripper
#   valid: boolean
#
#   // if set_pose or set_gripper
#   //    note: if nonblocking = true then defaults to true regardless
#   success: boolean
# }
