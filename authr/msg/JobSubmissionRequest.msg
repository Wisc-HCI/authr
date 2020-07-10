# Request
string id     # set to '' if not used, used to update a pending job
string agent

string type
# tof_pose
# tof_gripper
# set_pose
# set_gripper
# validate_pose
# validate_gripper
# cancel  #Note must have id

string params
# {
#   // if tof_pose
#   pose_start: stringified(EulerPose)
#   pose_end: stringified(EulerPose)
#   retries: number [optional]
#
#   // if tof_gripper
#   effort_start: number
#   effort_end: number
#   retries: number [optional]
#
#   // if validate_pose
#   pose: stringified(EulerPose)
#
#   // if validate_gripper
#   effort: number
#
#   // if set_pose
#   nonblocking: boolean
#   pose: stringified(EulerPose)
#
#   // if set_gripper
#   nonblocking: boolean
#   effort: number
# }
