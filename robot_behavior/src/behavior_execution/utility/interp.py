import math
from scipy import interpolate
from geometry_msgs.msg import Pose, Quaternion, Vector3
import tf

def cartesian_distance(pose1,pose2):
    pos1 = pose1.position
    pos2 = pose2.position
    return math.sqrt(math.pow(pos1.x-pos2.x,2)+math.pow(pos1.y-pos2.y,2)+math.pow(pos1.z-pos2.z,2))

# def midpoint(pose1,pose2):
#     pos1 = pose1.position
#     pos2 = pose2.position
#     ori1 = pose1.orientation
#     ori2 = pose2.orientation
#
#     posm = Vector3(x=(pos1.x+pos2.x)/2,y=(pos1.y+pos2.y)/2,z=(pos1.z+pos2.z)/2)
#     ori = tf.transformations.quaternion_slerp((ori1.x,ori1.y,ori1.z,ori1.w),(ori2.x,ori2.y,ori2.z,ori2.w), 0.5)
#     orim = Quaternion(x=ori[0],y=ori[1],z=ori[2],w=ori[3])
#     return Pose(position=posm,orientation=orim)

def interp3d(pose1,pose2,max_dist,time_scale):
    dist = cartesian_distance(pose1,pose2)
    steps = int(dist/max_dist)
    return interp3d_bound(pose1,pose2,steps),dist*time_scale

def interp3d_bound(pose1,pose2,steps):
    steps = int(steps+0.5)
    times = list(range(0,steps))
    ori1 = pose1.orientation
    ori2 = pose2.orientation
    plan = []
    xs = interpolate.interp1d([0,steps],[pose1.position.x,pose2.position.x])(times)
    ys = interpolate.interp1d([0,steps],[pose1.position.y,pose2.position.y])(times)
    zs = interpolate.interp1d([0,steps],[pose1.position.z,pose2.position.z])(times)
    for i, time in enumerate(times):
        position = Vector3(x=xs[i],y=ys[i],z=zs[i])
        ori = tf.transformations.quaternion_slerp((ori1.x,ori1.y,ori1.z,ori1.w),(ori2.x,ori2.y,ori2.z,ori2.w), float(i)/len(times))
        orim = Quaternion(x=ori[0],y=ori[1],z=ori[2],w=ori[3])
        plan.append(Pose(position=position,orientation=orim))
    plan.append(pose2) #fill in final gap from last step if any
    return plan


def interp1d(val1,val2,max_dist,time_scale):
    dist = math.fabs(val1-val2)
    steps = int(dist/max_dist)
    return interp1d_bound(val1,val2,steps), dist*time_scale

def interp1d_bound(val1,val2,steps):
    steps = int(steps+0.5)
    times = list(range(0,steps))
    plan = list(interpolate.interp1d([0,steps],[val1,val2])(times))
    plan.append(val2) #fill in final gap from last step if any
    return plan
