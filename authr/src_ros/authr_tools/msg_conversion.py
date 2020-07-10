
import tf
from geometry_msgs.msg import Pose, Quaternion, Vector3
from authr.msg import EulerPose

#===============================================================================
#                                 Position Conversion
#===============================================================================

def position_msgFromDict(dct):
    return Vector3(x=dct['x'],y=dct['y'],z=dct['z'])

def position_dictFromMsg(msg):
    return {'x':msg.x,'y':msg.y,'z':msg.z}

#===============================================================================
#                                Orientation Conversion
#===============================================================================

def orientation_eulerMsgFromQuaterionMsg(qmsg):
    (r,p,y) = tf.transformations.euler_from_quaternion([qmsg.x,qmsg.y,qmsg.z,qmsg.w])
    return Vector3(x=r,y=p,z=y)

def orientation_quaterionMsgFromEulerMsg(emsg, form='sxyz'):
    (x,y,z,w) = tf.transformations.quaternion_from_euler(emsg.x,emsg.y,emsg.z,form)
    return Quaternion(x=x,y=y,z=z,w=w)

def orientation_eulerMsgFromEulerDict(dct):
    return Vector3(x=dct['x'],y=dct['y'],z=dct['z'])

def orientation_quaterionMsgFromQuaternionDict(dct):
    return Quaternion(x=dct['x'],y=dct['y'],z=dct['z'],w=dct['w'])

def orientation_eulerDictFromEulerMsg(msg):
    return {'x':msg.x, 'y':msg.y, 'z':msg.z}

def orientation_eulerDictFromQuaternionMsg(qmsg):
    (r,p,y) = tf.transformations.euler_from_quaternion([qmsg.x,qmsg.y,qmsg.z,qmsg.w])
    return {'x':r, 'y':p, 'z':y}

def orientation_quaternionDictFromEulerMsg(emsg, form='sxyz'):
    (x,y,z,w) = tf.transformations.quaternion_from_euler(emsg.x,emsg.y,emsg.z,form)
    return {'x':x, 'y':y, 'z':z, 'w':w}

def orientation_eulerMsgFromQuaternionDict(dct):
    (r,p,y) = tf.transformations.euler_from_quaternion([dct['x'],dct['y'],dct['z'],dct['w']])
    return Vector3(x=r,y=p,z=y)

def orientation_quaternionMsgFromEulerDict(dct, form='sxyz'):
    (x,y,z,w) = tf.transformations.quaternion_from_euler(dct['x'],dct['y'],dct['z'],form)
    return Quaternion(x=x,y=y,z=z,w=w)

def orientation_quaternionDictFromQuaternionMsg(msg):
    return {'x':msg.x, 'y':msg.y, 'z':msg.z, 'w':msg.w}

def orientation_quaternionDictFromEulerDict(dct, form='sxyz'):
    (x,y,z,w) = tf.transformations.quaternion_from_euler(dct['x'],dct['y'],dct['z'],form)
    return {'x':x, 'y':y, 'z':z, 'w':w}

def orientation_eulerDictFromQuaternionDict(dct):
    (r,p,y) = tf.transformations.euler_from_quaternion([dct['x'],dct['y'],dct['z'],dct['w']])
    return {'x':r, 'y':p, 'z':y}

#===============================================================================
#                                Pose Conversion
#===============================================================================

def pose_eulerMsgFromQuaternionMsg(qmsg):
    return EulerPose(position=qmsg.position,
                     orientation=orientation_eulerMsgFromQuaterionMsg(qmsg.orientation))

def pose_eulerMsgFromEulerDict(dct):
    return EulerPose(position=position_msgFromDict(dct['position']),
                     orientation=orientation_eulerMsgFromEulerDict(dct['orientation']))

def pose_quaternionMsgFromEulerMsg(emsg, form='sxyz'):
    return Pose(position=emsg.position,
                orientation=orientation_quaterionMsgFromEulerMsg(emsg.orientation,form))

def pose_quaternionMsgFromQuaternionDict(dct):
    return Pose(position=position_msgFromDict(dct['position']),
                orientation=orientation_quaterionMsgFromQuaternionDict(dct['orientation']))

def pose_eulerDictFromEulerMsg(msg):
    return {
        'position': position_dictFromMsg(msg.position),
        'orientation': orientation_eulerDictFromEulerMsg(msg.orientation)
    }

def pose_quaternionDictFromQuaterionMsg(msg):
    return {
        'position': position_dictFromMsg(msg.position),
        'orientation': orientation_quaternionDictFromQuaternionMsg(msg.orientation)
    }

def pose_eulerDictFromQuaternionDict(dct):
    return {
        'position': dct['position'],
        'orientation': orientation_eulerDictFromQuaternionDict(dct['orientation'])
    }

def pose_quaternionDictFromEulerDict(dct, form='sxyz'):
    return {
        'position': dct['position'],
        'orientation': orientation_quaternionDictFromEulerDict(dct['orientation'],form)
    }

def pose_eulerDictFromQuaternionMsg(msg):
    return {
        'position': position_dictFromMsg(msg.position),
        'orientation': orientation_eulerDictFromQuaternionMsg(msg.orientation)
    }

def pose_quaternionDictFromEulerMsg(msg, form='sxyz'):
    return {
        'position': position_dictFromMsg(msg.position),
        'orientation': orientation_quaternionDictFromEulerMsg(msg.orientation,form)
    }

def pose_eulerMsgFromQuaternionDict(dct):
    EulerPose(position=position_msgFromDict(dct['position']),
              orientation=orientation_eulerMsgFromQuaternionDict(dct['orientation']))

def pose_quaternionMsgFromEulerDict(dct, form='sxyz'):
    return Pose(position=position_msgFromDict(dct['position']),
                orientation=orientation_quaternionMsgFromEulerDict(dct['orientation'],form))
