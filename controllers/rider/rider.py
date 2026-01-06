from controller import Supervisor
import math

supervisor = Supervisor()
root = supervisor.getSelf()

joints = {
    "headAngle": 10,
    "rightArmAngle": -90,
    "leftArmAngle": -90,
    "rightLowerArmAngle": 2,
    "leftLowerArmAngle": 2,
    "rightLegAngle": -90,
    "leftLegAngle": -90,
    "rightLowerLegAngle": 90,
    "leftLowerLegAngle": 90,
    "rightFootAngle": 0,
    "leftFootAngle": 0,
    "rightHandAngle": 0,
    "leftHandAngle": 0,
}

for joint_name, angle_deg in joints.items():
    field = root.getField(joint_name)
    field.setSFFloat(math.radians(angle_deg))

print("Rider pose set!")

while supervisor.step(int(supervisor.getBasicTimeStep())) != -1:
    pass
