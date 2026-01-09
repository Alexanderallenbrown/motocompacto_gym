from controller import Supervisor, Robot
import math

supervisor = Supervisor()
root = supervisor.getSelf()

# Get all joint fields once
joints = {
    "headAngle": 0,                # Head upright
    "rightArmAngle": -20,          # Arms forward/down
    "leftArmAngle": -20,
    "rightLowerArmAngle": 10,      # Elbows slightly bent
    "leftLowerArmAngle": 10,
    "rightLegAngle": -80,          # Legs bent for bike seat
    "leftLegAngle": -80,
    "rightLowerLegAngle": 80,      # Knees bent
    "leftLowerLegAngle": 80,
    "rightFootAngle": -10,         # Feet forward
    "leftFootAngle": -10,
    "rightHandAngle": 0,
    "leftHandAngle": 0,
}

fields = {}
for joint_name, angle_deg in joints.items():
    field = root.getField(joint_name)
    field.setSFFloat(math.radians(angle_deg))
    fields[joint_name] = field

print("Rider pose set to stable riding position!")

timestep = int(supervisor.getBasicTimeStep())
# Continuously enforce pose (prevents physics from overriding)
# while supervisor.step(timestep) != -1:
    # for joint_name, angle_deg in joints.items():
        # fields[joint_name].setSFFloat(math.radians(angle_deg))