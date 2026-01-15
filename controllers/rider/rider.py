from controller import Supervisor, Robot
import math

supervisor = Supervisor()
root = supervisor.getSelf()

timestep = int(supervisor.getBasicTimeStep())
# Continuously enforce pose (prevents physics from overriding)
# while supervisor.step(timestep) != -1:
    # for joint_name, angle_deg in joints.items():
        # fields[joint_name].setSFFloat(math.radians(angle_deg))