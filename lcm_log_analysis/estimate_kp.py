import numpy as np
import matplotlib
import matplotlib.pyplot as plt
from scipy.signal import savgol_filter
from pydrake.multibody.plant import MultibodyPlant
from pydrake.multibody.parsing import Parser
from pydrake.math import RigidTransform


def main():
    model_url = "/home/xuanhien/git/dairlib/examples/trifinger/robot_properties_fingers/urdf/edu/trifingeredu.urdf"
    plant = MultibodyPlant(time_step=0)
    (trifingeredu,) = Parser(plant).AddModels(model_url)
    plant.WeldFrames(
        plant.world_frame(),
        plant.GetFrameByName("base_link"),
        RigidTransform.Identity(),
    )
    plant.Finalize()
    plant_context = plant.CreateDefaultContext()
    position = np.zeros(9)
    position[:3] = np.array([0, 0.8, -1.3])


vel = np.zeros(9)
vel[:3] = velocities[i]
plant.SetPositions(plant_context, position)
plant.SetVelocities(plant_context, vel)

# compute accel, coriolis, and gravity terms.
gravity_torque = plant.CalcGravityGeneralizedForces(plant_context)[:3]
bias_torque = plant.CalcBiasTerm(plant_context)[:3]
accel_torque = plant.CalcMassMatrix(plant_context)[:3, :3]
