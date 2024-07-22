# python imports
import sys
import lcm
import numpy as np
import matplotlib.pyplot as plt

sys.path.append("/home/xuanhien/git/dairlib/bazel-bin/lcmtypes/")

# lcmtype imports
import dairlib
import drake

# drake imports
from pydrake.multibody.plant import AddMultibodyPlantSceneGraph
from pydrake.systems.framework import DiagramBuilder
from pydrake.all import MultibodyPlant, Parser, RigidTransform, FindResourceOrThrow

trifinger_model = "/home/xuanhien/git/dairlib/examples/trifinger/robot_properties_fingers/urdf/edu/trifingeredu.urdf"
cube_model = "/home/xuanhien/git/dairlib/examples/trifinger/robot_properties_fingers/urdf/cube/cube_v2.urdf"

trifinger_default_hw_channels = {
    "TRIFINGER_STATE": dairlib.lcmt_robot_output,
    "CUBE_STATE": dairlib.lcmt_object_state,
    "TRIFINGER_INPUT": dairlib.lcmt_robot_input,
    "OSC_DEBUG_TRIFINGER": dairlib.lcmt_osc_output,
    "FINGERTIPS_DELTA_POSITION": dairlib.lcmt_fingertips_delta_position,
}

trifinger_default_sm_channels = {
    "TRIFINGER_STATE_SIMULATION": dairlib.lcmt_robot_output,
    "CUBE_STATE": dairlib.lcmt_object_state,
    "TRIFINGER_INPUT_SIMULATION": dairlib.lcmt_robot_input,
    "OSC_DEBUG_TRIFINGER": dairlib.lcmt_osc_output,
    "FINGERTIPS_DELTA_POSITION": dairlib.lcmt_fingertips_delta_position,
}


def make_plant_and_context():
    trifinger_plant = MultibodyPlant(0.0)
    trifinger_parser = Parser(trifinger_plant)
    trifinger_parser.AddModels(trifinger_model)
    trifinger_plant.WeldFrames(
        trifinger_plant.world_frame(),
        trifinger_plant.GetFrameByName("base_link"),
        RigidTransform.Identity(),
    )
    trifinger_plant.Finalize()
    return trifinger_plant, trifinger_plant.CreateDefaultContext()
