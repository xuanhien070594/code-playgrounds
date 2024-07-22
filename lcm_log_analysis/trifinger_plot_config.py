import io
from yaml import load, dump

try:
    from yaml import CLoader as Loader, CDumper as Dumper
except ImportError:
    from yaml import Loader, Dumper


class TrifingerPlotConfig:
    def __init__(self, filename):
        data = load(io.open(filename, "r"), Loader=Loader)
        self.is_simulation = data["is_simulation"]
        self.channel_x = (
            data["channel_x_sim"] if self.is_simulation else data["channel_x"]
        )
        self.channel_u = (
            data["channel_u_sim"] if self.is_simulation else data["channel_u"]
        )
        self.channel_osc = data["channel_osc"]
        self.channel_fingertip_delta_pos = data["channel_fingertip_delta_pos"]
        self.channel_cube = data["channel_cube"]
        self.plot_style = data["plot_style"]

        self.start_time = data["start_time"]
        self.duration = data["duration"]
        self.plot_joint_positions = data["plot_joint_positions"]
        self.plot_joint_velocities = data["plot_joint_velocities"]
        self.plot_measured_efforts = data["plot_measured_efforts"]
        self.plot_commanded_efforts = data["plot_commanded_efforts"]
        self.pos_names = (
            data["special_positions_to_plot"]
            if data["special_positions_to_plot"]
            else []
        )
        self.vel_names = (
            data["special_velocities_to_plot"]
            if data["special_velocities_to_plot"]
            else []
        )
        self.act_names = (
            data["special_efforts_to_plot"] if data["special_efforts_to_plot"] else []
        )

        self.fsm_state_names = data["fsm_state_names"]

        self.plot_qp_costs = data["plot_qp_costs"]
        self.plot_qp_solve_time = data["plot_qp_solve_time"]
        self.plot_qp_solutions = data["plot_qp_solutions"]
        self.plot_tracking_costs = data["plot_tracking_costs"]
        self.plot_active_tracking_datas = data["plot_active_tracking_datas"]
        self.tracking_datas_to_plot = (
            data["tracking_datas_to_plot"] if data["tracking_datas_to_plot"] else []
        )
        self.plot_object_state = data["plot_object_state"]
