import sys
import lcm
import matplotlib.pyplot as plt
import numpy as np

import mbp_plotting_utils as mbp_plots
import trifinger_plotting_utils as plot_utils
import plot_styler

from trifinger_plot_config import TrifingerPlotConfig
from process_lcm_log import get_log_data, print_log_summary

from matplotlib.patches import Patch


def main():
    config_file = "trifinger_plot_config.yml"
    plot_config = TrifingerPlotConfig(config_file)

    channel_x = plot_config.channel_x
    channel_u = plot_config.channel_u
    channel_osc = plot_config.channel_osc
    channel_fingertip_delta_pos = plot_config.channel_fingertip_delta_pos
    channel_cube = plot_config.channel_cube

    if plot_config.plot_style == "paper":
        plot_styler.PlotStyler.set_paper_styling()
    elif plot_config.plot_style == "compact":
        plot_styler.PlotStyler.set_compact_styling()

    """ Get the plant """
    trifinger_plant, trifinger_context = plot_utils.make_plant_and_context()

    # pos_map, vel_map, act_map = mbp_plots.make_name_to_mbp_maps(trifinger_plant)
    # pos_names, vel_names, act_names = mbp_plots.make_mbp_name_vectors(trifinger_plant)

    """ Read the log """
    filename = sys.argv[1]
    log = lcm.EventLog(filename, "r")
    print_log_summary(filename, log)
    if plot_config.is_simulation:
        default_channels = plot_utils.trifinger_default_sm_channels
    else:
        default_channels = plot_utils.trifinger_default_hw_channels

    robot_output, robot_input, osc_debug = get_log_data(
        log,  # log
        default_channels,  # lcm channels
        plot_config.start_time,
        plot_config.duration,
        mbp_plots.load_default_channels,  # processing callback
        trifinger_plant,
        channel_x,
        channel_u,
        channel_osc,
    )  # processing callback arguments

    print("Finished processing log - making plots")
    # Define x time slice
    # t_x_slice = slice(robot_output["t"].size)
    t_osc_slice = slice(osc_debug["t_osc"].size)

    # print('Average OSC frequency: ', 1 / np.mean(np.diff(osc_debug['t_osc'])))

    # revert back next time
    # (
    #     trifinger_joint_position_limit_range,
    #     trifinger_joint_velocity_limit_range,
    #     trifinger_joint_actuator_limit_range,
    # ) = mbp_plots.generate_joint_limits(trifinger_plant)

    # Plot joint positions
    if plot_config.plot_joint_positions:
        plot = mbp_plots.plot_joint_positions(robot_output, pos_names, 0, t_x_slice)
        plt.ylim(trifinger_joint_position_limit_range)
    # Plot specific positions
    if plot_config.pos_names:
        plot = mbp_plots.plot_positions_by_name(
            robot_output, plot_config.pos_names, t_x_slice, pos_map
        )

    # Plot all joint velocities
    if plot_config.plot_joint_velocities:
        plot = mbp_plots.plot_joint_velocities(robot_output, vel_names, 0, t_x_slice)
        plt.ylim(trifinger_joint_velocity_limit_range)
        plt.axhline(trifinger_joint_velocity_limit_range[0], linestyle="--")
        plt.axhline(trifinger_joint_velocity_limit_range[1], linestyle="--")

    # Plot specific velocities
    if plot_config.vel_names:
        plot = mbp_plots.plot_velocities_by_name(
            robot_output, plot_config.vel_names, t_x_slice, vel_map
        )
    """ Plot Efforts """
    if plot_config.plot_measured_efforts or plot_config.plot_commanded_efforts:
        effort_plotter = plot_styler.PlotStyler(nrows=2)

    if plot_config.plot_measured_efforts:
        plot = mbp_plots.plot_measured_efforts(
            robot_output, act_names, t_x_slice, effort_plotter, subplot_index=0
        )
        plot.fig.axes[0].set_ylim(trifinger_joint_actuator_limit_range)

    if plot_config.plot_commanded_efforts:
        plot = mbp_plots.plot_commanded_efforts(
            robot_input, act_names, t_osc_slice, effort_plotter, subplot_index=1
        )
        plot.fig.axes[1].set_ylim(trifinger_joint_actuator_limit_range)

    if plot_config.act_names:
        plot = mbp_plots.plot_measured_efforts_by_name(
            robot_output, plot_config.act_names, t_x_slice, act_map
        )

    """ Plot OSC """
    if plot_config.plot_tracking_costs:
        plot = mbp_plots.plot_tracking_costs(osc_debug, t_osc_slice)
        # plt.ylim([0, 1e3])

    if plot_config.plot_qp_costs:
        plot = mbp_plots.plot_qp_costs(osc_debug, t_osc_slice)

    if plot_config.plot_qp_solutions:
        plot = mbp_plots.plot_ddq_sol(osc_debug, t_osc_slice, pos_names, slice(0, 7))
        plot = mbp_plots.plot_joint_velocities(robot_output, vel_names, 0, t_x_slice)
        plot = mbp_plots.plot_lambda_c_sol(osc_debug, t_osc_slice, slice(0, 3))
        # plot = mbp_plots.plot_lambda_h_sol(osc_debug, t_osc_slice, slice(0,
        # 6))

    if plot_config.tracking_datas_to_plot:
        for traj_name, config in plot_config.tracking_datas_to_plot.items():
            for deriv in config["derivs"]:
                for dim in config["dims"]:
                    plot = mbp_plots.plot_osc_tracking_data(
                        osc_debug, traj_name, dim, deriv, t_osc_slice
                    )
                    tracking_data = osc_debug["osc_debug_tracking_datas"][traj_name]
                    # plt.savefig(f"tracking_performance_{dim}.png", dpi=600)

    if plot_config.plot_qp_solve_time:
        plot = mbp_plots.plot_qp_solve_time(osc_debug, t_osc_slice)

    if plot_config.plot_active_tracking_datas:
        plot = mbp_plots.plot_active_tracking_datas(
            osc_debug,
            t_osc_slice,
            osc_debug["t_osc"],
            osc_debug["fsm"],
            plot_config.fsm_state_names,
        )
    plt.show()


if __name__ == "__main__":
    main()
