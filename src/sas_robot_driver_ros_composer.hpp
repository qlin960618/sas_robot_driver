#pragma once
/*
# Copyright (c) 2016-2022 Murilo Marques Marinho
#
#    This file is part of sas_robot_driver.
#
#    sas_robot_driver is free software: you can redistribute it and/or modify
#    it under the terms of the GNU Lesser General Public License as published by
#    the Free Software Foundation, either version 3 of the License, or
#    (at your option) any later version.
#
#    sas_robot_driver is distributed in the hope that it will be useful,
#    but WITHOUT ANY WARRANTY; without even the implied warranty of
#    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#    GNU Lesser General Public License for more details.
#
#    You should have received a copy of the GNU Lesser General Public License
#    along with sas_robot_driver.  If not, see <https://www.gnu.org/licenses/>.
#
# ################################################################
#
#   Author: Murilo M. Marinho, email: murilomarinho@ieee.org
#
# ################################################################*/

#include <exception>
#include <atomic>

#include <eigen3/Eigen/Dense>

#include <sas_robot_driver/sas_robot_driver.hpp>
#include <sas_robot_driver/sas_robot_driver_client.hpp>
#include <sas_core/sas_clock.hpp>

#include <dqrobotics/interfaces/vrep/DQ_VrepInterface.h>

using namespace Eigen;

namespace sas
{

struct RobotDriverROSComposerConfiguration
{
    bool use_real_robot;

    bool use_coppeliasim;
    std::vector<std::string> coppeliasim_robot_joint_names;
    std::string coppeliasim_ip;
    int coppeliasim_port;
    bool coppeliasim_dynamically_enabled_ = false;

    std::vector<std::string> robot_driver_client_names;

    bool override_joint_limits_with_robot_parameter_file;
    std::string robot_parameter_file_path;
    bool velocity_mode_enabled = false;
    double vrep_clock_sampling_time_sec;
};

class RobotDriverROSComposer: public sas_driver::RobotDriver
{

private:
    void _start_vrep_thread_main_loop();

    void _vrep_thread_main_loop();

    std::thread vrep_thread_;
    VectorXd vrep_joint_states_;
    VectorXd vrep_desired_joint_position_;

    // singaling for vrep thread
    std::atomic_bool driver_side_initialized_{false};
    std::atomic_bool vrep_side_initialized_{false};
    std::atomic_bool vrep_thread_exited_{false};
    std::atomic_bool vrep_thread_started_{false};

protected:
    std::shared_ptr<Node> node_;

    RobotDriverROSComposerConfiguration configuration_;
    DQ_VrepInterface vi_;
    std::vector<std::unique_ptr<sas::RobotDriverClient>> robot_driver_clients_;
    //std::atomic_bool* break_loops_;
    //std::tuple<VectorXd, VectorXd> joint_limits_;
    //RobotDriver(std::atomic_bool* break_loops);
    RobotDriverROSComposer()=delete;
    RobotDriverROSComposer(const RobotDriverROSComposer&)=delete;
public:
    RobotDriverROSComposer(const RobotDriverROSComposerConfiguration& configuration,
                           std::shared_ptr<Node>& node,
                           std::atomic_bool *break_loops);

    VectorXd get_joint_positions() override;
    void set_target_joint_positions(const VectorXd& set_target_joint_positions_rad) override;
    std::tuple<VectorXd, VectorXd> get_joint_limits() const override;
    void set_joint_limits(const std::tuple<VectorXd, VectorXd>&) override;

    VectorXd get_joint_velocities() override;
    void set_target_joint_velocities(const VectorXd& desired_joint_velocities) override;

    void connect() override;
    void disconnect() override;

    void initialize() override;
    void deinitialize() override;

    ~RobotDriverROSComposer();

};
}



