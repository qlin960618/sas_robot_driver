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
#include "sas_robot_driver_ros_composer.hpp"
//#include <ros/callback_queue_interface.h>
#include <dqrobotics/interfaces/json11/DQ_JsonReader.h>
//#include <sas_common/sas_common.h>
#include <sas_core/sas_core.hpp>

namespace sas
{
RobotDriverROSComposer::RobotDriverROSComposer(const RobotDriverROSComposerConfiguration &configuration,
                                               std::shared_ptr<Node> &node,
                                               std::atomic_bool *break_loops):
    RobotDriver(break_loops),
    node_(node),
    configuration_(configuration),
    vi_(break_loops)
{
    if(configuration.use_real_robot)
    {
        for(const std::string& topic_prefix: configuration.robot_driver_client_names)
        {
            //ROS_INFO_STREAM(ros::this_node::getName()+"::Adding subrobot driver with prefix "+topic_prefix);
            RCLCPP_INFO_STREAM(node_->get_logger(),"::Adding RobotDriverClient driver with prefix "+topic_prefix);
            robot_driver_clients_.push_back(std::unique_ptr<RobotDriverClient>(new RobotDriverClient(node,topic_prefix)));
        }
    }

    if(configuration_.override_joint_limits_with_robot_parameter_file)
    {
        DQ_SerialManipulatorDH smdh = DQ_JsonReader::get_from_json<DQ_SerialManipulatorDH>(configuration_.robot_parameter_file_path);
        joint_limits_ = {smdh.get_lower_q_limit(),smdh.get_upper_q_limit()};
    }
}

VectorXd RobotDriverROSComposer::get_joint_positions()
{
    if(configuration_.use_real_robot)
    {
        VectorXd joint_positions;
        for(const auto& interface : robot_driver_clients_)
        {
            joint_positions = concatenate(joint_positions, interface->get_joint_positions());
        }
        return joint_positions;
    }
    else
    {
        if(vrep_thread_exited_)
        {
            RCLCPP_ERROR_STREAM(node_->get_logger(),"::get_joint_positions::Vrep thread exited.");
            throw std::runtime_error("["+ std::string(node_->get_name()) + "]::get_joint_positions::Vrep thread exited.");
        }
        return vrep_joint_states_;
    }
}

void RobotDriverROSComposer::set_target_joint_positions(const VectorXd &set_target_joint_positions_rad)
{
    if(configuration_.use_real_robot)
    {
        int accumulator = 0;
        for(const auto& interface : robot_driver_clients_)
        {
            interface->send_target_joint_positions(set_target_joint_positions_rad.segment(accumulator,interface->get_joint_positions().size()));
            accumulator+=interface->get_joint_positions().size();
        }
        vrep_desired_joint_position_ = set_target_joint_positions_rad;
    }

    if(configuration_.use_coppeliasim)
    {
        // update vrep desired joint position
        vrep_desired_joint_position_ = set_target_joint_positions_rad;
    }
}

VectorXd RobotDriverROSComposer::get_joint_velocities() {
    if(configuration_.use_real_robot)
    {
        VectorXd joint_velocity;
        for(const auto& interface : robot_driver_clients_)
        {
            joint_velocity = concatenate(joint_velocity, interface->get_joint_velocities());
        }
        return joint_velocity;
    }
    if(configuration_.use_coppeliasim){
        if(vrep_thread_exited_)
        {
            RCLCPP_ERROR_STREAM(node_->get_logger(),"::get_joint_velocities::Vrep thread exited.");
            throw std::runtime_error("["+ std::string(node_->get_name()) + "]::get_joint_velocities::Vrep thread exited.");
        }
        return VectorXd::Zero(vrep_joint_states_.size());
    }
    return VectorXd::Zero(vrep_joint_states_.size());
}

void RobotDriverROSComposer::set_target_joint_velocities(const VectorXd& desired_joint_velocities) {
    if(configuration_.use_real_robot) {
        int accumulator = 0;
        for(const auto& interface : robot_driver_clients_)
        {
            interface->send_target_joint_velocities(desired_joint_velocities.segment(accumulator,interface->get_joint_velocities().size()));
            accumulator+=interface->get_joint_velocities().size();
        }
    }
    if(configuration_.use_coppeliasim){
        //do nothihng
    }
}



void RobotDriverROSComposer::set_joint_limits(const std::tuple<VectorXd, VectorXd>&)
{
    throw std::runtime_error("RobotDriverROSComposer::set_joint_limits::Not accepted.");
}

void RobotDriverROSComposer::connect()
{
}

void RobotDriverROSComposer::disconnect()
{
    if(configuration_.use_coppeliasim)
        vi_.disconnect();
}

void RobotDriverROSComposer::initialize()
{
    if(configuration_.use_real_robot)
    {
        bool initialized = false;
        while(not initialized and not (*break_loops_))
        {
            spin_some(node_);
            initialized = true;
            for(const auto& interface : robot_driver_clients_)
            {
                if(not interface->is_enabled())
                    initialized = false;
                if(configuration_.velocity_mode_enabled) {
                    if(not interface->is_enabled(Functionality::VelocityControl))
                        initialized = false;
                }
            }
        }
        //Send initial values to CoppeliaSim
        vrep_desired_joint_position_ = get_joint_positions();
        // wait for vrep initialized
    }
    else
    {
        //Call it once to initialize the CoppeliaSim streaming.
    }
    _start_vrep_thread_main_loop();
    RCLCPP_INFO_STREAM(node_->get_logger(),"::Waiting for Vrep thread to initialize.");
    while(not vrep_side_initialized_ and not (*break_loops_))
    {
        spin_some(node_);
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        if(vrep_thread_exited_)
        {
            RCLCPP_ERROR_STREAM(node_->get_logger(),"::initialize::Vrep thread exited.");
            throw std::runtime_error("["+std::string(node_->get_name())+"]::initialize::Vrep thread exited.");
        }
    }
    driver_side_initialized_ = true;
}

void RobotDriverROSComposer::_start_vrep_thread_main_loop()
{
    vrep_thread_ = std::thread(&RobotDriverROSComposer::_vrep_thread_main_loop,this);
    vrep_thread_started_ = true;
}

void RobotDriverROSComposer::_vrep_thread_main_loop()
{
    Clock thread_clock(configuration_.vrep_clock_sampling_time_sec);
    DQ_VrepInterface vi(break_loops_);

    if(!vi.connect(configuration_.coppeliasim_ip,
                    configuration_.coppeliasim_port,
                    100,
                    10))
    {
        vrep_thread_exited_ = true;
        throw std::runtime_error("["+ std::string(node_->get_name()) + "]::_vrep_thread_main_loop::Unable to connect to CoppeliaSim.");
    }
    try {
        if(configuration_.use_real_robot) {
            vi.set_joint_positions(configuration_.coppeliasim_robot_joint_names,vrep_desired_joint_position_);
            if(configuration_.coppeliasim_dynamically_enabled_)
                vi.set_joint_target_positions(configuration_.coppeliasim_robot_joint_names,vrep_desired_joint_position_);
            vrep_joint_states_ = vi.get_joint_positions(configuration_.coppeliasim_robot_joint_names);
        }else {
            vrep_joint_states_ = vi.get_joint_positions(configuration_.coppeliasim_robot_joint_names);
            vrep_desired_joint_position_ = vrep_joint_states_;
        }

    }catch (exceptions::RCLError &e)
    {
        vrep_thread_exited_ = true;
        RCLCPP_ERROR_STREAM(node_->get_logger(),"::_vrep_thread_main_loop::Exception::"+std::string(e.what()));
    }catch (...) {
        vrep_thread_exited_ = true;
        RCLCPP_ERROR_STREAM(node_->get_logger(),"::_vrep_thread_main_loop::Exception::Unknown");
    }
    RCLCPP_INFO_STREAM(node_->get_logger(),"::_vrep_thread_main_loop::Connected to CoppeliaSim");
    try {
        if(!configuration_.use_real_robot)
        {
            // not using real robot
            vrep_desired_joint_position_ = vi.get_joint_positions(configuration_.coppeliasim_robot_joint_names);
        }

        vrep_side_initialized_ = true;
        thread_clock.init();
        while(not (*break_loops_))
        {
            thread_clock.update_and_sleep();
            if(!driver_side_initialized_) {
                continue;
            }

            if(configuration_.coppeliasim_dynamically_enabled_)
            {
                vi.set_joint_target_positions(configuration_.coppeliasim_robot_joint_names,vrep_desired_joint_position_);
            }
            else
            {
                vi.set_joint_positions(configuration_.coppeliasim_robot_joint_names,vrep_desired_joint_position_);
            }

            vrep_joint_states_ = vi.get_joint_positions(configuration_.coppeliasim_robot_joint_names);

        }
    }catch (std::exception &e)
    {
        RCLCPP_ERROR_STREAM(node_->get_logger(),"::_vrep_thread_main_loop::Exception::"+std::string(e.what()));
    }catch (...) {
        RCLCPP_ERROR_STREAM(node_->get_logger(),"::_vrep_thread_main_loop::Exception::Unknown");
    }
    vi.disconnect();
    vrep_thread_exited_ = true;
    RCLCPP_INFO_STREAM(node_->get_logger(),"::_vrep_thread_main_loop::Exiting.");
}

void RobotDriverROSComposer::deinitialize()
{
    //nothing to do
    if(vrep_thread_started_) {
        if(vrep_thread_.joinable()) {
            vrep_thread_.join();
        }
        RCLCPP_INFO_STREAM(node_->get_logger(),"::Vrep thread joined.");
    }
}

RobotDriverROSComposer::~RobotDriverROSComposer()=default;

//Defined last because QTCreator messes up the identation because of the auto [,] operator.
std::tuple<VectorXd, VectorXd> RobotDriverROSComposer::get_joint_limits() const
{
    if(!configuration_.override_joint_limits_with_robot_parameter_file)
    {
        VectorXd joint_positions_min;
        VectorXd joint_positions_max;
        for(const auto& interface : robot_driver_clients_)
        {
            auto [joint_positions_min_l, joint_positions_max_l] = interface->get_joint_limits();
            joint_positions_min = concatenate(joint_positions_min, joint_positions_min_l);
            joint_positions_max = concatenate(joint_positions_max, joint_positions_max_l);
        }
        return {joint_positions_min, joint_positions_max};
    }
    return joint_limits_;
}


}
