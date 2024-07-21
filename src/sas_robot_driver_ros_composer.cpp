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
#   Contributors: Quentin Lin
#          -- added joint velocity control option
#          -- isolate vrep to thread
# ################################################################*/
#include "sas_robot_driver_ros_composer.h"
#include <ros/callback_queue_interface.h>
#include <dqrobotics/interfaces/json11/DQ_JsonReader.h>
#include <sas_clock/sas_clock.h>
#include <sas_common/sas_common.h>

namespace sas
{
RobotDriverROSComposer::RobotDriverROSComposer(const RobotDriverROSComposerConfiguration &configuration,
                                               ros::NodeHandle &node_handle,
                                               std::atomic_bool *break_loops):
    configuration_(configuration),
    RobotDriver(break_loops)
{
    if(configuration.use_real_robot)
    {
        for(const std::string& topic_prefix: configuration.robot_driver_interface_topic_prefixes)
        {
            ROS_INFO_STREAM(ros::this_node::getName()+"::Adding subrobot driver with prefix "+topic_prefix);
            robot_driver_interface_vector_.push_back(std::unique_ptr<RobotDriverInterface>(new RobotDriverInterface(node_handle,topic_prefix)));
        }
    }
    DQ_SerialManipulatorDH smdh = DQ_JsonReader::get_from_json<DQ_SerialManipulatorDH>(configuration_.robot_parameter_file_path);
    joint_limits_ = {smdh.get_lower_q_limit(),smdh.get_upper_q_limit()};
}

VectorXd RobotDriverROSComposer::get_joint_positions()
{
    if(configuration_.use_real_robot)
    {
        VectorXd joint_positions;
        for(const auto& interface : robot_driver_interface_vector_)
        {
            joint_positions = concatenate(joint_positions, interface->get_joint_positions());
        }
        return joint_positions;
    }
    else
    {
        if(vrep_thread_exited_)
        {
            throw std::runtime_error("["+ros::this_node::getName()+"]::get_joint_positions::Vrep thread exited.");
        }
        return vrep_joint_states_;
    }
}

void RobotDriverROSComposer::set_target_joint_positions(const VectorXd &set_target_joint_positions_rad)
{
    if(configuration_.use_real_robot)
    {
        int accumulator = 0;
        for(const auto& interface : robot_driver_interface_vector_)
        {
            interface->send_target_joint_positions(set_target_joint_positions_rad.segment(accumulator,interface->get_joint_positions().size()));
            accumulator+=interface->get_joint_positions().size();
        }
    }
    // update vrep desired joint position
    vrep_desired_joint_position_ = set_target_joint_positions_rad;
}

VectorXd RobotDriverROSComposer::get_joint_velocities() {
    if(configuration_.use_real_robot)
    {
        VectorXd joint_velocity;
        for(const auto& interface : robot_driver_interface_vector_)
        {
            joint_velocity = concatenate(joint_velocity, interface->get_joint_velocities());
        }
        return joint_velocity;
    }
    else
    {
        if(vrep_thread_exited_)
        {
            throw std::runtime_error("["+ros::this_node::getName()+"]::get_joint_positions::Vrep thread exited.");
        }
        return VectorXd::Zero(vrep_joint_states_.size());
    }
}
void RobotDriverROSComposer::set_target_joint_velocities(const VectorXd& desired_joint_velocities) {
    if(configuration_.use_real_robot) {
        int accumulator = 0;
        for(const auto& interface : robot_driver_interface_vector_)
        {
            interface->send_target_joint_velocities(desired_joint_velocities.segment(accumulator,interface->get_joint_velocities().size()));
            accumulator+=interface->get_joint_velocities().size();
        }
    }else {
        //do nothihng
    }
}

void RobotDriverROSComposer::set_joint_limits(const std::tuple<VectorXd, VectorXd> &joint_limits)
{
    throw std::runtime_error("RobotDriverROSComposer::set_joint_limits::Not accepted.");
}

void RobotDriverROSComposer::connect()
{

}

void RobotDriverROSComposer::disconnect()
{
}

void RobotDriverROSComposer::initialize()
{
    if(configuration_.use_real_robot)
    {
        bool initialized = false;
        while(not initialized and not (*break_loops_))
        {
            ros::spinOnce();
            initialized = true;
            for(const auto& interface : robot_driver_interface_vector_)
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
    ROS_INFO_STREAM("["+ros::this_node::getName()+"]::Waiting for Vrep thread to initialize.");
    while(not vrep_side_initialized_ and not (*break_loops_))
    {
        ros::spinOnce();
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        if(vrep_thread_exited_)
        {
            throw std::runtime_error("["+ros::this_node::getName()+"]::initialize::Vrep thread exited.");
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
    Clock thread_clock(configuration_.vrep_clock_sampling_time_nsec);
    DQ_VrepInterface vi(break_loops_);

    if(!vi.connect(configuration_.vrep_ip,
                    configuration_.vrep_port,
                    100,
                    10))
    {
        vrep_thread_exited_ = true;
        throw std::runtime_error("["+ros::this_node::getName()+"]::_vrep_thread_main_loop::Unable to connect to CoppeliaSim.");
    }
    try {
        if(configuration_.use_real_robot) {
            vi.set_joint_positions(configuration_.vrep_robot_joint_names,vrep_desired_joint_position_);
            if(configuration_.vrep_dynamically_enabled_)
                vi.set_joint_target_positions(configuration_.vrep_robot_joint_names,vrep_desired_joint_position_);
            vrep_joint_states_ = vi.get_joint_positions(configuration_.vrep_robot_joint_names);
        }else {
            vrep_joint_states_ = vi.get_joint_positions(configuration_.vrep_robot_joint_names);
            vrep_desired_joint_position_ = vrep_joint_states_;
        }

    }catch (ros::Exception &e)
    {
        vrep_thread_exited_ = true;
        ROS_ERROR_STREAM("["+ros::this_node::getName()+"]::_vrep_thread_main_loop::Exception::"+e.what());
    }catch (...) {
        vrep_thread_exited_ = true;
        ROS_ERROR_STREAM("["+ros::this_node::getName()+"]::_vrep_thread_main_loop::Exception::Unknown");
    }
    ROS_INFO_STREAM(ros::this_node::getName()+"::_vrep_thread_main_loop::Connected to CoppeliaSim");
    try {
        if(!configuration_.use_real_robot)
        {
            // not using real robot
            vrep_desired_joint_position_ = vi.get_joint_positions(configuration_.vrep_robot_joint_names);
        }

        vrep_side_initialized_ = true;
        thread_clock.init();
        while(not (*break_loops_))
        {

            if(configuration_.vrep_dynamically_enabled_)
            {
                vi.set_joint_target_positions(configuration_.vrep_robot_joint_names,vrep_desired_joint_position_);
            }
            else
            {
                vi.set_joint_positions(configuration_.vrep_robot_joint_names,vrep_desired_joint_position_);
            }

            vrep_joint_states_ = vi.get_joint_positions(configuration_.vrep_robot_joint_names);

            thread_clock.update_and_sleep();
        }
    }catch (std::exception &e)
    {
        ROS_ERROR_STREAM(ros::this_node::getName()+"::_vrep_thread_main_loop::Exception::"+e.what());
    }catch (...) {
        ROS_ERROR_STREAM(ros::this_node::getName()+"::_vrep_thread_main_loop::Exception::Unknown");
    }
    vi.disconnect();
    vrep_thread_exited_ = true;
    ROS_INFO_STREAM(ros::this_node::getName()+"::_vrep_thread_main_loop::Exiting.");
}

void RobotDriverROSComposer::deinitialize()
{
    //nothing to do
    if(vrep_thread_started_) {
        if(vrep_thread_.joinable()) {
            vrep_thread_.join();
        }
        ROS_INFO_STREAM("["+ros::this_node::getName()+"]::Vrep thread joined.");
    }
}

RobotDriverROSComposer::~RobotDriverROSComposer()=default;

}
