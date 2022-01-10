//|
//|    Copyright (C) 2019 Learning Algorithms and Systems Laboratory, EPFL, Switzerland
//|    Authors:  Konstantinos Chatzilygeroudis (maintainer)
//|              Bernardo Fichera
//|              Walid Amanhoud
//|    email:    costashatz@gmail.com
//|              bernardo.fichera@epfl.ch
//|              walid.amanhoud@epfl.ch
//|    Other contributors:
//|              Yoan Mollard (yoan@aubrune.eu)
//|    website:  lasa.epfl.ch
//|
//|    This file is part of iiwa_ros.
//|
//|    iiwa_ros is free software: you can redistribute it and/or modify
//|    it under the terms of the GNU General Public License as published by
//|    the Free Software Foundation, either version 3 of the License, or
//|    (at your option) any later version.
//|
//|    iiwa_ros is distributed in the hope that it will be useful,
//|    but WITHOUT ANY WARRANTY; without even the implied warranty of
//|    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//|    GNU General Public License for more details.
//|
#ifndef IIWA_DRIVER_IIWA_H
#define IIWA_DRIVER_IIWA_H

// ROS Headers
#include <ros/ros.h>
#include <std_msgs/Bool.h>

#include <realtime_tools/realtime_publisher.h>

#include <iiwa_driver/AdditionalOutputs.h>
#include <std_msgs/Float64MultiArray.h>

#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>

#include <joint_limits_interface/joint_limits.h>
#include <joint_limits_interface/joint_limits_interface.h>
#include <joint_limits_interface/joint_limits_rosparam.h>
#include <joint_limits_interface/joint_limits_urdf.h>

// FRI Headers
#include <kuka/fri/LBRCommand.h>
#include <kuka/fri/LBRState.h>
#include <kuka/fri/UdpConnection.h>

namespace controller_manager {
    class ControllerManager;
}

namespace kuka {
    namespace fri {
        class ClientData;

        class DummyState : public LBRState {
        public:
            FRIMonitoringMessage* message() { return _message; }
            void set_message(FRIMonitoringMessage* msg) { _message = msg; }
            int monitoring_message_id() { return LBRMONITORMESSAGEID; }
        };

        class DummyCommand : public LBRCommand {
        public:
            FRICommandMessage* message() { return _message; }
            void set_message(FRICommandMessage* msg) { _message = msg; }
            int command_message_id() { return LBRCOMMANDMESSAGEID; }
        };
    } // namespace fri
} // namespace kuka

namespace iiwa_ros {
    class Iiwa : public hardware_interface::RobotHW {
    public:
        Iiwa(ros::NodeHandle& nh);
        ~Iiwa();

        void init(ros::NodeHandle& nh);
        void run();

    protected:
        void _init();
        void _load_params();
        void _read(ros::Duration elapsed_time);
        void _write(ros::Duration elapsed_time);
        bool _connect_fri();
        void _disconnect_fri();

        // Shared memory
        int _num_joints;
        int _joint_mode; // position, velocity, or effort
        std::vector<std::string> _joint_names;
        std::vector<int> _joint_types;
        std::vector<double> _joint_position, _joint_position_prev;
        std::vector<double> _joint_velocity;
        std::vector<double> _joint_effort;
        std::vector<double> _joint_position_command;
        std::vector<double> _joint_velocity_command;
        std::vector<double> _joint_effort_command;

        // FRI Connection
        kuka::fri::UdpConnection _fri_connection;
        kuka::fri::ClientData* _fri_message_data;
        kuka::fri::DummyState _robot_state; //!< wrapper class for the FRI monitoring message
        kuka::fri::DummyCommand _robot_command; //!< wrapper class for the FRI command message
        int _message_size;
        bool _idle, _commanding;

        int _port;
        std::string _remote_host;

        // ROS communication/timing related
        ros::NodeHandle _nh;
        std::string _robot_description;
        ros::Duration _control_period;
        ros::Publisher _commanding_status_pub;
        double _control_freq;
        bool _initialized;
    };
} // namespace iiwa_ros

#endif
