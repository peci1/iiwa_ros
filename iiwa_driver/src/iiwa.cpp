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
#include <iiwa_driver/iiwa.h>

// ROS Headers
#include <control_toolbox/filters.h>
#include <controller_manager/controller_manager.h>

#include <urdf/model.h>

// FRI Headers
#include <kuka/fri/ClientData.h>

#include <thread>

namespace iiwa_ros {
    Iiwa::Iiwa(ros::NodeHandle& nh)
    {
        init(nh);
    }

    Iiwa::~Iiwa()
    {
        // Disconnect from robot
        _disconnect_fri();

        // Delete FRI message data
        if (_fri_message_data)
            delete _fri_message_data;
    }

    void Iiwa::init(ros::NodeHandle& nh)
    {
        _nh = nh;
        _load_params(); // load parameters
        _init(); // initialize

        _idle = true;
        _commanding = false;

        // Create message/client data
        _fri_message_data = new kuka::fri::ClientData(_robot_state.NUMBER_OF_JOINTS);

        // link monitoring and command message to wrappers
        _robot_state.set_message(&_fri_message_data->monitoringMsg);
        _robot_command.set_message(&_fri_message_data->commandMsg);

        // set specific message IDs
        _fri_message_data->expectedMonitorMsgID = _robot_state.monitoring_message_id();
        _fri_message_data->commandMsg.header.messageIdentifier = _robot_command.command_message_id();

        if (!_connect_fri())
            _initialized = false;
        else
            _initialized = true;
    }

    void Iiwa::run()
    {
        if (!_initialized) {
            ROS_ERROR_STREAM("Not connected to the robot. Cannot run!");
            return;
        }
        static ros::Rate rate(_control_freq);
        while (ros::ok()) {
            ros::Time time = ros::Time::now();

            // TO-DO: Get real elapsed time?
            auto elapsed_time = ros::Duration(1. / _control_freq);

            _read(elapsed_time);
            _write(elapsed_time);

            // _publish();
            rate.sleep();
        }
    }

    void Iiwa::_init()
    {
        // Get joint names
        _num_joints = _joint_names.size();

        // Resize vectors
        _joint_position.resize(_num_joints);
        _joint_velocity.resize(_num_joints);
        _joint_effort.resize(_num_joints);
        _joint_position_command.resize(_num_joints);
        _joint_velocity_command.resize(_num_joints);
        _joint_effort_command.resize(_num_joints);

        // Get the URDF XML from the parameter server
        urdf::Model urdf_model;
        std::string urdf_string;

        // search and wait for robot_description on param server
        while (urdf_string.empty()) {
            ROS_INFO_ONCE_NAMED("Iiwa", "Iiwa is waiting for model"
                                        " URDF in parameter [%s] on the ROS param server.",
                _robot_description.c_str());

            _nh.getParam(_robot_description, urdf_string);

            usleep(100000);
        }
        ROS_INFO_STREAM_NAMED("Iiwa", "Received urdf from param server, parsing...");

        const urdf::Model* const urdf_model_ptr = urdf_model.initString(urdf_string) ? &urdf_model : nullptr;
        if (urdf_model_ptr == nullptr)
            ROS_WARN_STREAM_NAMED("Iiwa", "Could not read URDF from '" << _robot_description << "' parameters. Joint limits will not work.");
    }

    void Iiwa::_load_params()
    {
        ros::NodeHandle n_p("~");

        n_p.param("fri/port", _port, 30200); // Default port is 30200
        n_p.param<std::string>("fri/robot_ip", _remote_host, "192.170.10.2"); // Default robot ip is 192.170.10.2
        n_p.param<std::string>("fri/robot_description", _robot_description, "/robot_description");

        n_p.param("hardware_interface/control_freq", _control_freq, 200.);
        n_p.getParam("hardware_interface/joints", _joint_names);
    }

    void Iiwa::_read(ros::Duration elapsed_time)
    {
        // Read data from robot (via FRI)
        kuka::fri::ESessionState current_state;
        if (!_fri_connection.isOpen()) {
            // TO-DO: Use ROS output
            // printf("Error: client application is not connected!\n");
            return;
        }

        // **************************************************************************
        // Receive and decode new monitoring message
        // **************************************************************************
        _message_size = _fri_connection.receive(_fri_message_data->receiveBuffer, kuka::fri::FRI_MONITOR_MSG_MAX_SIZE);

        if (_message_size <= 0) { // TODO: size == 0 -> connection closed (maybe go to IDLE instead of stopping?)
            // TO-DO: Use ROS output
            // printf("Error: failed while trying to receive monitoring message!\n");
            return;
        }

        if (!_fri_message_data->decoder.decode(_fri_message_data->receiveBuffer, _message_size)) {
            return;
        }

        // check message type (so that our wrappers match)
        if (_fri_message_data->expectedMonitorMsgID != _fri_message_data->monitoringMsg.header.messageIdentifier) {
            // TO-DO: Use ROS output
            // printf("Error: incompatible IDs for received message (got: %d expected %d)!\n",
            //     (int)_fri_message_data->monitoringMsg.header.messageIdentifier,
            //     (int)_fri_message_data->expectedMonitorMsgID);
            return;
        }

        current_state = (kuka::fri::ESessionState)_fri_message_data->monitoringMsg.connectionInfo.sessionState;

        if (_fri_message_data->lastState != current_state) {
            _fri_message_data->lastState = current_state;
        }

        switch (current_state) {
        case kuka::fri::MONITORING_WAIT:
        case kuka::fri::MONITORING_READY:
        case kuka::fri::COMMANDING_WAIT:
            _idle = false;
            _commanding = false;
            break;
        case kuka::fri::COMMANDING_ACTIVE:
            _idle = false;
            _commanding = true;
            break;
        case kuka::fri::IDLE: // if idle, do nothing
        default:
            _idle = true;
            _commanding = false;
            return;
        }

        // Update ROS structures
        _joint_position_prev = _joint_position;

        for (int i = 0; i < _num_joints; i++) {
            _joint_position[i] = _robot_state.getMeasuredJointPosition()[i];
            _joint_velocity[i] = filters::exponentialSmoothing((_joint_position[i] - _joint_position_prev[i]) / elapsed_time.toSec(), _joint_velocity[i], 0.2);
            _joint_effort[i] = _robot_state.getMeasuredTorque()[i];
        }
    }

    void Iiwa::_write(ros::Duration elapsed_time)
    {
        if (_idle) // if idle, do nothing
            return;

        // reset commmand message
        _fri_message_data->resetCommandMessage();

        if (_robot_state.getClientCommandMode() == kuka::fri::TORQUE) {
            _robot_command.setTorque(_joint_effort_command.data());
            _robot_command.setJointPosition(_joint_position.data());
        }
        else if (_robot_state.getClientCommandMode() == kuka::fri::POSITION)
            _robot_command.setJointPosition(_joint_position_command.data());
        // else ERROR

        // **************************************************************************
        // Encode and send command message
        // **************************************************************************

        _fri_message_data->lastSendCounter++;
        // check if its time to send an answer
        if (_fri_message_data->lastSendCounter >= _fri_message_data->monitoringMsg.connectionInfo.receiveMultiplier) {
            _fri_message_data->lastSendCounter = 0;

            // set sequence counters
            _fri_message_data->commandMsg.header.sequenceCounter = _fri_message_data->sequenceCounter++;
            _fri_message_data->commandMsg.header.reflectedSequenceCounter = _fri_message_data->monitoringMsg.header.sequenceCounter;

            if (!_fri_message_data->encoder.encode(_fri_message_data->sendBuffer, _message_size)) {
                return;
            }

            if (!_fri_connection.send(_fri_message_data->sendBuffer, _message_size)) {
                // TO-DO: Use ROS output
                // printf("Error: failed while trying to send command message!\n");
                return;
            }
        }
    }

    bool Iiwa::_connect_fri()
    {
        if (_fri_connection.isOpen()) {
            // TO-DO: Use ROS output
            // printf("Warning: client application already connected!\n");
            return true;
        }

        return _fri_connection.open(_port, _remote_host.c_str());
    }

    void Iiwa::_disconnect_fri()
    {
        if (_fri_connection.isOpen())
            _fri_connection.close();
    }
} // namespace iiwa_ros
