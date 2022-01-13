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
//#include <iiwa_driver/iiwa.h>

//int main(int argc, char** argv)
//{
//    ros::init(argc, argv, "iiwa_hardware_interface");
//
//    ros::NodeHandle nh;
//    ros::AsyncSpinner spinner(1);
//    spinner.start();
//
//    iiwa_ros::Iiwa iiwa(nh);
//
//    iiwa.run();
//
//    spinner.stop();
//
//    return 0;
//}

// ROS Headers
#include <ros/ros.h>

#include <thread>

// FRI Headers
#include <kuka/fri/LBRCommand.h>
#include <kuka/fri/LBRState.h>
#include <kuka/fri/UdpConnection.h>
#include <kuka/fri/ClientData.h>

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
class Iiwa {
public:
  Iiwa() {
    if (init()) {
      _initialized = true;
    } else {
      _initialized = false;
    }
  }
  ~Iiwa() {
    if (_fri_connection.isOpen()) {
      _fri_connection.close();
    }
//    if (_fri_message_data) {
//      delete _fri_message_data;
//    }
  }

  void run(bool start_thread) {
    if (!_initialized) {
      ROS_ERROR_STREAM("Not connected to the robot. Cannot run!");
      return;
    }
    if (start_thread) {
      std::thread t1(&Iiwa::control_loop, this);
      t1.join();
    } else {
      control_loop();
    }
  }

private:
  bool init() {
    _fri_message_data = new kuka::fri::ClientData(7);
    // link monitoring and command message to wrappers
    _robot_state.set_message(&_fri_message_data->monitoringMsg);
    _robot_command.set_message(&_fri_message_data->commandMsg);
    // set specific message IDs
    _fri_message_data->expectedMonitorMsgID = _robot_state.monitoring_message_id();
    _fri_message_data->commandMsg.header.messageIdentifier = _robot_command.command_message_id();
    if (_fri_connection.isOpen()) {
      ROS_ERROR("already connected");
      return false;
    }
    return _fri_connection.open(30200, "192.170.10.2");
  }

  void control_loop() {
    static ros::Rate rate(200);
    while (ros::ok()) {
      _read();
      _write();
      rate.sleep();
    }
  }

  void _read() {
    kuka::fri::ESessionState current_state;
    if (!_fri_connection.isOpen()) {
      ROS_ERROR("connection not open");
      return;
    }

    _message_size = _fri_connection.receive(_fri_message_data->receiveBuffer, kuka::fri::FRI_MONITOR_MSG_MAX_SIZE);

    if (_message_size <= 0) { // TODO: size == 0 -> connection closed (maybe go to IDLE instead of stopping?)
      ROS_ERROR("failed while trying to receive monitoring message");
      return;
    }
    if (!_fri_message_data->decoder.decode(_fri_message_data->receiveBuffer, _message_size)) {
      ROS_ERROR("failed while decoding");
      return;
    }
    if (_fri_message_data->expectedMonitorMsgID != _fri_message_data->monitoringMsg.header.messageIdentifier) {
      ROS_ERROR("incompatible IDs for received message");
      return;
    }

    current_state = (kuka::fri::ESessionState) _fri_message_data->monitoringMsg.connectionInfo.sessionState;

    if (_fri_message_data->lastState != current_state) {
      _fri_message_data->lastState = current_state;
    }

    switch (current_state) {
      case kuka::fri::MONITORING_WAIT:
      case kuka::fri::MONITORING_READY:
      case kuka::fri::COMMANDING_WAIT:
        _idle = false;
        break;
      case kuka::fri::COMMANDING_ACTIVE:
        _idle = false;
        break;
      case kuka::fri::IDLE:
      default:
        _idle = true;
        return;
    }

    for (int i = 0; i < 7; i++) {
      _joint_position[i] = _robot_state.getMeasuredJointPosition()[i];
    }
  }

  void _write() {
    if (_idle) {
      ROS_INFO("idling");
      return;
    }

    _fri_message_data->resetCommandMessage();

    if (_robot_state.getClientCommandMode() == kuka::fri::TORQUE) {
//      _robot_command.setTorque(_joint_effort_command.data());
      _robot_command.setJointPosition(_joint_position.data());
    } else if (_robot_state.getClientCommandMode() == kuka::fri::POSITION) {
      _robot_command.setJointPosition(_joint_position.data());
    } else {
      ROS_INFO("neither iin torque or position command");
    }

    _fri_message_data->lastSendCounter++;
    if (_fri_message_data->lastSendCounter >= _fri_message_data->monitoringMsg.connectionInfo.receiveMultiplier) {
      _fri_message_data->lastSendCounter = 0;

      _fri_message_data->commandMsg.header.sequenceCounter = _fri_message_data->sequenceCounter++;
      _fri_message_data->commandMsg.header.reflectedSequenceCounter =
          _fri_message_data->monitoringMsg.header.sequenceCounter;

      if (!_fri_message_data->encoder.encode(_fri_message_data->sendBuffer, _message_size)) {
        ROS_ERROR("failed encoding");
        return;
      }

      if (!_fri_connection.send(_fri_message_data->sendBuffer, _message_size)) {
        ROS_ERROR("failed sending");
        return;
      }
    }
  }

  std::vector<double> _joint_position;

  // FRI Connection
  kuka::fri::UdpConnection _fri_connection;
  kuka::fri::ClientData* _fri_message_data;
  kuka::fri::DummyState _robot_state; //!< wrapper class for the FRI monitoring message
  kuka::fri::DummyCommand _robot_command; //!< wrapper class for the FRI command message
  int _message_size;
  bool _idle;
  bool _initialized;
};
} // namespace iiwa_ros


int main(int argc, char** argv) {
  ros::init(argc, argv, "iiwa_hardware_interface");

  ros::NodeHandle nh;
  ros::AsyncSpinner spinner(1);
  spinner.start();

  iiwa_ros::Iiwa iiwa;
  iiwa.run(false);

  spinner.stop();

  return 0;
}