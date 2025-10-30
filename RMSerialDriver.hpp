#pragma once

// clang-format off
/* === MODULE MANIFEST V2 ===
module_description: No description provided
constructor_args:
  timestamp_offset: 0.0
  device_name: /dev/ttyUSB0
  baud_rate: 115200
  parity: LibXR::UART::Parity::NO_PARITY
template_args: []
required_hardware: []
depends:
  - qdu-future/ArmorTracker
=== END MANIFEST === */
// clang-format on
#include <string>

#include "ArmorTracker.hpp"
#include "SolveTrajectory.hpp"
#include "app_framework.hpp"
#include "linux_uart.hpp"
#include "message.hpp"
#include "thread.hpp"
#include "uart.hpp"

class RMSerialDriver : public LibXR::Application
{
 public:
  RMSerialDriver(LibXR::HardwareContainer& hw, LibXR::ApplicationManager& app,
                 double timestamp_offset, std::string device_name, int baud_rate,
                 LibXR::UART::Parity parity);

  ~RMSerialDriver();

  void OnMonitor() override {}

  float pitch_trans(float originAngle);
  float pitch_re_trans(float originAngle);
  float yaw_trans(float originAngle);
  float yaw_re_trans(float originAngle);

 private:
  void receiveData();

  void sendData(ArmorTracker::Send msg);

  void reopenPort();

  void resetTracker();

  // Serial port
  std::string device_name_;
  int baud_rate_;
  LibXR::UART::Parity parity_;

  bool initial_set_param_ = false;
  uint8_t previous_receive_color_ = 0;

  struct Receive
  {
    double pitch;
    double yaw;
    double roll;
  };

  Receive receive_msg_;
  LibXR::Thread receive_thread_;
  LibXR::LinuxUART* uart_;

  double timestamp_offset_ = 0;

  LibXR::Topic velocity_topic_ = LibXR::Topic("/current_velocity", sizeof(double));
  LibXR::Topic receive_topic_ = LibXR::Topic("/tracker/receive", sizeof(Receive));
  LibXR::Topic send_topic_{};
};
