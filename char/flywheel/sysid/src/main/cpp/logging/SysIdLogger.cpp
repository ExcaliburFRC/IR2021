// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "logging/SysIdLogger.h"

#include <array>
#include <cstddef>
#include <sstream>
#include <stdexcept>

#include <frc/Notifier.h>
#include <frc/RobotBase.h>
#include <frc/RobotController.h>
#include <frc/Threads.h>
#include <frc/livewindow/LiveWindow.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/Timer.h>

SysIdLogger::SysIdLogger() {
  frc::LiveWindow::GetInstance()->DisableAllTelemetry();
  frc::SmartDashboard::PutNumber("SysIdAutoSpeed", 0.0);
  frc::SmartDashboard::PutString("SysIdTelemetry", "");
  frc::SmartDashboard::PutBoolean("SysIdRotate", false);
}

void SysIdLogger::SendData() {
  std::stringstream ss;
  std::for_each(m_data.begin(), m_data.end(),
                [&ss](auto& pt) { ss << std::to_string(pt) << ", "; });

  frc::SmartDashboard::PutString("SysIdTelemetry", ss.str());

  // Clear everything after test
  m_data.clear();
  m_autoSpeed = 0;
  m_commandVoltage = 0.0;
  m_timestamp = 0.0;
}

void SysIdLogger::UpdateThreadPriority() {
  if constexpr (!frc::RobotBase::IsSimulation()) {
    if (!frc::Notifier::SetHALThreadPriority(true, kThreadPriority + 1) ||
        !frc::SetCurrentThreadPriority(true, kThreadPriority)) {
      throw std::runtime_error("Setting the RT Priority failed\n");
    }
  }
}

void SysIdLogger::UpdateData() {
  m_autoSpeed = frc::SmartDashboard::GetNumber("SysIdAutoSpeed", 0.0);
  double robotVoltage = frc::RobotController::GetInputVoltage();
  m_commandVoltage = m_autoSpeed * robotVoltage;
  m_timestamp = frc2::Timer::GetFPGATimestamp().to<double>();
}
