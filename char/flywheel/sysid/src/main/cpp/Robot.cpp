// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"

#include <hal/HAL.h>
#include <rev/CANSparkMax.h>
#include <signal.h>

#include <cstddef>
#include <exception>
#include <iostream>
#include <memory>
#include <string>

#include <frc/Filesystem.h>
#include <frc/RobotController.h>
#include <frc/Spark.h>
#include <frc/TimedRobot.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <wpi/FileSystem.h>
#include <wpi/SmallString.h>
#include <wpi/StringMap.h>
#include <wpi/raw_ostream.h>

#include "rev/CANEncoder.h"

Robot::Robot() : frc::TimedRobot(5_ms) {
  std::error_code ec;

  wpi::SmallString<128> path;
  wpi::raw_svector_ostream os{path};

//  if constexpr (RobotBase::IsSimulation()) {
//    os << PROJECT_ROOT_DIR << "/src/main/deploy/config.json";
//
//  } else {
    frc::filesystem::GetDeployDirectory(path);
    os << "/config.json";
//  }

  wpi::raw_fd_istream is{path.c_str(), ec};

  if (ec) {
    wpi::outs() << "File error: " << path.c_str() << "\n";
    wpi::outs().flush();
    throw std::runtime_error("Unable to read: " + m_path);
  }

  is >> m_json;

  wpi::outs() << "reading json \n";
  wpi::outs().flush();

  try {
    std::vector<int> ports =
        m_json.at("primary motor ports").get<std::vector<int>>();
    std::vector<std::string> controllerNames =
        m_json.at("motor controllers").get<std::vector<std::string>>();
    std::vector<int> encoderPorts =
        m_json.at("primary encoder ports").get<std::vector<int>>();
    std::vector<bool> motorsInverted =
        m_json.at("primary motors inverted").get<std::vector<bool>>();

    std::string encoderType = m_json.at("encoder type").get<std::string>();
    bool encoderInverted = m_json.at("primary encoder inverted").get<bool>();

    double cpr = m_json.at("cpr").get<double>();

    wpi::outs() << "Initializing motors \n";
    wpi::outs().flush();
    for (size_t i = 0; i < ports.size(); i++) {
      AddMotorController(ports[i], controllerNames[i], motorsInverted[i]);
    }

    wpi::outs() << "Initializing encoder\n";
    wpi::outs().flush();
    if (encoderType == "Built-In") {
      if (controllerNames[0] == "TalonSRX" || controllerNames[0] == "TalonFX") {
        if (controllerNames[0] == "TalonSRX") {
          dynamic_cast<WPI_BaseMotorController*>(m_controllers.at(0).get())
              ->ConfigSelectedFeedbackSensor(FeedbackDevice::QuadEncoder);
        } else {
          dynamic_cast<WPI_BaseMotorController*>(m_controllers.at(0).get())
              ->ConfigSelectedFeedbackSensor(FeedbackDevice::IntegratedSensor);
        }
        dynamic_cast<WPI_BaseMotorController*>(m_controllers.at(0).get())
            ->SetSensorPhase(encoderInverted);
        m_position = [&, this] {
          return dynamic_cast<WPI_BaseMotorController*>(
                     m_controllers.at(0).get())
                     ->GetSelectedSensorPosition() /
                 cpr;
        };
        m_rate = [&, this] {
          return dynamic_cast<WPI_BaseMotorController*>(
                     m_controllers.at(0).get())
                     ->GetSelectedSensorVelocity() /
                 cpr / 0.1;  // Conversion factor from 100 ms to seconds
        };
      } else {
        if (controllerNames[0] != "SPARK MAX (Brushless)") {
          m_canencoder = std::make_unique<rev::CANEncoder>(
              *static_cast<rev::CANSparkMax*>(m_controllers.at(0).get()),
              rev::CANEncoder::EncoderType::kQuadrature, cpr);
          m_canencoder->SetInverted(encoderInverted);
        } else {
          m_canencoder = std::make_unique<rev::CANEncoder>(
              *static_cast<rev::CANSparkMax*>(m_controllers.at(0).get()));
        }
        m_position = [&, this] { return m_cancoder->GetPosition(); };
        m_rate = [&, this] { return m_canencoder->GetVelocity() / 60; };
      }
    } else if (encoderType == "CANCoder / Alternate") {
      if (controllerNames[0] == "SPARK MAX (Brushless)" ||
          controllerNames[0] == "SPARK MAX (Brushed)") {
        m_canencoder = std::make_unique<rev::CANEncoder>(
            *static_cast<rev::CANSparkMax*>(m_controllers.at(0).get()),
            rev::CANEncoder::AlternateEncoderType::kQuadrature, cpr);

        m_canencoder->SetInverted(encoderInverted);
        m_position = [&, this] { return m_canencoder->GetPosition(); };
        m_rate = [&, this] { return m_canencoder->GetVelocity() / 60; };
      } else {
        m_cancoder = std::make_unique<CANCoder>(encoderPorts[0]);
        m_cancoder->ConfigSensorDirection(encoderInverted);
        m_position = [&, this] { return m_cancoder->GetPosition() / cpr; };

        m_rate = [&, this] { return m_cancoder->GetVelocity() / cpr; };
      }
    } else {
      m_encoder =
          std::make_unique<frc::Encoder>(encoderPorts[0], encoderPorts[1]);
      m_encoder->SetDistancePerPulse(1 / cpr);
      m_encoder->SetReverseDirection(encoderInverted);
      m_position = [&, this] { return m_encoder->GetDistance(); };

      m_rate = [&, this] { return m_encoder->GetRate(); };
    }
  } catch (std::exception& e) {
    wpi::outs() << "Project failed: " << e.what() << "\n";
    wpi::outs().flush();
    std::exit(-1);
  }
#ifdef INTEGRATION
  // TODO use std::exit or EndCompetition once CTRE bug is fixed
  std::set_terminate([]() { std::_Exit(0); });
#endif
}

void Robot::AddMotorController(int port, std::string controller,
                               bool inverted) {
  if (controller == "TalonSRX" || controller == "VictorSPX" ||
      controller == "TalonFX") {
    if (controller == "TalonSRX") {
      m_controllers.emplace_back(std::make_unique<WPI_TalonSRX>(port));
    } else if (controller == "TalonFX") {
      m_controllers.emplace_back(std::make_unique<WPI_TalonFX>(port));
    } else {
      m_controllers.emplace_back(std::make_unique<WPI_VictorSPX>(port));
    }
    dynamic_cast<WPI_BaseMotorController*>(m_controllers.back().get())
        ->SetInverted(inverted);
  } else if (controller == "SPARK MAX (Brushless)" ||
             controller == "SPARK MAX (Brushed)") {
    if (controller == "SPARK MAX (Brushless)") {
      m_controllers.emplace_back(std::make_unique<rev::CANSparkMax>(
          port, rev::CANSparkMax::MotorType::kBrushless));
    } else {
      m_controllers.emplace_back(std::make_unique<rev::CANSparkMax>(
          port, rev::CANSparkMax::MotorType::kBrushed));
    }
    static_cast<rev::CANSparkMax*>(m_controllers.back().get())
        ->SetInverted(inverted);
  } else {
    m_controllers.emplace_back(std::make_unique<frc::Spark>(port));
    static_cast<frc::Spark*>(m_controllers.back().get())->SetInverted(inverted);
  }
  m_logger.UpdateThreadPriority();
}

void Robot::RobotInit() {}

/**
 * This function is called every robot packet, no matter the mode. Use
 * this for items like diagnostics that you want ran during disabled,
 * autonomous, teleoperated and test.
 *
 * <p> This runs after the mode specific periodic functions, but before
 * LiveWindow and SmartDashboard integrated updating.
 */
void Robot::RobotPeriodic() {
  // TODO Put actual readings once supported
  // frc::SmartDashboard::PutNumber("Position", m_position());
  // frc::SmartDashboard::PutNumber("Rate", m_rate());
}

/**
 * This autonomous (along with the chooser code above) shows how to select
 * between different autonomous modes using the dashboard. The sendable chooser
 * code works with the Java SmartDashboard. If you prefer the LabVIEW Dashboard,
 * remove all of the chooser code and uncomment the GetString line to get the
 * auto name from the text box below the Gyro.
 *
 * You can add additional auto modes by adding additional comparisons to the
 * if-else structure below with additional strings. If using the SendableChooser
 * make sure to add them to the chooser code above as well.
 */
void Robot::AutonomousInit() {}

/**
 * Outputs data in the format: timestamp, voltage, position, velocity
 */
void Robot::AutonomousPeriodic() {
  m_logger.Log(m_position(), m_rate());
  SetMotorControllers(m_logger.GetMotorVoltage());
}

void Robot::SetMotorControllers(units::volt_t power) {
  for (auto&& controller : m_controllers) {
    controller->SetVoltage(power);
  }
}

void Robot::TeleopInit() {}

void Robot::TeleopPeriodic() {}

void Robot::DisabledInit() {
  SetMotorControllers(units::volt_t{0});
  wpi::outs() << "Robot disabled\n";
  m_logger.SendData();
}

void Robot::SimulationPeriodic() {
#ifdef INTEGRATION
  if (frc::SmartDashboard::GetBoolean("SysIdKill", false)) {
    // TODO use std::exit or EndCompetition once CTRE bug is fixed
    std::terminate();
  }
#endif
}

void Robot::DisabledPeriodic() {}

void Robot::TestInit() {}

void Robot::TestPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif
