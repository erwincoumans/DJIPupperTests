#include "DriveSystem.h"
#define DISABLE_SAFETY

#ifndef USE_SIM
#include <ArduinoJson.h>
#include <Streaming.h>
#endif
#include "Utils.h"

DriveSystem::DriveSystem() 
#ifndef USE_SIM
    : front_bus_(), rear_bus_() 
#endif
{
  control_mode_ = DriveControlMode::kIdle;
  fault_current_ = 10.0;  // TODO: don't make this so high at default
  fault_position_ = 3.5;  // TODO: make this a param
  fault_velocity_ =
      100000.0;  // TODO: make this a param, set to something more reasonable.
  max_current_ = 0.0;
  position_reference_.fill(0.0);
  velocity_reference_.fill(0.0);
  ff_force_.Fill(0.);
  cartesian_velocity_reference_.fill(0.0);

  measured_positions_.fill(0.0);
  measured_velocities_.fill(0.0);

  current_reference_.fill(
      0.0);  // TODO: log the commanded current even when in position PID mode
  active_mask_.fill(false);
  zero_position_.fill(0.0);

  cartesian_position_gains_.kp.Fill(0.0);
  cartesian_position_gains_.kd.Fill(0.0);

#ifdef USE_SIM
  std::array<float, 12> direction_multipliers = {1, 1, 1, 1, 1, 1,
                                                 1, 1, 1, 1, 1, 1};
#else
  std::array<float, 12> direction_multipliers = {-1, -1, 1, -1, 1, -1,
                                                 -1, -1, 1, -1, 1, -1};
#endif
  direction_multipliers_ = direction_multipliers;

  SetDefaultCartesianPositions();
}

void DriveSystem::CheckForCANMessages() {
#ifndef USE_SIM
  front_bus_.PollCAN();
  rear_bus_.PollCAN();
#else
#endif
}

DriveControlMode DriveSystem::CheckErrors() {
  for (size_t i = 0; i < kNumActuators; i++) {
    // check positions
    if (abs(GetActuatorPosition(i)) > fault_position_) {
#ifndef USE_SIM
      Serial << "actuator[" << i << "] hit fault position: " << fault_position_
             << endl;
#endif
#ifndef DISABLE_SAFETY
      return DriveControlMode::kError;
#endif
    }
    // check velocities
    if (abs(GetActuatorVelocity(i)) > fault_velocity_) {
#ifndef USE_SIM
      Serial << "actuator[" << i << "] hit fault velocity: " << fault_velocity_
             << endl;
#endif
#ifndef DISABLE_SAFETY
      return DriveControlMode::kError;
#endif
    }
  }
  return DriveControlMode::kIdle;
}
#include <stdio.h>

void DriveSystem::SetIdle() { 
    printf("Setting Idle\n");
    control_mode_ = DriveControlMode::kIdle; 
}

void DriveSystem::ZeroCurrentPosition() {
  SetZeroPositions(GetRawActuatorPositions());
}

void DriveSystem::SetZeroPositions(ActuatorPositionVector zero) {
  zero_position_ = zero;
}

ActuatorCurrentVector DriveSystem::GetLastCommandedCurrents() {
    return last_commanded_current_;
}

ActuatorPositionVector DriveSystem::DefaultCartesianPositions() {
  ActuatorPositionVector pos;
  for (int i = 0; i < 4; i++) {
    BLA::Matrix<3> p = ForwardKinematics({0, 0, 0}, leg_parameters_, i) +
                       HipPosition(hip_layout_parameters_, i);
    pos[3 * i] = p(0);
    pos[3 * i + 1] = p(1);
    pos[3 * i + 2] = p(2);
  }
  return pos;
}

void DriveSystem::SetDefaultCartesianPositions() {
  cartesian_position_reference_ = DefaultCartesianPositions();
}

void DriveSystem::SetJointPositions(ActuatorPositionVector pos) {
  control_mode_ = DriveControlMode::kPositionControl;
  position_reference_ = pos;
}

void DriveSystem::SetPositionKp(float kp) { 
    position_gains_.kp = kp; 
}

void DriveSystem::SetPositionKd(float kd) { 
    position_gains_.kd = kd; 
}

void DriveSystem::SetCartesianKp(const std::array<float, 3>& kp3) {
  BLA::Matrix<3, 3> kp;
  kp.Fill(0.0);
  kp(0, 0) = kp3[0];
  kp(1, 1) = kp3[1];
  kp(2, 2) = kp3[2];
  SetCartesianKp3x3(kp);
}

void DriveSystem::SetCartesianKd(const std::array<float, 3>& kd3) {
  BLA::Matrix<3, 3> kd;
  kd.Fill(0.);
  kd(0, 0) = kd3[0];
  kd(1, 1) = kd3[1];
  kd(2, 2) = kd3[2];
  SetCartesianKd3x3(kd);
}

void DriveSystem::SetCartesianKp3x3(BLA::Matrix<3, 3> kp) {
  cartesian_position_gains_.kp = kp;
}

void DriveSystem::SetCartesianKd3x3(BLA::Matrix<3, 3> kd) {
  cartesian_position_gains_.kd = kd;
}

void DriveSystem::SetCartesianPositions(ActuatorPositionVector pos) {
  control_mode_ = DriveControlMode::kCartesianPositionControl;
  cartesian_position_reference_ = pos;
}

void DriveSystem::SetCartesianVelocities(ActuatorVelocityVector vel) {
  control_mode_ = DriveControlMode::kCartesianPositionControl;
  cartesian_velocity_reference_ = vel;
}

void DriveSystem::SetFeedForwardForce(BLA::Matrix<12> force) {
  ff_force_ = force;
}

void DriveSystem::SetCurrent(uint8_t i, float current_reference) {
  control_mode_ = DriveControlMode::kCurrentControl;
  current_reference_[i] = current_reference;
}

void DriveSystem::SetFaultCurrent(float fault_current) {
  fault_current_ = fault_current;
}

void DriveSystem::SetMaxCurrent(float max_current) {
  max_current_ = max_current;
}

std::array<float,12> DriveSystem::CartesianPositionControl2() {
    return Utils::VectorToArray<12, 12>(CartesianPositionControl());
}

BLA::Matrix<12> DriveSystem::CartesianPositionControl() {
  BLA::Matrix<12> actuator_torques;
  for (int leg_index = 0; leg_index < 4; leg_index++) {
    auto joint_angles = LegJointAngles(leg_index);
    auto joint_velocities = LegJointVelocities(leg_index);
    BLA::Matrix<3, 3> jac =
        LegJacobian(joint_angles, leg_parameters_, leg_index);

    auto measured_hip_relative_positions =
        ForwardKinematics(joint_angles, leg_parameters_, leg_index);
    auto measured_velocities = jac * joint_velocities;
    auto reference_hip_relative_positions =
        LegCartesianPositionReference(leg_index) -
        HipPosition(hip_layout_parameters_, leg_index);
    auto reference_velocities = LegCartesianVelocityReference(leg_index);

    auto cartesian_forces =
        PDControl3(measured_hip_relative_positions, measured_velocities,
                   reference_hip_relative_positions, reference_velocities,
                   cartesian_position_gains_) +
        LegFeedForwardForce(leg_index);
    auto joint_torques = ~jac * cartesian_forces;

    // Ensures that the direction of the force is preserved when motors
    // saturate
    float norm = Utils::InfinityNorm3(joint_torques);
    if (norm > max_current_) {
      joint_torques = joint_torques * max_current_ / norm;
    }

    actuator_torques(3 * leg_index) = joint_torques(0);
    actuator_torques(3 * leg_index + 1) = joint_torques(1);
    actuator_torques(3 * leg_index + 2) = joint_torques(2);
  }
  return actuator_torques;
}

void DriveSystem::Update() {
  // If there are errors, put the system in the error state.
  if (CheckErrors() == DriveControlMode::kError) {
#ifndef DISABLE_SAFETY
    control_mode_ = DriveControlMode::kError;
#endif
  }

  switch (control_mode_) {
    case DriveControlMode::kError: {
#ifndef USE_SIM
      Serial << "ERROR" << endl;
#endif
      CommandIdle();
      break;
    }
    case DriveControlMode::kIdle: {
      CommandIdle();
      break;
    }
    case DriveControlMode::kPositionControl: {
      ActuatorCurrentVector pd_current;
      for (size_t i = 0; i < kNumActuators; i++) {
        PD(pd_current[i], GetActuatorPosition(i), GetActuatorVelocity(i),
           position_reference_[i], velocity_reference_[i], position_gains_);
      }
      CommandCurrents(pd_current);
      break;
    }
    case DriveControlMode::kCartesianPositionControl: {
      CommandCurrents(Utils::VectorToArray<12, 12>(CartesianPositionControl()));
      break;
    }
    case DriveControlMode::kCurrentControl: {
      CommandCurrents(current_reference_);
      break;
    }
  }
}

void DriveSystem::SetActivations(ActuatorActivations acts) {
  active_mask_ = acts;  // Is this a copy?
}

void DriveSystem::CommandIdle() {
  ActuatorCurrentVector currents;
  currents.fill(0.0);
  CommandCurrents(currents);
}

void DriveSystem::CommandCurrents(ActuatorCurrentVector currents) {
  ActuatorCurrentVector current_command =
      Utils::Constrain(currents, -max_current_, max_current_);
  if (Utils::Maximum(current_command) > fault_current_ ||
      Utils::Minimum(current_command) < -fault_current_) {
#ifndef USE_SIM
    Serial << "Requested current too large. Erroring out." << endl;
#endif
#ifndef DISABLE_SAFETY
    control_mode_ = DriveControlMode::kError;
#endif
    return;
  }
  // Set disabled motors to zero current
  current_command = Utils::MaskArray(current_command, active_mask_);

  // Update record of last commanded current
  last_commanded_current_ = current_command;

  // Convert the currents into the motors' local frames
  current_command =
      Utils::ElemMultiply(current_command, direction_multipliers_);

  // Convert from float array to int32 array in units milli amps.
  std::array<int32_t, kNumActuators> currents_mA =
      Utils::ConvertToFixedPoint(current_command, 1000);

#ifndef USE_SIM
  // Send current commands down the CAN buses
  front_bus_.CommandTorques(currents_mA[0], currents_mA[1], currents_mA[2],
                            currents_mA[3], C610Subbus::kIDZeroToThree);
  front_bus_.CommandTorques(currents_mA[4], currents_mA[5], 0, 0,
                            C610Subbus::kIDFourToSeven);
  rear_bus_.CommandTorques(currents_mA[6], currents_mA[7], currents_mA[8],
                           currents_mA[9], C610Subbus::kIDZeroToThree);
  rear_bus_.CommandTorques(currents_mA[10], currents_mA[11], 0, 0,
                           C610Subbus::kIDFourToSeven);
#else
#endif
}

#ifndef USE_SIM
C610 DriveSystem::GetController(uint8_t i) {
  // TODO put these constants somewhere else
  if (i >= 0 && i <= 5) {
    return front_bus_.Get(i);
  } else if (i >= 6 && i <= 11) {
    return rear_bus_.Get(i - 6);
  } else {
    Serial << "Invalid actuator index. Must be 0<=i<=11." << endl;
    control_mode_ = DriveControlMode::kError;
    return C610();
  }
}
#endif
float DriveSystem::GetRawActuatorPosition(uint8_t i) {
#ifndef USE_SIM
  return GetController(i).Position();
#else
   return measured_positions_[i];
#endif
}

ActuatorPositionVector DriveSystem::GetRawActuatorPositions() {
  ActuatorPositionVector pos;
  for (size_t i = 0; i < pos.size(); i++) {
    pos[i] = GetRawActuatorPosition(i);
  }
  return pos;
}

float DriveSystem::GetActuatorPosition(uint8_t i) {
  return (GetRawActuatorPosition(i) - zero_position_[i]) *
         direction_multipliers_[i];
}

ActuatorPositionVector DriveSystem::GetActuatorPositions() {
  ActuatorPositionVector pos;
  for (size_t i = 0; i < pos.size(); i++) {
    pos[i] = GetActuatorPosition(i);
  }
  return pos;
}

float DriveSystem::GetActuatorVelocity(uint8_t i) {
#ifndef USE_SIM
  return GetController(i).Velocity() * direction_multipliers_[i];
#else
    return measured_velocities_[i];
#endif
}

float DriveSystem::GetActuatorCurrent(uint8_t i) {
#ifndef USE_SIM
  return GetController(i).Current() * direction_multipliers_[i];
#else
    return 0;
#endif
}

float DriveSystem::GetTotalElectricalPower() {
  float power = 0.0;
#ifndef USE_SIM  
  for (uint8_t i = 0; i < kNumActuators; i++) {
    power += GetController(i).ElectricalPower();
  }
#endif
  return power;
}

float DriveSystem::GetTotalMechanicalPower() {
  float power = 0.0;
#ifndef USE_SIM  
  for (uint8_t i = 0; i < kNumActuators; i++) {
    power += GetController(i).MechanicalPower();
  }
#endif
  return power;
}

BLA::Matrix<3> DriveSystem::LegJointAngles(uint8_t i) {
  return {GetActuatorPosition(3 * i), GetActuatorPosition(3 * i + 1),
          GetActuatorPosition(3 * i + 2)};
}

BLA::Matrix<3> DriveSystem::LegJointVelocities(uint8_t i) {
  return {GetActuatorVelocity(3 * i), GetActuatorVelocity(3 * i + 1),
          GetActuatorVelocity(3 * i + 2)};
}

// Get the cartesian reference position for leg i.
BLA::Matrix<3> DriveSystem::LegCartesianPositionReference(uint8_t i) {
  return {cartesian_position_reference_[3 * i],
          cartesian_position_reference_[3 * i + 1],
          cartesian_position_reference_[3 * i + 2]};
}

// Return the cartesian reference velocity for leg i.
BLA::Matrix<3> DriveSystem::LegCartesianVelocityReference(uint8_t i) {
  return {cartesian_velocity_reference_[3 * i],
          cartesian_velocity_reference_[3 * i + 1],
          cartesian_velocity_reference_[3 * i + 2]};
}

BLA::Matrix<3> DriveSystem::LegFeedForwardForce(uint8_t i) {
  return {ff_force_(3 * i), ff_force_(3 * i + 1), ff_force_(3 * i + 2)};
}

void DriveSystem::PrintHeader(DrivePrintOptions options) {
#ifndef USE_SIM
  if (options.time) {
    Serial << "T" << options.delimiter;
  }
  for (size_t i = 0; i < kNumActuators; i++) {
    if (!active_mask_[i]) continue;
    if (options.positions) {
      Serial << "p[" << i << "]" << options.delimiter;
    }
    if (options.velocities) {
      Serial << "v[" << i << "]" << options.delimiter;
    }
    if (options.currents) {
      Serial << "I[" << i << "]" << options.delimiter;
    }
    if (options.position_references) {
      Serial << "pr[" << i << "]" << options.delimiter;
    }
    if (options.velocity_references) {
      Serial << "vr[" << i << "]" << options.delimiter;
    }
    if (options.current_references) {
      Serial << "Ir[" << i << "]" << options.delimiter;
    }
    if (options.last_current) {
      Serial << "Il[" << i << "]" << options.delimiter;
    }
  }
  Serial << endl;
#endif
}

void DriveSystem::PrintMsgPackStatus(DrivePrintOptions options) {
#ifndef USE_SIM
  StaticJsonDocument<2048> doc;
  // 21 micros to put this doc together
  doc["ts"] = millis();
  for (uint8_t i = 0; i < kNumActuators; i++) {
    if (options.positions) {
      doc["pos"][i] = GetActuatorPosition(i);
    }
    if (options.velocities) {
      doc["vel"][i] = GetActuatorVelocity(i);
    }
    if (options.currents) {
      doc["cur"][i] = GetActuatorCurrent(i);
    }
    if (options.position_references) {
      doc["pref"][i] = position_reference_[i];
    }
    if (options.velocity_references) {
      doc["vref"][i] = velocity_reference_[i];
    }
    if (options.current_references) {
      doc["cref"][i] = current_reference_[i];
    }
    if (options.last_current) {
      doc["lcur"][i] = last_commanded_current_[i];
    }
  }
  uint16_t num_bytes = measureMsgPack(doc);
  // Serial.println(num_bytes);
  Serial.write(69);
  Serial.write(69);
  Serial.write(num_bytes >> 8 & 0xff);
  Serial.write(num_bytes & 0xff);
  serializeMsgPack(doc, Serial);
  Serial.println();
#endif
}

void DriveSystem::PrintStatus(DrivePrintOptions options) {
#ifndef USE_SIM
  char delimiter = options.delimiter;
  if (options.time) {
    Serial << millis() << delimiter;
  }
  for (uint8_t i = 0; i < kNumActuators; i++) {
    if (!active_mask_[i]) continue;
    if (options.positions) {
      Serial.print(GetActuatorPosition(i), 2);
      Serial << delimiter;
    }
    if (options.velocities) {
      Serial.print(GetActuatorVelocity(i), 2);
      Serial << delimiter;
    }
    if (options.currents) {
      Serial.print(GetActuatorCurrent(i), 2);
      Serial << delimiter;
    }
    if (options.position_references) {
      Serial.print(position_reference_[i], 2);
      Serial << delimiter;
    }
    if (options.velocity_references) {
      Serial.print(velocity_reference_[i], 2);
      Serial << delimiter;
    }
    if (options.current_references) {
      Serial.print(current_reference_[i], 2);
      Serial << delimiter;
    }
    if (options.last_current) {
      Serial.print(last_commanded_current_[i], 2);
      Serial << delimiter;
    }
  }
  Serial << endl;
  Serial.flush();
#endif
}

BLA::Matrix<kNumDriveSystemDebugValues> DriveSystem::DebugData() {

  uint32_t write_index = 0;
  BLA::Matrix<kNumDriveSystemDebugValues> output;
  #ifndef USE_SIM
  output(write_index++) = millis();
  for (uint8_t i = 0; i < kNumActuators; i++) {
    output(write_index++) = GetActuatorPosition(i);
    output(write_index++) = GetActuatorVelocity(i);
    output(write_index++) = GetActuatorCurrent(i);
    output(write_index++) = position_reference_[i];
    output(write_index++) = velocity_reference_[i];
    output(write_index++) = current_reference_[i];
    output(write_index++) = last_commanded_current_[i];
  }
#endif
  return output;

}