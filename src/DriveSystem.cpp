#include "DriveSystem.h"

#include <Streaming.h>

#include "Utils.h"

DriveSystem::DriveSystem() : front_bus_(), rear_bus_() {
  control_mode_ = DriveControlMode::kIdle;
  fault_current_ = 10.0;  // TODO: don't make this so high at default
  fault_position_ = 3.5;  // TODO: make this a param
  fault_velocity_ =
      100000.0;  // TODO: make this a param, set to something more reasonable.
  max_current_ = 0.0;
  position_reference_.fill(0.0);
  velocity_reference_.fill(0.0);
  current_reference_.fill(
      0.0);  // TODO: log the commanded current even when in position PID mode
  active_mask_.fill(false);
  zero_position_.fill(0.0);

  cartesian_position_gains_.kp.Fill(0.0);
  cartesian_position_gains_.kd.Fill(0.0);

  std::array<float, 12> direction_multipliers = {-1, -1, 1, -1, 1, -1,
                                                 -1, -1, 1, -1, 1, -1};
  direction_multipliers_ = direction_multipliers;

  SetDefaultCartesianPositions();
}

void DriveSystem::CheckForCANMessages() {
  front_bus_.pollCAN();
  rear_bus_.pollCAN();
}

DriveControlMode DriveSystem::CheckErrors() {
  for (size_t i = 0; i < kNumActuators; i++) {
    // check positions
    if (abs(GetActuatorPosition(i)) > fault_position_) {
      Serial << "actuator[" << i << "] hit fault position: " << fault_position_
             << endl;
      return DriveControlMode::kError;
    }
    // check velocities
    if (abs(GetActuatorVelocity(i)) > fault_velocity_) {
      Serial << "actuator[" << i << "] hit fault velocity: " << fault_velocity_
             << endl;
      return DriveControlMode::kError;
    }
  }
  return DriveControlMode::kIdle;
}

void DriveSystem::SetIdle() { control_mode_ = DriveControlMode::kIdle; }

void DriveSystem::ZeroCurrentPosition() {
  SetZeroPositions(GetRawActuatorPositions());
}

void DriveSystem::SetZeroPositions(ActuatorPositionVector zero) {
  zero_position_ = zero;
}

ActuatorPositionVector DriveSystem::CartesianPositions(
    BLA::Matrix<3> joint_angles) {
  ActuatorPositionVector pos;
  for (int i = 0; i < 4; i++) {
    auto p =
        ForwardKinematics(joint_angles, leg_parameters_, i) + HipPosition(i);
    pos[3 * i] = p(0);
    pos[3 * i + 1] = p(1);
    pos[3 * i + 2] = p(2);
  }
  return pos;
}

ActuatorPositionVector DriveSystem::DefaultCartesianPositions() {
  return CartesianPositions({0, 0, 0});
}

void DriveSystem::SetDefaultCartesianPositions() {
  cartesian_position_reference_ = DefaultCartesianPositions();
}

void DriveSystem::SetAllPositions(ActuatorPositionVector pos) {
  control_mode_ = DriveControlMode::kPositionControl;
  position_reference_ = pos;
}

void DriveSystem::SetPosition(uint8_t i, float position_reference) {
  control_mode_ = DriveControlMode::kPositionControl;
  position_reference_[i] = position_reference;
}

void DriveSystem::SetPositionKp(float kp) { position_gains_.kp = kp; }

void DriveSystem::SetPositionKd(float kd) { position_gains_.kd = kd; }

void DriveSystem::SetPositionGains(PDGains gains) { position_gains_ = gains; }

void DriveSystem::SetCartesianKp3x3(BLA::Matrix<3, 3> kp) {
  cartesian_position_gains_.kp = kp;
}

void DriveSystem::SetCartesianKd3x3(BLA::Matrix<3, 3> kd) {
  cartesian_position_gains_.kd = kd;
}

void DriveSystem::SetCartesianKp(BLA::Matrix<3> kp) {
  cartesian_position_gains_.kp = DiagonalMatrix3x3(kp);
}

void DriveSystem::SetCartesianKd(BLA::Matrix<3> kd) {
  cartesian_position_gains_.kd = DiagonalMatrix3x3(kd);
}

void DriveSystem::SetCartesianPositions(ActuatorPositionVector pos) {
  control_mode_ = DriveControlMode::kCartesianPositionControl;
  cartesian_position_reference_ = pos;
}

void DriveSystem::SetCartesianVelocities(ActuatorVelocityVector vel) {
  control_mode_ = DriveControlMode::kCartesianPositionControl;
  cartesian_velocity_reference_ = vel;
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

void DriveSystem::ActivateActuator(uint8_t i) { active_mask_[i] = true; }

void DriveSystem::DeactivateActuator(uint8_t i) { active_mask_[i] = false; }

void DriveSystem::SetActivations(ActuatorActivations acts) {
  active_mask_ = acts;  // Is this a copy?
}

void DriveSystem::ActivateAll() {
  for (size_t i = 0; i < kNumActuators; i++) {
    ActivateActuator(i);
  }
}

void DriveSystem::DeactivateAll() {
  for (size_t i = 0; i < kNumActuators; i++) {
    DeactivateActuator(i);
  }
}

void DriveSystem::CommandIdle() {
  ActuatorCurrentVector currents;
  currents.fill(0.0);
  CommandCurrents(currents);
}

void DriveSystem::CommandCurrents(ActuatorCurrentVector currents) {
  ActuatorCurrentVector current_command =
      Constrain(currents, -max_current_, max_current_);
  // TODO: kind of redundant with constrain right above
  if (Maximum(current_command) > fault_current_ ||
      Minimum(current_command) < -fault_current_) {
    Serial << "Requested current too large. Erroring out." << endl;
    control_mode_ = DriveControlMode::kError;
    return;
  }
  // Set disabled motors to zero current
  current_command = MaskArray(current_command, active_mask_);

  // Update record of last commanded current
  last_commanded_current_ = current_command;

  // Convert the currents into the motors' local frames
  current_command = ElemMultiply(current_command, direction_multipliers_);

  // Convert from float array to int32 array in units milli amps.
  std::array<int32_t, kNumActuators> currents_mA =
      ConvertToFixedPoint(current_command, kMilliAmpPerAmp);

  // Send current commands down the CAN buses
  front_bus_.commandTorques(currents_mA[0], currents_mA[1], currents_mA[2],
                            currents_mA[3], 0);
  front_bus_.commandTorques(currents_mA[4], currents_mA[5], 0, 0, 1);
  rear_bus_.commandTorques(currents_mA[6], currents_mA[7], currents_mA[8],
                           currents_mA[9], 0);
  rear_bus_.commandTorques(currents_mA[10], currents_mA[11], 0, 0, 1);
}

C610 DriveSystem::GetController(uint8_t i) {
  // TODO put these constants somewhere else
  if (i >= 0 && i <= 5) {
    return front_bus_.get(i);
  } else if (i >= 6 && i <= 11) {
    return rear_bus_.get(i - 6);
  } else {
    Serial << "Invalid actuator index. Must be 0<=i<=11." << endl;
    control_mode_ = DriveControlMode::kError;
    return C610();
  }
}

float DriveSystem::GetRawActuatorPosition(uint8_t i) {
  return (GetController(i).counts() / kCountsPerRad);
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
  return (GetController(i).rpm() / kRPMPerRadS) * direction_multipliers_[i];
}

float DriveSystem::GetActuatorCurrent(uint8_t i) {
  return (GetController(i).torque() / kMilliAmpPerAmp) *
         direction_multipliers_[i];
}