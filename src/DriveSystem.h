#pragma once

#include <BasicLinearAlgebra.h>
#include <C610Bus.h>

#include "Kinematics.h"
#include "PID.h"
#include "RobotTypes.h"

// Class for controlling the 12 (no more and no less) actuators on Pupper
class DriveSystem {
 private:
  static const size_t kNumActuators = 12;  // TODO something else with this

  C610Bus<CAN1> front_bus_;
  C610Bus<CAN2> rear_bus_;

  ActuatorPositionVector zero_position_;

  ActuatorCurrentVector last_commanded_current_;

  // Indicates which motors can have non-zero torque.
  ActuatorActivations active_mask_;

  // Maximum current for current control.
  float max_current_;

  // Maximum commandable current before system triggers a fault.
  float fault_current_;

  // Constants specific to the C610 + M2006 setup.
  static constexpr float kReduction = 36.0F;
  static constexpr float kCountsPerRad =
      C610::COUNTS_PER_REV * kReduction / (2 * M_PI);
  static constexpr float kRPMPerRadS = kReduction * 2.0F * M_PI / 60.0F;
  static constexpr float kMilliAmpPerAmp = 1000.0F;

  // Important direction multipliers
  std::array<float, 12> direction_multipliers_; /* */

  // Initialize the two CAN buses
  void InitializeDrive();

 public:
  // Construct drive system and initialize CAN buses.
  // Set position and current-control references to zero.
  DriveSystem();

  // Check for messages on the CAN bus and run callbacks.
  void CheckForCANMessages();

  // Check for errors
  DriveControlMode CheckErrors();

  // Go into idle mode, which sends 0A to all motors.
  void SetIdle();

  // Set the measured position to the zero point for the actuators.
  void ZeroCurrentPosition();

  // Set the zero point for all actuators from the provided vector.
  void SetZeroPositions(ActuatorPositionVector zero);

  // Set current target for actuator i
  void SetCurrent(uint8_t i, float target_current);

  // Set current level that would trigger a fault
  void SetFaultCurrent(float fault_current);

  // Set maximum PID and current control torque
  void SetMaxCurrent(float max_current);

  // Activates an actuator. Deactive actuators will be commanded 0 amps.
  void ActivateActuator(uint8_t i);

  // Deactivate an actuator.
  void DeactivateActuator(uint8_t i);

  void SetActivations(ActuatorActivations acts);

  // Activates all twelve actuators.
  void ActivateAll();

  // Deactivates all twelve actuators.
  void DeactivateAll();

  // Send zero torques to the escs.
  void CommandIdle();

  /*
  The ordering of the torques array goes like this:
  front-right abduction
  front-right hip
  front-right knee
  front-left ...
  back-right ...
  back-left ...

  Send torque commands to C610 escs. Converts from decimal value of amps to
  integer milliamps.
  */
  void CommandCurrents(
      ActuatorCurrentVector currents);  // passing "torques" by reference
                                        // enforces that it have 12 elements

  // Get the C610 controller object corresponding to index i.
  C610 GetController(uint8_t i);

  // Returns the output shaft's position in [radians].
  float GetActuatorPosition(uint8_t i);

  // Returns all output shaft positions [radians]
  ActuatorPositionVector GetActuatorPositions();

  // Returns the output shaft's position in [radians].
  float GetRawActuatorPosition(uint8_t i);

  // Returns all output shaft positions [radians]
  ActuatorPositionVector GetRawActuatorPositions();

  // Returns the output shaft's velocity in [radians/s].
  float GetActuatorVelocity(uint8_t i);

  // Returns the motor's actual current in [A]
  float GetActuatorCurrent(uint8_t i);