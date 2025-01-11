// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Elevator;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
public class ElevatorSubsystem extends SubsystemBase {
  /** Creates a new Elevatorsubsystem. */
  private ElevatorIO elevatorIO;

  private Constraints constraints = new Constraints(ElevatorConstant.MAX_VELOCITY, ElevatorConstant.MAX_ACCELERATION);

  private ProfiledPIDController leftpidController = new ProfiledPIDController(ElevatorConstant.ELEVATOR_PID_KP,
      ElevatorConstant.ELEVATOR_PID_KI, ElevatorConstant.ELEVATOR_PID_KD, constraints);
  private ProfiledPIDController rightpidController = new ProfiledPIDController(ElevatorConstant.ELEVATOR_PID_KP,
      ElevatorConstant.ELEVATOR_PID_KI, ElevatorConstant.ELEVATOR_PID_KD, constraints);

  
  public ElevatorSubsystem() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
