// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Elevator;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** Add your docs here. */
public class ElevatorSubsytem extends SubsystemBase {
  private ElevatorIO elevatorIO;

  public ElevatorSubsytem() {
    if (RobotBase.isSimulation()) {
      elevatorIO = new ElevatorIOSim();
    } else {
      elevatorIO = new ElevatorIOReal();
    }

  }

  @Override
  public void periodic() {
    elevatorIO.update();
  }

  public Command motorUp() {
    return this.runOnce(() -> {
      elevatorIO.setPositionLeft(ElevatorConstants.LEFT_UP_POSITION);
      elevatorIO.setPositionRight(ElevatorConstants.RIGHT_UP_POSITION);
    }).andThen(Commands.waitUntil(() -> isMotorAtGoal(ElevatorConstants.LEFT_UP_POSITION, ElevatorConstants.RIGHT_UP_POSITION)))
        .withName("Motor Up");
  }

  public Command goTo(double meters) {
    // spin the motor

  }

}
