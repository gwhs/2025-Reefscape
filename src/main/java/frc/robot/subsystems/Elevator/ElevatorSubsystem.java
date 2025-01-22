// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Elevator;

import dev.doglog.DogLog;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ElevatorSubsystem extends SubsystemBase {
  private ElevatorIO elevatorIO;
  private ElevatorIOReal elevatorIOReal;

  public ElevatorSubsystem() {
    if (RobotBase.isSimulation()) {
      elevatorIO = new ElevatorIOSim();
    } else {
      elevatorIO = new ElevatorIOReal();
    }

    SmartDashboard.putData("Elevator to 0", goTo(0));
    SmartDashboard.putData("Elevator to 1", goTo(1));
    SmartDashboard.putData("Elevator to 1.3", goTo(1.3));
    SmartDashboard.putData("Elevator to 0.5", goTo(0.5));
  }

  @Override
  public void periodic() {
    elevatorIO.update();
    DogLog.log("Elevator/rotation", elevatorIO.getRotation());
    DogLog.log("Elevator/meters", rotationsToMeters(elevatorIO.getRotation()));
    DogLog.log("Elevator/Limit Switch Value (Reverse)", elevatorIO.getReverseLimit());
    DogLog.log("Elevator/Limit Switch Value (Forward)", elevatorIO.getForwardLimit());

    DogLog.log("Elevator/Max Height (meter)", ElevatorConstants.TOP_METER);
  }

  public Command goTo(double meters) {
    return this.runOnce(
            () -> {
              elevatorIO.setRotation(metersToRotations(meters));
            })
        .andThen(
            Commands.waitUntil(
                () -> MathUtil.isNear(meters, rotationsToMeters(elevatorIO.getRotation()), 0.1)));
  }

  public Command homingCommand() {
    return this.runOnce(
            () -> {
              elevatorIO.setVoltage(-3);
            })
        .andThen(Commands.waitUntil(() -> elevatorIO.getReverseLimit()))
        .andThen(
            Commands.runOnce(
                () -> {
                  elevatorIO.setVoltage(0);
                }));
  }

  public static double rotationsToMeters(double rotations) {
    return rotations
        / ElevatorConstants.GEAR_RATIO
        * (ElevatorConstants.SPROCKET_DIAMETER * Math.PI)
        * 2;
  }

  public static double metersToRotations(double meters) {
    return meters
        / (ElevatorConstants.SPROCKET_DIAMETER * Math.PI)
        * ElevatorConstants.GEAR_RATIO
        / 2;
  }
}
