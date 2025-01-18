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

/** Add your docs here. */
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
    SmartDashboard.putData("Elevator to 20", goTo(20));
    SmartDashboard.putData("Elevator to 40", goTo(40));
  }

  @Override
  public void periodic() {
    elevatorIO.update();
    DogLog.log("Elevator/position", elevatorIO.getPosition());
    DogLog.log("Elevator/Limit Switch Value (Reverse)", elevatorIOReal.getReverseLimit());
    DogLog.log("Elevator/Limit Switch Value (Forward)", elevatorIOReal.getForwardLimit());
  }

  public Command goTo(double meters) {
    return this.runOnce(
            () -> {
              elevatorIO.setPosition(metersToRotations(meters));
            })
        .andThen(
            Commands.waitUntil(() -> MathUtil.isNear(meters, rotationsToMeters(elevatorIO.getPosition()), 0.1)));
  }

  // homming command
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
    return rotations / ElevatorConstants.GEAR_RATIO * (ElevatorConstants.SPROCKET_DIAMETER * Math.PI);
  }

  public static double metersToRotations(double meters) {
    return meters / (ElevatorConstants.SPROCKET_DIAMETER * Math.PI) * ElevatorConstants.GEAR_RATIO;
  }

}
