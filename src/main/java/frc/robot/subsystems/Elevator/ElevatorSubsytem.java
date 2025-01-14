// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Elevator;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.RobotBase;
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

  public Command goTo(double position) {
    return this.runOnce(
            () -> {
              elevatorIO.setPosition(position);
            })
        .andThen(
            Commands.waitUntil(() -> MathUtil.isNear(position, elevatorIO.getPosition(), 0.1)));
  }

  // homming command

}
