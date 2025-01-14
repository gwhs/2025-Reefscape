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
public class ElevatorSubsytem extends SubsystemBase {
  private ElevatorIO elevatorIO;

  public ElevatorSubsytem() {
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
    DogLog.log("Elevator/position",elevatorIO.getPosition());
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
