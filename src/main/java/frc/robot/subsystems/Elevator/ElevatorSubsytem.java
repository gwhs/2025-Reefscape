// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Elevator;

import java.util.Map;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
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

    SmartDashboard.putData("Command Testing/Elevator Motor Up", motorUp());
    SmartDashboard.putData("Command Testing/Elevator Motor Down", motorDown());

  }
  public boolean isMotorAtGoal(double leftMotorGoal, double rightMotorGoal) {
    return MathUtil.isNear(leftMotorGoal, elevatorIO.getLeftMotorPosition(), 5) && MathUtil.isNear(rightMotorGoal, elevatorIO.getRightMotorPosition(), 5);
  }


  @Override
  public void periodic() {
    elevatorIO.update();

    NetworkTableInstance.getDefault().getEntry("Elevator/Left motor Position").setNumber(elevatorIO.getLeftMotorPosition());
    NetworkTableInstance.getDefault().getEntry("Elevator/Right motor Position").setNumber(elevatorIO.getRightMotorPosition());

  }

  public Command motorUp() {
    return this.runOnce(() -> {
      elevatorIO.setPositionLeft(ElevatorConstants.LEFT_UP_POSITION);
      elevatorIO.setPositionRight(ElevatorConstants.RIGHT_UP_POSITION);
    }).andThen(Commands.waitUntil(() -> isMotorAtGoal(ElevatorConstants.LEFT_UP_POSITION, ElevatorConstants.RIGHT_UP_POSITION)))
        .withName("Motor Up");
  }

  public Command motorDown() {
    return this.runOnce(() -> {
      elevatorIO.setPositionLeft(ElevatorConstants.LEFT_DOWN_POSITION);
      elevatorIO.setPositionRight(ElevatorConstants.RIGHT_DOWN_POSITION);
    }).andThen(Commands.waitUntil(() -> isMotorAtGoal(ElevatorConstants.LEFT_UP_POSITION/2, ElevatorConstants.RIGHT_UP_POSITION/2)))
        .withName("Motor Down");
  }

  public Command motorHalfWay() {
    return this.runOnce(() -> {
      elevatorIO.setPositionLeft(ElevatorConstants.LEFT_UP_POSITION/2);
      elevatorIO.setPositionRight(ElevatorConstants.RIGHT_UP_POSITION/2);
    }).andThen(Commands.waitUntil(() -> isMotorAtGoal(ElevatorConstants.LEFT_DOWN_POSITION, ElevatorConstants.RIGHT_DOWN_POSITION)))
        .withName("Motor half way");
  }
}
