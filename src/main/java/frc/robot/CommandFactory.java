// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.RobotContainer.CoralLevel;
import frc.robot.commands.AlignToPose;
import frc.robot.commands.DriveCommand;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.CommandSwerveDrivetrain.TargetMode;
import frc.robot.subsystems.arm.ArmConstants;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.climb.ClimbSubsystem;
import frc.robot.subsystems.elevator.ElevatorConstants;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.endEffector.EndEffectorConstants;
import frc.robot.subsystems.endEffector.EndEffectorSubsystem;
import frc.robot.subsystems.groundIntake.GroundIntakeConstants;
import frc.robot.subsystems.groundIntake.GroundIntakeSubsystem;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class CommandFactory {
  private final CommandSwerveDrivetrain drivetrain;
  private final ArmSubsystem arm;
  private final ElevatorSubsystem elevator;
  private final EndEffectorSubsystem endEffector;
  private final ClimbSubsystem climb;
  private final GroundIntakeSubsystem groundIntake;

  private final CommandXboxController driverController;
  private final CommandXboxController operatorController;

  public CommandFactory(
      CommandSwerveDrivetrain drivetrain,
      ArmSubsystem arm,
      ElevatorSubsystem elevator,
      EndEffectorSubsystem endEffector,
      ClimbSubsystem climb,
      GroundIntakeSubsystem groundIntake,
      CommandXboxController driverController,
      CommandXboxController operatorController,
      DriveCommand driveCommand) {
    this.drivetrain = drivetrain;
    this.arm = arm;
    this.elevator = elevator;
    this.endEffector = endEffector;
    this.climb = climb;
    this.groundIntake = groundIntake;

    this.driverController = driverController;
    this.operatorController = operatorController;
  }

  public Command zeroElevator() {
    return elevator.homingCommand();
  }

  /**
   * this is a wrapper for the command of the same name
   *
   * @param Pose pose to go to
   * @return run the command
   */
  public Command alignToPose(Supplier<Pose2d> Pose) {
    return new AlignToPose(Pose, drivetrain, () -> elevator.getHeightMeters(), driverController);
  }

  /**
   * @return prep to pickup coral
   */
  public Command prepCoralIntake(double elevatorHeight, double armAngle) {
    return Commands.parallel(
            endEffector.intake(),
            elevator.setHeight(elevatorHeight).withTimeout(0.5),
            arm.setAngle(armAngle).withTimeout(1),
            Commands.runOnce(() -> RobotContainer.robotState = RobotContainer.RobotState.INTAKE))
        .withName("Prepare Coral Intake");
  }

  public Command prepCoralIntakeAuton() {
    return Commands.parallel(
            endEffector.intake(),
            elevator.setHeight(ElevatorConstants.INTAKE_METER_AUTON).withTimeout(0.5),
            arm.setAngle(ArmConstants.ARM_INTAKE_ANGLE).withTimeout(1))
        .withName("Prepare Coral Intake Auton");
  }

  public Command prepCoralIntake() {
    return prepCoralIntake(ElevatorConstants.INTAKE_METER, ArmConstants.ARM_INTAKE_ANGLE);
  }

  public Command stopIntake() {
    return Commands.parallel(
            arm.setAngle(ArmConstants.ARM_STOW_ANGLE),
            elevator.setHeight(ElevatorConstants.STOW_METER),
            endEffector.holdCoral(),
            Commands.runOnce(() -> RobotContainer.robotState = RobotContainer.RobotState.IDLE))
        .withName("stop Intake");
  }

  public Command groundIntakeScoreL1() {
    return Commands.sequence(
            groundIntake
                .setAngleAndVoltage(GroundIntakeConstants.SCORE_CORAL_ANGLE, 6)
                .withTimeout(0.5),
            Commands.waitSeconds(0.3),
            groundIntake
                .setAngleAndVoltage(GroundIntakeConstants.CORAL_STOW_ANGLE, 0)
                .withTimeout(0.5))
        .withName("Ground Intake Score Coral L1");
  }

  public Command scoreAlgae() {
    return Commands.sequence(
            groundIntake
                .setAngleAndVoltage(GroundIntakeConstants.SCORE_ALGAE_ANGLE, -6)
                .withTimeout(0.5),
            Commands.waitSeconds(0.7),
            groundIntake
                .setAngleAndVoltage(GroundIntakeConstants.ALGAE_STOW_ANGLE, 0)
                .withTimeout(0.5))
        .withName("Score Algae");
  }

  /**
   * @param elevatorHeight how tall should the elavator be?
   * @param armAngle what angle should the arm be at
   * @return run the command
   */
  public Command prepScoreCoral(double elevatorHeight, double armAngle) {
    return Commands.parallel(
            endEffector.holdCoral(),
            elevator.setHeight(elevatorHeight).withTimeout(1.5),
            arm.setAngle(armAngle).withTimeout(1.5),
            Commands.runOnce(() -> RobotContainer.robotState = RobotContainer.RobotState.PREPSCORE))
        .withName(
            "Prepare Score Coral; Elevator Height: " + elevatorHeight + " Arm Angle: " + armAngle);
  }

  public Command prepScoreCoral(DoubleSupplier elevatorHeight, DoubleSupplier armAngle) {
    return Commands.parallel(
            endEffector.holdCoral(),
            elevator.setHeightSupplier(elevatorHeight).withTimeout(.5),
            arm.setAngleSupplier(armAngle).withTimeout(.5),
            Commands.runOnce(() -> RobotContainer.robotState = RobotContainer.RobotState.PREPSCORE))
        .withName(
            "Prepare Score Coral; Elevator Height: " + elevatorHeight + " Arm Angle: " + armAngle);
  }

  public Command prepScoreCoral(CoralLevel level) {
    DoubleSupplier elevatorHeightSupplier =
        () -> EagleUtil.getOffsetElevatorHeight(level, drivetrain.getPose());
    DoubleSupplier armAngleSupplier =
        () -> EagleUtil.getOffsetArmAngle(level, drivetrain.getPose());
    return prepScoreCoral(elevatorHeightSupplier, armAngleSupplier).repeatedly();
  }

  public Command autonScoreCoral() {
    return Commands.sequence(
        endEffector.shoot(EndEffectorConstants.VOLTAGE_L4), Commands.waitSeconds(0.05));
  }

  /**
   * @return score the coral
   */
  public Command scoreCoral() {
    Command scoreCoral =
        Commands.sequence(
                endEffector.shoot(EndEffectorConstants.VOLTAGE_L4).onlyIf(RobotContainer.IS_L4),
                endEffector.shoot(EndEffectorConstants.VOLTAGE_L3).onlyIf(RobotContainer.IS_L3),
                endEffector.shoot(EndEffectorConstants.VOLTAGE_L2).onlyIf(RobotContainer.IS_L2),
                endEffector.shoot(EndEffectorConstants.VOLTAGE_L1).onlyIf(RobotContainer.IS_L1),
                Commands.waitSeconds(0.05),
                drivetrain.driveBackward(1).withTimeout(0.2).onlyIf(RobotContainer.IS_L2),
                arm.setAngle(ArmConstants.ARM_STOW_ANGLE).withTimeout(0.0),
                elevator.setHeight(ElevatorConstants.STOW_METER).withTimeout(0.0),
                endEffector.stopMotor())
            .withTimeout(0.5);

    Command deAlgae =
        Commands.sequence(
                endEffector.shoot(EndEffectorConstants.VOLTAGE_L4).onlyIf(RobotContainer.IS_L4),
                endEffector.shoot(EndEffectorConstants.VOLTAGE_L3).onlyIf(RobotContainer.IS_L3),
                endEffector.shoot(EndEffectorConstants.VOLTAGE_L2).onlyIf(RobotContainer.IS_L2),
                endEffector.shoot(EndEffectorConstants.VOLTAGE_L1).onlyIf(RobotContainer.IS_L1),
                Commands.waitSeconds(0.04), // .05
                endEffector.stopMotor(),
                alignToPose(() -> EagleUtil.getNearestAlgaePoint(drivetrain.getPose()))
                    .withTimeout(0.6)
                    .alongWith(
                        arm.setAngle(90)
                            .alongWith(
                                Commands.either(
                                    elevator.setHeight(ElevatorConstants.DEALGAE_HIGH_POSITION),
                                    elevator.setHeight(ElevatorConstants.DEALGAE_LOW_POSITION),
                                    RobotContainer.ALGAE_HIGH))
                            .withTimeout(0.4)), // .5
                Commands.either(prepDealgaeHigh(), prepDealgaeLow(), RobotContainer.ALGAE_HIGH)
                    .withTimeout(.1) // .6
                    .deadlineFor(
                        alignToPose(() -> EagleUtil.getNearestAlgaePoint(drivetrain.getPose()))),
                dealgae())
            .withInterruptBehavior(InterruptionBehavior.kCancelIncoming);

    return Commands.sequence(
        Commands.either(deAlgae, scoreCoral, driverController.leftTrigger())
            .withName("Score Coral/deAlgae"),
        Commands.runOnce(() -> RobotContainer.robotState = RobotContainer.RobotState.IDLE)
            .withInterruptBehavior(InterruptionBehavior.kCancelIncoming));
  }

  // DeAlgae Commands
  public Command prepDealgaeLow() {
    return Commands.parallel(
            elevator.setHeight(ElevatorConstants.DEALGAE_LOW_POSITION),
            arm.setAngle(ArmConstants.PRE_DEALGAE_ANGLE),
            endEffector.setVoltage(0))
        .withName("prep Delalgae low");
  }

  public Command prepDealgaeHigh() {
    return Commands.parallel(
            elevator.setHeight(ElevatorConstants.DEALGAE_HIGH_POSITION),
            arm.setAngle(ArmConstants.PRE_DEALGAE_ANGLE),
            endEffector.setVoltage(0))
        .withName("prep Dealgae high");
  }

  public Command dealgae() {
    return Commands.sequence(
            arm.setAngle(ArmConstants.DEALGAE_ANGLE)
                .alongWith(elevator.decreaseHeight(0.1))
                .withTimeout(0.2),
            drivetrain.driveBackward(1).withTimeout(0.6),
            Commands.parallel(
                elevator.setHeight(ElevatorConstants.STOW_METER).withTimeout(.1),
                arm.setAngle(ArmConstants.ARM_STOW_ANGLE).withTimeout(.1),
                endEffector.stopMotor()))
        .withName("Dealgae");
  }

  public Command unPrepClimbCommand() {
    return Commands.sequence(
            arm.setAngle(ArmConstants.CLIMB_ANGLE).withTimeout(1),
            Commands.runOnce(() -> drivetrain.setTargetMode(TargetMode.REEF)),
            climb.stow().withTimeout(5),
            elevator.setHeight(ElevatorConstants.STOW_METER).withTimeout(1),
            arm.setAngle(ArmConstants.ARM_STOW_ANGLE))
        .withInterruptBehavior(InterruptionBehavior.kCancelIncoming)
        .withName("unPrepClimb");
  }

  public Command climb() {
    Trigger climbTrigger = operatorController.rightBumper().and(operatorController.leftBumper());

    Command climbCommand =
        Commands.parallel(
                climb.climb(),
                elevator.setHeight(0),
                arm.setAngle(ArmConstants.CLIMB_ANGLE),
                Commands.runOnce(() -> drivetrain.setTargetMode(TargetMode.NORMAL)))
            .withTimeout(0.5);

    return Commands.sequence(
            Commands.sequence(Commands.runOnce(() -> drivetrain.setTargetMode(TargetMode.CAGE))),
            groundIntake.setAngleAndVoltage(GroundIntakeConstants.CLIMB_ANGLE, 0).withTimeout(1),
            arm.setAngle(ArmConstants.PREP_CLIMB_ANGLE).withTimeout(1),
            elevator.setHeight(0).withTimeout(1),
            climb.latch().withTimeout(1),
            Commands.waitUntil(climbTrigger),
            climbCommand)
        .withInterruptBehavior(InterruptionBehavior.kCancelIncoming)
        .withName("Climb");
  }
}
