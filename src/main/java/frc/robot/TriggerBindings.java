// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.RobotContainer.CoralLevel;
import frc.robot.commands.DriveCommand;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.CommandSwerveDrivetrain.DriveMode;
import frc.robot.subsystems.CommandSwerveDrivetrain.ReefPositions;
import frc.robot.subsystems.CommandSwerveDrivetrain.TargetMode;
import frc.robot.subsystems.arm.ArmConstants;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.climb.ClimbSubsystem;
import frc.robot.subsystems.elevator.ElevatorConstants;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.endEffector.EndEffectorSubsystem;
import frc.robot.subsystems.groundIntake.GroundIntakeConstants;
import frc.robot.subsystems.groundIntake.GroundIntakeSubsystem;

/** Add your docs here. */
public class TriggerBindings {

  public static void configureBindings(
      CommandSwerveDrivetrain drivetrain,
      ArmSubsystem arm,
      ElevatorSubsystem elevator,
      EndEffectorSubsystem endEffector,
      ClimbSubsystem climb,
      GroundIntakeSubsystem groundIntake,
      CommandXboxController driverController,
      CommandXboxController operatorController,
      DriveCommand driveCommand,
      CommandFactory commandFactory) {
    // BATTERY_BROWN_OUT.onTrue(drivetrain.setDriveMotorCurrentLimit());

    // drivetrain
    //     .IS_ALIGNING_TO_POSE
    //     .and(drivetrain.IS_AT_TARGET_POSE)
    //     .onTrue(led.setPattern(LEDPattern.solid(Color.kGreen)));
    // drivetrain
    //     .IS_ALIGNING_TO_POSE
    //     .and(drivetrain.IS_AT_TARGET_POSE.negate())
    //     .onTrue(led.setPattern(LEDPattern.solid(Color.kBlack)));

    RobotContainer.IS_DISABLED.onTrue(
        Commands.runOnce(
                () -> {
                  // drivetrain.configNeutralMode(NeutralModeValue.Coast);
                  // elevator.setNeutralMode(NeutralModeValue.Coast);
                  driveCommand.stopDrivetrain();
                })
            .ignoringDisable(true));

    // IS_DISABLED
    //     .and(() -> RobotController.getBatteryVoltage() >= 12)
    //     .onTrue(led.setPattern(LEDPattern.solid(Color.kGreen)));

    // IS_DISABLED
    //     .and(() -> RobotController.getBatteryVoltage() < 12)
    //     .onTrue(led.setPattern(LEDPattern.solid(Color.kRed)));

    RobotContainer.IS_DISABLED.onFalse(
        Commands.runOnce(
                () -> {
                  // drivetrain.configNeutralMode(NeutralModeValue.Brake);
                  // elevator.setNeutralMode(NeutralModeValue.Brake);
                })
            .andThen(groundIntake.setAngleAndVoltage(GroundIntakeConstants.CORAL_STOW_ANGLE, 0))
            .ignoringDisable(false));

    // IS_DISABLED
    //     .and(() -> RobotController.getBatteryVoltage() < 12)
    //     .onTrue(EagleUtil.triggerAlert(batteryUnderTwelveVolts));

    driverController
        .x()
        .or(driverController.y())
        .whileTrue(
            Commands.startEnd(
                    () -> drivetrain.setTargetMode(TargetMode.CORAL_STATION),
                    () -> {
                      drivetrain.setTargetMode(TargetMode.REEF);
                      drivetrain.setReefMode(ReefPositions.FRONT_REEF);
                    })
                .withName("Face Coral Station"));

    // m_driverController
    //     .x()
    //     .and(IS_NEAR_CORAL_STATION)
    //     .onTrue(Commands.runOnce(() -> driveCommand.setSlowMode(true, 0.25)))
    //     .onFalse(Commands.runOnce(() -> driveCommand.setSlowMode(false, 0)));

    driverController
        .x()
        .whileTrue(commandFactory.prepCoralIntake())
        .onFalse(commandFactory.stopIntake());
    driverController
        .y()
        .whileTrue(
            commandFactory.prepCoralIntake(
                ElevatorConstants.INTAKE_METER_BACKUP, ArmConstants.ARM_INTAKE_ANGLE_BACKUP))
        .onFalse(commandFactory.stopIntake());

    // IS_TELEOP
    //     .and(IS_CORAL_LOADED)
    //     .and(IS_INTAKE.debounce(.5))
    //     .onTrue(drivetrain.driveBackward(-4.5).withTimeout(.5));

    // IS_TELEOP
    //     .and(IS_REEFMODE)
    //     .and(IS_CLOSE_TO_REEF)
    //     .onTrue(
    //         prepScoreCoral(ElevatorConstants.STOW_METER, 220).withName("auto prep score coral"));

    driverController
        .leftBumper()
        .onTrue(
            Commands.runOnce(() -> drivetrain.setTargetMode(TargetMode.NORMAL))
                .withName("Back to Original State"));

    driverController
        .rightTrigger()
        .onFalse(
            commandFactory
                .scoreCoral()
                .withInterruptBehavior(InterruptionBehavior.kCancelIncoming)
                .withName("score Coral"));

    driverController
        .rightTrigger()
        .negate()
        .and(RobotContainer.ALGAE_HIGH.negate())
        .and(driverController.leftTrigger())
        .onTrue(commandFactory.prepDealgaeLow());

    driverController
        .rightTrigger()
        .negate()
        .and(RobotContainer.ALGAE_HIGH)
        .and(driverController.leftTrigger())
        .onTrue(commandFactory.prepDealgaeHigh());

    driverController
        .rightTrigger()
        .negate()
        .and(driverController.leftTrigger())
        .whileTrue(
            commandFactory.alignToPose(() -> EagleUtil.getNearestAlgaePoint(drivetrain.getPose())))
        .whileTrue(
            Commands.startEnd(
                    () -> {
                      drivetrain.setDriveMode(DriveMode.ROBOT_CENTRIC);
                    },
                    () -> {
                      drivetrain.setDriveMode(DriveMode.FIELD_CENTRIC);
                    })
                .withName("DeALgae Robot Centric"));

    driverController.leftTrigger().onFalse(commandFactory.dealgae());

    RobotContainer.IS_L4
        .and(driverController.rightTrigger())
        .whileTrue(commandFactory.prepScoreCoral(CoralLevel.L4));
    RobotContainer.IS_L3
        .and(driverController.rightTrigger())
        .whileTrue(commandFactory.prepScoreCoral(CoralLevel.L3));
    RobotContainer.IS_L2
        .and(driverController.rightTrigger())
        .whileTrue(commandFactory.prepScoreCoral(CoralLevel.L2));
    RobotContainer.IS_L1
        .and(driverController.rightTrigger())
        .whileTrue(commandFactory.prepScoreCoral(CoralLevel.L1));

    // m_operatorController
    //     .leftStick()
    //     .whileTrue(groundIntake.setAngleAndVoltage(GroundIntakeConstants.INTAKE_ALGAE_ANGLE, 6))
    //     .onFalse(groundIntake.setAngleAndVoltage(GroundIntakeConstants.ALGAE_STOW_ANGLE, 2));

    operatorController
        .rightStick()
        .whileTrue(groundIntake.setAngleAndVoltage(GroundIntakeConstants.SCORE_ALGAE_ANGLE, 1))
        .onFalse(commandFactory.scoreAlgae());

    operatorController
        .rightStick()
        .whileTrue(
            Commands.runOnce(() -> drivetrain.setTargetMode(TargetMode.PROCESSOR))
                .withName("Face Processor"))
        .onFalse(
            Commands.waitSeconds(0.5)
                .andThen(Commands.runOnce(() -> drivetrain.setTargetMode(TargetMode.REEF))));

    // m_operatorController
    //     .leftStick()
    //     .whileTrue(
    //         Commands.startEnd(
    //                 () -> driveCommand.setTargetMode(DriveCommand.TargetMode.NORMAL),
    //                 () -> driveCommand.setTargetMode(DriveCommand.TargetMode.REEF))
    //             .withName("Ground Intake normal"));

    // m_operatorController
    //     .x()
    //     .whileTrue(
    //         Commands.startEnd(
    //                 () -> driveCommand.setTargetMode(DriveCommand.TargetMode.NORMAL),
    //                 () -> driveCommand.setTargetMode(DriveCommand.TargetMode.REEF))
    //             .withName("Algae Normal"));

    RobotContainer.IS_L1
        .and(drivetrain.IS_REEF_MODE)
        .onTrue(
            Commands.runOnce(
                () -> {
                  drivetrain.setReefMode(ReefPositions.BACK_REEF);
                }));

    RobotContainer.IS_L2
        .or(RobotContainer.IS_L3)
        .or(RobotContainer.IS_L4)
        .and(drivetrain.IS_REEF_MODE)
        .onTrue(
            Commands.runOnce(
                () -> {
                  drivetrain.setReefMode(ReefPositions.FRONT_REEF);
                }));

    driverController.start().onTrue(Commands.runOnce(drivetrain::seedFieldCentric));

    driverController
        .rightBumper()
        .whileTrue(
            Commands.startEnd(
                    () -> {
                      driveCommand.setSlowMode(true, 0.25);
                    },
                    () -> {
                      driveCommand.setSlowMode(false, 0.25);
                    })
                .withName("Slow Mode"));

    driverController
        .rightTrigger()
        .whileTrue(
            Commands.startEnd(
                    () -> {
                      drivetrain.setDriveMode(DriveMode.ROBOT_CENTRIC);
                      driveCommand.setSlowMode(true, 0.25);
                    },
                    () -> {
                      drivetrain.setDriveMode(DriveMode.FIELD_CENTRIC);
                      driveCommand.setSlowMode(false, 0.25);
                    })
                .withName("Slow and Robot Centric"));

    drivetrain
        .IS_AT_TARGET_POSE
        .and(drivetrain.IS_ALIGNING_TO_POSE)
        .and(RobotContainer.IS_PREPSCORE)
        .and(elevator.AT_GOAL_HEIGHT)
        .and(arm.AT_GOAL_ANGLE)
        .and(RobotContainer.IS_TELEOP)
        .debounce(0.33)
        .onTrue(commandFactory.scoreCoral());

    RobotContainer.IS_L1
        .and(driverController.a())
        .whileTrue(
            commandFactory.alignToPose(() -> EagleUtil.getClosestL1Back(drivetrain.getPose(0.25))));

    // IS_L2
    //     .and(m_driverController.a())
    //     .whileTrue(alignToPose(() ->
    // EagleUtil.getClosestLeftReefBack(drivetrain.getPose(0.25))));

    RobotContainer.IS_L4
        .or(RobotContainer.IS_L3)
        .or(RobotContainer.IS_L2)
        .and(driverController.a())
        .whileTrue(
            commandFactory.alignToPose(
                () -> EagleUtil.getClosestLeftReef(drivetrain.getPose(0.25))));

    RobotContainer.IS_L4
        .or(RobotContainer.IS_L3)
        .or(RobotContainer.IS_L2)
        .and(driverController.b())
        .whileTrue(
            commandFactory.alignToPose(
                () -> EagleUtil.getClosestRightReef(drivetrain.getPose(0.25))));

    // IS_L2
    //     .and(m_driverController.b())
    //     .whileTrue(alignToPose(() ->
    // EagleUtil.getClosestRightReefBack(drivetrain.getPose(0.25))));

    RobotContainer.IS_L1
        .and(driverController.b())
        .whileTrue(
            commandFactory.alignToPose(
                () -> EagleUtil.getClosestRightReefBack(drivetrain.getPose(0.25))));

    operatorController.start().onTrue(elevator.homingCommand());

    operatorController
        .leftStick()
        .whileTrue(
            commandFactory.alignToPose(
                () -> EagleUtil.getClosestCoralStation(drivetrain.getPose()))); // TODO

    operatorController
        .x()
        .onTrue(
            Commands.runOnce(
                () -> {
                  RobotContainer.coralLevel = CoralLevel.L1;
                  driveCommand.setInverted(true);
                }));

    operatorController
        .a()
        .onTrue(
            Commands.runOnce(
                () -> {
                  RobotContainer.coralLevel = CoralLevel.L2;
                  driveCommand.setInverted(false);
                }));

    operatorController
        .b()
        .onTrue(
            Commands.runOnce(
                () -> {
                  RobotContainer.coralLevel = CoralLevel.L3;
                  driveCommand.setInverted(false);
                }));

    operatorController
        .y()
        .onTrue(
            Commands.runOnce(
                () -> {
                  RobotContainer.coralLevel = CoralLevel.L4;
                  driveCommand.setInverted(false);
                }));

    // m_operatorController.y().whileTrue(arm.sysIdQuasistatic(Direction.kForward));
    // m_operatorController.b().whileTrue(arm.sysIdQuasistatic(Direction.kReverse));
    // m_operatorController.a().whileTrue(arm.sysIdDynamic(Direction.kForward));
    // m_operatorController.x().whileTrue(arm.sysIdDynamic(Direction.kReverse));

    operatorController.povRight().onTrue(arm.increaseAngle(3.0));
    operatorController.povLeft().onTrue(arm.decreaseAngle(3.0));
    operatorController.povUp().onTrue(elevator.increaseHeight(0.02));
    operatorController.povDown().onTrue(elevator.decreaseHeight(0.02));

    // m_operatorController.leftBumper().onTrue(groundIntake.decreaseAngle(3));
    // m_operatorController.rightBumper().onTrue(groundIntake.increaseAngle(3));

    operatorController
        .leftTrigger()
        .and(operatorController.rightTrigger())
        .onTrue(commandFactory.climb());
  }
}
