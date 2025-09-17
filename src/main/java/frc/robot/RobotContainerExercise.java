// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.arm.ArmConstants;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.elevator.ElevatorConstants;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.endEffector.EndEffectorConstants;
import frc.robot.subsystems.endEffector.EndEffectorSubsystem;
import frc.robot.subsystems.groundIntake.GroundIntakeSubsystem;

public class RobotContainerExercise {

  /********************
   * Subsystems
   ********************/
  private final ElevatorSubsystem elevator = new ElevatorSubsystem();

  private final ArmSubsystem arm = new ArmSubsystem();
  private final EndEffectorSubsystem endEffector = new EndEffectorSubsystem();
  private final GroundIntakeSubsystem groundIntake = new GroundIntakeSubsystem();

  private final CommandXboxController controller = new CommandXboxController(0);

  private final RobotVisualizer robotVisualizer = new RobotVisualizer(elevator, arm, groundIntake);

  public RobotContainerExercise() {
    configureBindings();

    SmartDashboard.putData("Command Scheduler", CommandScheduler.getInstance());
  }

  public void configureBindings() {
    /********************
     * Button Bindings
     ********************/
    controller.a().onTrue(elevator.setHeight(0.3));
    controller.b().onTrue(elevator.setHeight(.75));
    controller.x().onTrue(elevator.setHeight(0));

    // TODO 1: press left bumper: set arm angle to 120
    controller.leftBumper().onTrue(arm.setAngle(120));

    // TODO 2: press right bumper -> set arm angle to -90
    controller.rightBumper().onTrue(arm.setAngle(-90));

    // TODO 3: press start -> score L4 Coral (finish in command composition first)
    controller.start().and(controller.back()).debounce(1).onTrue(scoreL4Coral());

    controller.start().and(controller.back()).negate().debounce(1).onTrue(arm.setAngle(90));

    controller.start().negate().and(controller.back()).debounce(1).onTrue(arm.setAngle(-90));
  }

  public void periodic() {
    robotVisualizer.update();
  }

  /********************
   * Command Compositions
   ********************/
  public Command scoreL4Coral() {
    return Commands.sequence(
            // TODO:
            // Command 1: In parallel: extend elevator to ElevatorConstants.L4_PREP_POSITION +
            // rotate
            // arm to ArmConstants.L4_PREP_POSITION (Commands.parallel())
            Commands.parallel(
                arm.setAngle(ArmConstants.L4_PREP_POSITION),
                elevator.setHeight(ElevatorConstants.L4_PREP_POSITION)),
            // Command 2: Spin endeffector at EndEffectorConstants.VOLTAGE_L4
            endEffector.setVoltage(EndEffectorConstants.VOLTAGE_L4),
            // Command 3: Wait 0.05 seconds (Commands.waitSeconds())
            Commands.waitSeconds(0.5),
            // Command 4: In parallel: retract elevator to ElevatorConstants.STOW_METER + rotate arm
            // to
            Commands.parallel(
                elevator.setHeight(ElevatorConstants.STOW_METER),
                arm.setAngle(ArmConstants.ARM_STOW_ANGLE),
                endEffector.setVoltage(0)))
        .withTimeout(0.01)
        .withName("Score L4 Coral");
    // ArmConstants.ARM_STOW_ANGLE + spin endeffector at 0 git volts

  }
}
