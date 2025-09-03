// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
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

    // TODO 3: press start -> score L4 Coral
    // ↓↓↓↓↓↓↓↓↓↓ COMPLETE THE COMMAND COMPOSITION IN scoreL4Coral() METHOD BELOW FIRST ↓↓↓↓↓↓↓↓↓↓
    controller.start().onTrue(scoreL4Coral());
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
        // Command 1: In parallel: (Commands.parallel())
        Commands.parallel(
            elevator.setHeight(ElevatorConstants.L4_PREP_POSITION),
            arm.setAngle(ArmConstants.L4_PREP_POSITION)),
        //     Command 1a: extend elevator to ElevatorConstants.L4_PREP_POSITION
        //     Command 1b: rotate arm to ArmConstants.L4_PREP_POSITION
        // Command 2: Spin endeffector at EndEffectorConstants.VOLTAGE_L4
        endEffector.setVoltage(EndEffectorConstants.VOLTAGE_L4),
        // Command 3: Wait 0.05 seconds ()
        Commands.waitSeconds(0.05),
        // Command 4: In parallel:
        Commands.parallel(
            arm.setAngle(ArmConstants.ARM_STOW_ANGLE),
            elevator.setHeight(ElevatorConstants.STOW_METER),
            endEffector.setVoltage(0))
        //     Command 4a: retract elevator to ElevatorConstants.STOW_METER
        //     Command 4b: rotate arm to ArmConstants.ARM_STOW_ANGLE
        //     Command 4c: spin endeffector at 0 volts
        );
  }
}
