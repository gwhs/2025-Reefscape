// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.EagleUtil;
import frc.robot.RobotContainer;
import frc.robot.subsystems.arm.ArmConstants;
import frc.robot.subsystems.elevator.ElevatorConstants;

public class CenterAuton extends PathPlannerAuto {

  private RobotContainer robotContainer;

  private double waitTime = 1.5;

  public CenterAuton(RobotContainer robotContainer, boolean nonProcessorSide) {
    super(Commands.run(() -> {}));

    this.robotContainer = robotContainer;

    try {
      PathPlannerPath test1 = PathPlannerPath.fromPathFile("test1");
      PathPlannerPath test2 = PathPlannerPath.fromPathFile("test2");

      Pose2d startingPose =
          new Pose2d(test1.getPoint(0).position, test1.getIdealStartingState().rotation());

      isRunning()
          .onTrue(
              Commands.sequence(
                  AutoBuilder.resetOdom(startingPose).onlyIf(() -> RobotBase.isSimulation()),
                  AutoBuilder.followPath(test1),
                  AutoBuilder.followPath(test2)));

    } catch (Exception e) {
      DriverStation.reportError("Path Not Found: " + e.getMessage(), e.getStackTrace());
    }
  }

  public Command autoHelper(PathPlannerPath pathOne, PathPlannerPath pathTwo) {
    return Commands.sequence(
        // wait until coral is loaded
        Commands.waitUntil(robotContainer.IS_CORAL_LOADED),
        // Commands.waitSeconds(waitTime),
        // drive to scoring position
        AutoBuilder.followPath(pathOne)
            .deadlineFor(
                Commands.sequence(
                    Commands.waitSeconds(.3),
                    robotContainer
                        .prepScoreCoral(
                            ElevatorConstants.INTAKE_METER, ArmConstants.L4_PREP_POSITION)
                        .withTimeout(0.02),
                    Commands.waitSeconds(0.6),
                    robotContainer.prepScoreCoral(
                        ElevatorConstants.L4_PREP_POSITION, ArmConstants.L4_PREP_POSITION))),
        Commands.sequence(
                Commands.waitSeconds(.1)
                    .deadlineFor(robotContainer.prepScoreCoral(RobotContainer.CoralLevel.L4)),
                robotContainer.autonScoreCoral())
            .deadlineFor(
                robotContainer.alignToPose(
                    () -> EagleUtil.getCachedReefPose(robotContainer.getRobotPose()))),
        AutoBuilder.followPath(pathTwo).alongWith(robotContainer.prepCoralIntakeAuton()));
  }
}
