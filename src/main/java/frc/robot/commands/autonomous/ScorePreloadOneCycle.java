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
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.EagleUtil;
import frc.robot.RobotContainer;
import frc.robot.subsystems.arm.ArmConstants;
import frc.robot.subsystems.elevator.ElevatorConstants;

public class ScorePreloadOneCycle extends PathPlannerAuto {
  public ScorePreloadOneCycle(RobotContainer robotContainer) {
    super(Commands.run(() -> {}));

    try {
      PathPlannerPath SCpreloadScore = PathPlannerPath.fromPathFile("SC-preload");
      Pose2d startingPose =
          new Pose2d(
              SCpreloadScore.getPoint(0).position,
              SCpreloadScore.getIdealStartingState().rotation());

      isRunning()
          .onTrue(
              Commands.sequence(
                      AutoBuilder.resetOdom(startingPose).onlyIf(() -> RobotBase.isSimulation()),
                      robotContainer.zeroElevator().onlyIf(() -> RobotBase.isReal()),
                      AutoBuilder.followPath(SCpreloadScore)
                          .deadlineFor(
                              robotContainer.prepScoreCoral(
                                  ElevatorConstants.L4_PREP_POSITION,
                                  ArmConstants.L4_PREP_POSITION)),
                      robotContainer
                          .prepScoreCoral(
                              ElevatorConstants.L4_PREP_POSITION, ArmConstants.L4_PREP_POSITION)
                          .deadlineFor(
                              robotContainer.alignToPose(
                                  () ->
                                      EagleUtil.getCachedReefPose(robotContainer.getRobotPose()))),
                      robotContainer.scoreCoral())
                  .withName("Leave and score preload coral"));

    } catch (Exception e) {
      DriverStation.reportError("Path Not Found: " + e.getMessage(), e.getStackTrace());
    }
  }
}
