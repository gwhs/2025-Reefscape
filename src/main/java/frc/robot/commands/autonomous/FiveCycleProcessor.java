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

public class FiveCycleProcessor extends PathPlannerAuto {
  public FiveCycleProcessor(RobotContainer robotContainer) {
    super(Commands.run(() -> {}));

    try {
      PathPlannerPath SC_F = PathPlannerPath.fromPathFile("SC-F");
      PathPlannerPath F_CSP = PathPlannerPath.fromPathFile("F-CSP");
      PathPlannerPath CSP_E = PathPlannerPath.fromPathFile("CSP-E");
      PathPlannerPath E_CSP = PathPlannerPath.fromPathFile("E-CSP");
      PathPlannerPath CSP_D = PathPlannerPath.fromPathFile("CSP-D");
      PathPlannerPath D_CSP = PathPlannerPath.fromPathFile("D-CSP");
      PathPlannerPath CSP_C = PathPlannerPath.fromPathFile("CSP-C");
      PathPlannerPath C_CSP = PathPlannerPath.fromPathFile("C-CSP");
      PathPlannerPath CSP_B = PathPlannerPath.fromPathFile("CSP-B");
      PathPlannerPath B_CSP = PathPlannerPath.fromPathFile("B-CSP");

      double waitTime = 0.5;

      Pose2d startingPose =
          new Pose2d(SC_F.getPoint(0).position, SC_F.getIdealStartingState().rotation());

      isRunning()
          .onTrue(
              Commands.sequence(
                      AutoBuilder.resetOdom(startingPose).onlyIf(() -> RobotBase.isSimulation()),
                      AutoBuilder.followPath(SC_F)
                          .deadlineFor(
                              Commands.sequence(
                                  robotContainer.zeroElevator(),
                                  robotContainer.prepScoreCoral(
                                      ElevatorConstants.L4_PREP_POSITION,
                                      ArmConstants.L4_PREP_POSITION))))
                  .withName("Leave SC"));

      event("atF")
          .onTrue(
              Commands.sequence(
                      robotContainer
                          .prepScoreCoralL4()
                          .deadlineFor(
                              robotContainer.alignToPose(
                                  () ->
                                      EagleUtil.getCachedReefPose(robotContainer.getRobotPose()))),
                      robotContainer.scoreCoral(),
                      AutoBuilder.followPath(F_CSP).alongWith(robotContainer.prepCoralIntake()))
                  .withName("Score preload at F"));

      event("atCSP_F")
          .onTrue(
              Commands.sequence(
                      Commands.waitSeconds(waitTime),
                      AutoBuilder.followPath(CSP_E)
                          .deadlineFor(
                              Commands.sequence(
                                  Commands.waitSeconds(0.2),
                                  robotContainer.prepScoreCoral(
                                      ElevatorConstants.L4_PREP_POSITION,
                                      ArmConstants.L4_PREP_POSITION))),
                      robotContainer
                          .prepScoreCoralL4()
                          .deadlineFor(
                              robotContainer.alignToPose(
                                  () ->
                                      EagleUtil.getCachedReefPose(robotContainer.getRobotPose()))),
                      robotContainer.scoreCoral(),
                      AutoBuilder.followPath(E_CSP).alongWith(robotContainer.prepCoralIntake()))
                  .withName("CSP to E"));

      event("atCSP_E")
          .onTrue(
              Commands.sequence(
                      Commands.waitSeconds(waitTime),
                      AutoBuilder.followPath(CSP_D)
                          .deadlineFor(
                              Commands.sequence(
                                  Commands.waitSeconds(0.2),
                                  robotContainer.prepScoreCoral(
                                      ElevatorConstants.L4_PREP_POSITION,
                                      ArmConstants.L4_PREP_POSITION))),
                      robotContainer
                          .prepScoreCoralL4()
                          .deadlineFor(
                              robotContainer.alignToPose(
                                  () ->
                                      EagleUtil.getCachedReefPose(robotContainer.getRobotPose()))),
                      robotContainer.scoreCoral(),
                      AutoBuilder.followPath(D_CSP).alongWith(robotContainer.prepCoralIntake()))
                  .withName("CSP to D"));

      event("atCSP_D")
          .onTrue(
              Commands.sequence(
                      Commands.waitSeconds(waitTime),
                      AutoBuilder.followPath(CSP_C)
                          .deadlineFor(
                              Commands.sequence(
                                  Commands.waitSeconds(0.2),
                                  robotContainer.prepScoreCoral(
                                      ElevatorConstants.L4_PREP_POSITION,
                                      ArmConstants.L4_PREP_POSITION))),
                      robotContainer
                          .prepScoreCoralL4()
                          .deadlineFor(
                              robotContainer.alignToPose(
                                  () ->
                                      EagleUtil.getCachedReefPose(robotContainer.getRobotPose()))),
                      robotContainer.scoreCoral(),
                      AutoBuilder.followPath(C_CSP).alongWith(robotContainer.prepCoralIntake()))
                  .withName("CSP to C"));

      /*event("atCSP_C")
      .onTrue(
          Commands.sequence(
                  Commands.waitSeconds(waitTime),
                  AutoBuilder.followPath(CSP_B).alongWith(robotContainer.coralHandoff()),
                  robotContainer
                      .prepScoreCoralL4()
                      .deadlineFor(
                          robotContainer.alignToPose(
                              () ->
                                  EagleUtil.getCachedReefPose(robotContainer.getRobotPose()))),
                  robotContainer.scoreCoral(),
                  AutoBuilder.followPath(B_CSP).alongWith(robotContainer.prepCoralIntake()))
              .withName("CSP to B"));*/

    } catch (Exception e) {
      DriverStation.reportError("Path Not Found: " + e.getMessage(), e.getStackTrace());
    }
  }
}
