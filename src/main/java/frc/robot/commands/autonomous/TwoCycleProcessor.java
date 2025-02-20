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

public class TwoCycleProcessor extends PathPlannerAuto {
  public TwoCycleProcessor(RobotContainer robotContainer) {
    super(Commands.run(() -> {}));

    try {
      PathPlannerPath SP_E = PathPlannerPath.fromPathFile("SP-E");
      PathPlannerPath E_CSP = PathPlannerPath.fromPathFile("E-CSP");
      PathPlannerPath CSP_D = PathPlannerPath.fromPathFile("CSP-D");
      PathPlannerPath D_CSP = PathPlannerPath.fromPathFile("D-CSP");

      double waitTime = 0.5;

      Pose2d startingPose =
          new Pose2d(SP_E.getPoint(0).position, SP_E.getIdealStartingState().rotation());

      isRunning()
          .onTrue(
              Commands.sequence(
                      AutoBuilder.resetOdom(startingPose).onlyIf(() -> RobotBase.isSimulation()),
                      AutoBuilder.followPath(SP_E))
                  .withName("Leave SP to score preload at E"));

      event("atE")
          .onTrue(
              Commands.sequence(
                      robotContainer
                          .prepScoreCoral(
                              ElevatorConstants.L4_PREP_POSITION, ArmConstants.L4_PREP_POSITION)
                          .deadlineFor(
                              robotContainer.alignToPose(
                                  () ->
                                      EagleUtil.getCachedReefPose(robotContainer.getRobotPose()))),
                      robotContainer.scoreCoral(),
                      AutoBuilder.followPath(E_CSP).alongWith(robotContainer.prepCoralIntake()))
                  .withName("E to CSP"));

      event("atCSP_E")
          .onTrue(
              Commands.sequence(
                      Commands.waitSeconds(waitTime),
                      AutoBuilder.followPath(CSP_D).alongWith(robotContainer.coralHandoff()),
                      robotContainer
                          .prepScoreCoral(
                              ElevatorConstants.L4_PREP_POSITION, ArmConstants.L4_PREP_POSITION)
                          .deadlineFor(
                              robotContainer.alignToPose(
                                  () ->
                                      EagleUtil.getCachedReefPose(robotContainer.getRobotPose()))),
                      robotContainer.scoreCoral(),
                      AutoBuilder.followPath(D_CSP).alongWith(robotContainer.prepCoralIntake()),
                      Commands.waitSeconds(waitTime),
                      robotContainer.coralHandoff())
                  .withName("CSP to D"));

    } catch (Exception e) {
      DriverStation.reportError("Path Not Found: " + e.getMessage(), e.getStackTrace());
    }
  }
}
