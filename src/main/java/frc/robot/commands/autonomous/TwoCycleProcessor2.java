// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotContainer;

public class TwoCycleProcessor2 extends PathPlannerAuto {
  public TwoCycleProcessor2(RobotContainer robotContainer) {
    super(Commands.run(() -> {}));

    try {
      PathPlannerPath E_CSP = PathPlannerPath.fromPathFile("E-CSP");
      PathPlannerPath CSP_D = PathPlannerPath.fromPathFile("CSP-D");
      PathPlannerPath D_CSP = PathPlannerPath.fromPathFile("D-CSP");

      double waitTime = 0.5;

      Pose2d startingPose =
          new Pose2d(E_CSP.getPoint(0).position, E_CSP.getIdealStartingState().rotation());

      isRunning()
          .onTrue(
              Commands.sequence(
                      AutoBuilder.resetOdom(startingPose),
                      AutoBuilder.followPath(E_CSP).alongWith(robotContainer.prepCoralIntake()))
                  .withName("E to CSP"));

      event("atCSP_E")
          .onTrue(
              Commands.sequence(
                      robotContainer.prepCoralIntake(),
                      Commands.waitSeconds(waitTime),
                      AutoBuilder.followPath(CSP_D).alongWith(robotContainer.coralHandoff()),
                      robotContainer.prepScoreCoralL4(),
                      robotContainer.scoreCoralL4Command(),
                      AutoBuilder.followPath(D_CSP).alongWith(robotContainer.prepCoralIntake()),
                      Commands.waitSeconds(waitTime),
                      robotContainer.coralHandoff())
                  .withName("CSP to D"));

    } catch (Exception e) {
      DriverStation.reportError("Path Not Found: " + e.getMessage(), e.getStackTrace());
    }
  }
}
