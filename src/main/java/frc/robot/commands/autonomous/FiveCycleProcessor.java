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
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.RobotContainer;

public class FiveCycleProcessor extends PathPlannerAuto {
  public FiveCycleProcessor(RobotContainer robotContainer, boolean partTwoOnly) {
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

      double waitTime = 0.05;
      double scoringTime = 0.2;

      isRunning()
          .and(new Trigger(() -> !partTwoOnly))
          .onTrue(
              Commands.sequence(
                      AutoBuilder.resetOdom(
                          new Pose2d(
                              SC_F.getPoint(0).position, SC_F.getIdealStartingState().rotation())),
                      AutoBuilder.followPath(SC_F),
                      Commands.waitSeconds(scoringTime))
                  .withName("Leave SC to score preload at F"));

      event("atF")
          .or(isRunning().and(new Trigger(() -> partTwoOnly)))
          .onTrue(
              Commands.sequence(
                      AutoBuilder.resetOdom(
                          new Pose2d(
                              F_CSP.getPoint(0).position, SC_F.getIdealStartingState().rotation())),
                      AutoBuilder.followPath(F_CSP))
                  .withName("F to CSP"));

      event("atCSP_F")
          .onTrue(
              Commands.sequence(
                      Commands.waitSeconds(waitTime),
                      AutoBuilder.followPath(CSP_E),
                      Commands.waitSeconds(scoringTime),
                      AutoBuilder.followPath(E_CSP))
                  .withName("CSP to E"));

      event("atCSP_E")
          .onTrue(
              Commands.sequence(
                      Commands.waitSeconds(waitTime),
                      AutoBuilder.followPath(CSP_D),
                      Commands.waitSeconds(scoringTime),
                      AutoBuilder.followPath(D_CSP))
                  .withName("CSP to D"));

      event("atCSP_D")
          .onTrue(
              Commands.sequence(
                      Commands.waitSeconds(waitTime),
                      AutoBuilder.followPath(CSP_C),
                      Commands.waitSeconds(scoringTime),
                      AutoBuilder.followPath(C_CSP))
                  .withName("CSP to C"));

      event("atCSP_C")
          .onTrue(
              Commands.sequence(
                      Commands.waitSeconds(waitTime),
                      AutoBuilder.followPath(CSP_B),
                      Commands.waitSeconds(scoringTime),
                      AutoBuilder.followPath(B_CSP))
                  .withName("CSP to B"));

    } catch (Exception e) {
      DriverStation.reportError("Path Not Found: " + e.getMessage(), e.getStackTrace());
    }
  }
}
