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

public class FiveCycleProcessor2 extends PathPlannerAuto {
  public FiveCycleProcessor2(RobotContainer robotContainer) {
    super(Commands.run(() -> {}));

    try {
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

      Pose2d startingPose =
          new Pose2d(F_CSP.getPoint(0).position, F_CSP.getIdealStartingState().rotation());

      isRunning()
          .onTrue(
              Commands.sequence(AutoBuilder.resetOdom(startingPose), AutoBuilder.followPath(F_CSP))
                  .withName("F to CSP"));

      event("atCSP_F")
          .onTrue(
              Commands.sequence(
                      robotContainer.prepCoralIntake(),
                      robotContainer.coralHandoff(),
                      Commands.waitSeconds(waitTime),
                      AutoBuilder.followPath(CSP_E),
                      robotContainer.prepScoreCoralL4(),
                      robotContainer.scoreCoral(),
                      Commands.waitSeconds(scoringTime),
                      AutoBuilder.followPath(E_CSP),
                      robotContainer.prepCoralIntake(),
                      robotContainer.coralHandoff())
                  .withName("CSP to E"));

      event("atCSP_E")
          .onTrue(
              Commands.sequence(
                      Commands.waitSeconds(waitTime),
                      AutoBuilder.followPath(CSP_D),
                      robotContainer.prepScoreCoralL4(),
                      robotContainer.scoreCoral(),
                      Commands.waitSeconds(scoringTime),
                      AutoBuilder.followPath(D_CSP),
                      AutoBuilder.followPath(D_CSP),
                      robotContainer.prepCoralIntake(),
                      robotContainer.coralHandoff())
                  .withName("CSP to D"));

      event("atCSP_D")
          .onTrue(
              Commands.sequence(
                      Commands.waitSeconds(waitTime),
                      AutoBuilder.followPath(CSP_C),
                      robotContainer.prepScoreCoralL4(),
                      robotContainer.scoreCoral(),
                      Commands.waitSeconds(scoringTime),
                      AutoBuilder.followPath(C_CSP),
                      robotContainer.prepCoralIntake(),
                      robotContainer.coralHandoff())
                  .withName("CSP to C"));

      event("atCSP_C")
          .onTrue(
              Commands.sequence(
                      Commands.waitSeconds(waitTime),
                      AutoBuilder.followPath(CSP_B),
                      robotContainer.prepScoreCoralL4(),
                      robotContainer.scoreCoral(),
                      Commands.waitSeconds(scoringTime),
                      AutoBuilder.followPath(B_CSP),
                      robotContainer.prepCoralIntake(),
                      robotContainer.coralHandoff())
                  .withName("CSP to B"));

    } catch (Exception e) {
      DriverStation.reportError("Path Not Found: " + e.getMessage(), e.getStackTrace());
    }
  }
}
