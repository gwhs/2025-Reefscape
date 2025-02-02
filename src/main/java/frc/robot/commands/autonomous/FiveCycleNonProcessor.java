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

public class FiveCycleNonProcessor extends PathPlannerAuto {
  public FiveCycleNonProcessor(RobotContainer robotContainer, boolean partTwoOnly) {
    super(Commands.run(() -> {}));

    double waitTime = 0.5;

    try {
      PathPlannerPath SL_I = PathPlannerPath.fromPathFile("(5CC1) SL-I");
      PathPlannerPath I_CS = PathPlannerPath.fromPathFile("(5CC1) I-CoralStation");
      PathPlannerPath CS_J = PathPlannerPath.fromPathFile("(5CC1) CoralStation-J");
      PathPlannerPath J_CS = PathPlannerPath.fromPathFile("(5CC1) J-CoralStation");
      PathPlannerPath CS_K = PathPlannerPath.fromPathFile("(5CC1) CoralStation-K");
      PathPlannerPath K_CS = PathPlannerPath.fromPathFile("(5CC1) K-CoralStation");
      PathPlannerPath CS_L = PathPlannerPath.fromPathFile("(5CC1) CoralStation-L");
      PathPlannerPath L_CS = PathPlannerPath.fromPathFile("(5CC1) L-CoralStation");
      PathPlannerPath CS_A = PathPlannerPath.fromPathFile("(5CC1) CoralStation-A");

      isRunning()
          .and(new Trigger(() -> !partTwoOnly))
          .onTrue(
              Commands.sequence(
                      AutoBuilder.resetOdom(
                          new Pose2d(
                              SL_I.getPoint(0).position, SL_I.getIdealStartingState().rotation())),
                      AutoBuilder.followPath(SL_I))
                  .withName("Leave SL, score preload at I"));

      event("atI")
          .or(isRunning().and(new Trigger(() -> partTwoOnly)))
          .onTrue(
              Commands.sequence(
                      AutoBuilder.resetOdom(
                          new Pose2d(
                              I_CS.getPoint(0).position, I_CS.getIdealStartingState().rotation())),
                      AutoBuilder.followPath(I_CS))
                  .withName("I to Coral Station"));

      event("atCS")
          .onTrue(
              Commands.sequence(
                      Commands.waitSeconds(waitTime),
                      AutoBuilder.followPath(CS_J),
                      AutoBuilder.followPath(J_CS),
                      Commands.waitSeconds(waitTime),
                      AutoBuilder.followPath(CS_K),
                      AutoBuilder.followPath(K_CS),
                      Commands.waitSeconds(waitTime),
                      AutoBuilder.followPath(CS_L),
                      AutoBuilder.followPath(L_CS),
                      Commands.waitSeconds(waitTime),
                      AutoBuilder.followPath(CS_A),
                      Commands.waitSeconds(waitTime))
                  .withName("CS to J to CS, CS to K to CS, CS to L to CS, CS to A"));

    } catch (Exception e) {
      DriverStation.reportError("Path Not Found: " + e.getMessage(), e.getStackTrace());
    }
  }
}
