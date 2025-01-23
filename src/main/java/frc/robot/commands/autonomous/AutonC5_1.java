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

public class AutonC5_1 extends PathPlannerAuto {
  public AutonC5_1(RobotContainer robotContainer) {
    super(Commands.run(() -> {}));

    /* All your code should go inside this try-catch block */
    try {

      PathPlannerPath SC_F = PathPlannerPath.fromPathFile("SC-F");
      // double waitTime = 0.1; never used?
      double scoringTime = 0.3;

      Pose2d startingPose =
          new Pose2d(SC_F.getPoint(0).position, SC_F.getIdealStartingState().rotation());

      isRunning()
          .onTrue(
              Commands.sequence(
                      AutoBuilder.resetOdom(startingPose),
                      AutoBuilder.followPath(SC_F),
                      Commands.waitSeconds(scoringTime),
                      Commands.runOnce(() -> new AutonC5_2(robotContainer).schedule()))
                  .withName("Leave SC to score preload at F"));

      /* TODO: Other triggers */

    } catch (Exception e) {
      DriverStation.reportError("Path Not Found: " + e.getMessage(), e.getStackTrace());
    }
  }
}
