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

public class autonC5_1 extends PathPlannerAuto {
  public autonC5_1(RobotContainer robotContainer) {
    super(Commands.run(() -> {}));

    /* All your code should go inside this try-catch block */
    try {
      /* TODO: Load all paths needed */
      PathPlannerPath SC_F = PathPlannerPath.fromPathFile("SC-F");
      double waitTime = 0.1;
      double scoringTime = 0.3;

      /* TODO: Get starting position of starting path */
      Pose2d startingPose =
          new Pose2d(SC_F.getPoint(0).position, SC_F.getIdealStartingState().rotation());

      /* TODO: When autonomous begins */
      isRunning()
          .onTrue(
              Commands.sequence(AutoBuilder.resetOdom(startingPose), 
              AutoBuilder.followPath(SC_F), 
              Commands.waitSeconds(scoringTime),
              Commands.runOnce(() -> new autonC5_2(robotContainer).schedule()))
              // waiting for preload for scoring 
                  // TODO: Name of command
                  .withName("Leave SC to score preload at F"));

      /* TODO: Other triggers */

    } catch (Exception e) {
      DriverStation.reportError("Path Not Found: " + e.getMessage(), e.getStackTrace());
    }
  }
}
