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

public class SC_preloadScore extends PathPlannerAuto {
  public SC_preloadScore(RobotContainer robotContainer) {
    super(Commands.run(() -> {}));

    /* All your code should go inside this try-catch block */
    try {
      /* TODO: Load all paths needed */
      PathPlannerPath SCpreloadScore = PathPlannerPath.fromPathFile("SC-preload");

      /* TODO: Get starting position of starting path */
      Pose2d startingPose =
          new Pose2d(SCpreloadScore.getPoint(0).position, SCpreloadScore.getIdealStartingState().rotation());

      /* TODO: When autonomous begins */
      isRunning().onTrue(Commands.sequence(AutoBuilder.resetOdom(startingPose)));

      /* TODO: Other triggers */

      event("atReef").onTrue(
        Commands.sequence(            
            AutoBuilder.resetOdom(startingPose), AutoBuilder.followPath(SCpreloadScore))
            // TODO: Name of command
            .withName("Leave and acore preload coral")
            );

    } catch (Exception e) {
      DriverStation.reportError("Path Not Found: " + e.getMessage(), e.getStackTrace());
    }
  }
}
