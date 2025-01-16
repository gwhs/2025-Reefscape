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

public class autonC5_4 extends PathPlannerAuto {
  public autonC5_4(RobotContainer robotContainer) {
    super(Commands.run(() -> {}));

    /* All your code should go inside this try-catch block */
    try {
      /* TODO: Load all paths needed */
      PathPlannerPath S3Leave = PathPlannerPath.fromPathFile("S3-C5");

      /* TODO: Get starting position of starting path */
      Pose2d startingPose =
          new Pose2d(S3Leave.getPoint(0).position, S3Leave.getIdealStartingState().rotation());

      /* TODO: When autonomous begins */
      isRunning().onTrue(Commands.sequence(AutoBuilder.resetOdom(startingPose)));

      /* TODO: Other triggers */

    } catch (Exception e) {
      DriverStation.reportError("Path Not Found: " + e.getMessage(), e.getStackTrace());
    }
  }
}
