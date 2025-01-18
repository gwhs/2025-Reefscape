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
      PathPlannerPath D_CSP = PathPlannerPath.fromPathFile("D-CSP");
      PathPlannerPath CSP_C = PathPlannerPath.fromPathFile("CSP-C");

      /* TODO: Get starting position of starting path */
      Pose2d startingPose =
          new Pose2d(D_CSP.getPoint(0).position, D_CSP.getIdealStartingState().rotation());

      /* TODO: When autonomous begins */
      isRunning().onTrue(Commands.sequence(
                      AutoBuilder.resetOdom(startingPose), AutoBuilder.followPath(D_CSP))
                  // TODO: Name of command
                  .withName("D to CSP"));

      /* TODO: Other triggers */
    event("atCSP").onTrue(
        Commands.sequence(
        AutoBuilder.followPath(CSP_C)
        .withName("CSP to C")));

      /* TODO: Other triggers */

    } catch (Exception e) {
      DriverStation.reportError("Path Not Found: " + e.getMessage(), e.getStackTrace());
    }
  }
}
