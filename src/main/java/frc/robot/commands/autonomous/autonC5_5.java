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

public class autonC5_5 extends PathPlannerAuto {
  public autonC5_5(RobotContainer robotContainer) {
    super(Commands.run(() -> {}));

    /* All your code should go inside this try-catch block */
    try {
      /* TODO: Load all paths needed */
      PathPlannerPath C_CSP = PathPlannerPath.fromPathFile("C-CSP");
      PathPlannerPath CSP_B = PathPlannerPath.fromPathFile("CSP-B");
      PathPlannerPath B_CSP = PathPlannerPath.fromPathFile("B-CSP");

      /* TODO: Get starting position of starting path */
      Pose2d startingPose =
          new Pose2d(C_CSP.getPoint(0).position, C_CSP.getIdealStartingState().rotation());

      /* TODO: When autonomous begins */
      isRunning().onTrue(Commands.sequence(
                      AutoBuilder.resetOdom(startingPose), AutoBuilder.followPath(C_CSP))
                  // TODO: Name of command
                  .withName("C to CSP"));

      /* TODO: Other triggers */
    event("atCSP").onTrue(
        Commands.sequence(
        AutoBuilder.followPath(CSP_B)
        .withName("CSP to B")));

    event("atB").onTrue(
        Commands.sequence(
        AutoBuilder.followPath(B_CSP)
        .withName("B to CSP")));        

      /* TODO: Other triggers */

    } catch (Exception e) {
      DriverStation.reportError("Path Not Found: " + e.getMessage(), e.getStackTrace());
    }
  }
}
