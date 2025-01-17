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

public class auton_2_cycle2 extends PathPlannerAuto {
  public auton_2_cycle2(RobotContainer robotContainer) {
    super(Commands.run(() -> {}));

    /* All your code should go inside this try-catch block */
    try {
      /* TODO: Load all paths needed */
      PathPlannerPath E_CSP = PathPlannerPath.fromPathFile("E-CSP");
      PathPlannerPath CSP_D = PathPlannerPath.fromPathFile("CSP-D");
      PathPlannerPath D_CSP = PathPlannerPath.fromPathFile("D-CSP");
      double waitTime = 0.5;

      /* TODO: Get starting position of starting path */
      Pose2d startingPose =
          new Pose2d(E_CSP.getPoint(0).position, E_CSP.getIdealStartingState().rotation());

      /* TODO: When autonomous begins */
      isRunning()
          .onTrue(
              Commands.sequence(AutoBuilder.resetOdom(startingPose), AutoBuilder.followPath(E_CSP))

                  // TODO: Name of command
                  .withName("E to CSP"));

      /* TODO: Other triggers */
      event("atCSP")
          .onTrue(
              Commands.sequence(
                  Commands.waitSeconds(waitTime),
                  AutoBuilder.followPath(CSP_D),
                  AutoBuilder.followPath(D_CSP),
                  Commands.waitSeconds(waitTime).withName("CSP to D")));

    } catch (Exception e) {
      DriverStation.reportError("Path Not Found: " + e.getMessage(), e.getStackTrace());
    }
  }
}
