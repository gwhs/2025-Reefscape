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

public class Auton_2_Cycle_1 extends PathPlannerAuto {
  public Auton_2_Cycle_1(RobotContainer robotContainer) {
    super(Commands.run(() -> {}));

    /* All your code should go inside this try-catch block */
    try {

      PathPlannerPath SP_E = PathPlannerPath.fromPathFile("SP-E");

      Pose2d startingPose =
          new Pose2d(SP_E.getPoint(0).position, SP_E.getIdealStartingState().rotation());

      isRunning()
          .onTrue(
              Commands.sequence(AutoBuilder.resetOdom(startingPose), AutoBuilder.followPath(SP_E))
                  .withName("Leave SP to score preload at E"));

      /* TODO: Other triggers */

    } catch (Exception e) {
      DriverStation.reportError("Path Not Found: " + e.getMessage(), e.getStackTrace());
    }
  }
}
