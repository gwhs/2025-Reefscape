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

public class Drivetrain_Practice extends PathPlannerAuto {
  public Drivetrain_Practice(RobotContainer robotContainer) {
    super(Commands.run(() -> {}));

    try {

      PathPlannerPath TestPath = PathPlannerPath.fromPathFile("TestPath");

      Pose2d startingPose =
          new Pose2d(TestPath.getPoint(0).position, TestPath.getIdealStartingState().rotation());

      isRunning()
          .onTrue(
              Commands.sequence(
                  AutoBuilder.resetOdom(startingPose), AutoBuilder.followPath(TestPath)));

    } catch (Exception e) {
      DriverStation.reportError("Path Not Found: " + e.getMessage(), e.getStackTrace());
    }
  }
}
