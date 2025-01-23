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

public class Five_Cycle_Non_Processor extends PathPlannerAuto {
  public Five_Cycle_Non_Processor(RobotContainer robotContainer) {
    super(Commands.run(() -> {}));}

    try {

      PathPlannerPath SL_I = PathPlannerPath.fromPathFile("(5CC1) SL-I");

      Pose2d startingPose =
          new Pose2d(SL_I.getPoint(0).position, SL_I.getIdealStartingState().rotation());

      isRunning()
          .onTrue(
              Commands.sequence(AutoBuilder.resetOdom(startingPose), AutoBuilder.followPath(SL_I))
                  .withName("Leave SL, score preload at I"));

    } catch (Exception e) {
      DriverStation.reportError("Path Not Found: " + e.getMessage(), e.getStackTrace());
    }
  }

