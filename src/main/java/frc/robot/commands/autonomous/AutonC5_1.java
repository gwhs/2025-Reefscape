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

<<<<<<<< HEAD:src/main/java/frc/robot/commands/autonomous/Five_Cycle_Processor.java
public class Five_Cycle_Processor extends PathPlannerAuto {
  public Five_Cycle_Processor(RobotContainer robotContainer) {
========
public class AutonC5_1 extends PathPlannerAuto {
  public AutonC5_1(RobotContainer robotContainer) {
>>>>>>>> dev:src/main/java/frc/robot/commands/autonomous/AutonC5_1.java
    super(Commands.run(() -> {}));

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
<<<<<<<< HEAD:src/main/java/frc/robot/commands/autonomous/Five_Cycle_Processor.java
                      Commands.runOnce(() -> new Five_Cycle_Processor(robotContainer).schedule()))
========
                      Commands.runOnce(() -> new AutonC5_2(robotContainer).schedule()))
>>>>>>>> dev:src/main/java/frc/robot/commands/autonomous/AutonC5_1.java
                  .withName("Leave SC to score preload at F"));

    } catch (Exception e) {
      DriverStation.reportError("Path Not Found: " + e.getMessage(), e.getStackTrace());
    }
  }
}
