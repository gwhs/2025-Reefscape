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
import frc.robot.subsystems.arm.ArmConstants;
import frc.robot.subsystems.elevator.ElevatorConstants;

public class FiveCycleProcessor extends PathPlannerAuto {
  public FiveCycleProcessor(RobotContainer robotContainer) {
    super(Commands.run(() -> {}));

    try {
      PathPlannerPath SC_F = PathPlannerPath.fromPathFile("SC-F");

      Pose2d startingPose =
          new Pose2d(SC_F.getPoint(0).position, SC_F.getIdealStartingState().rotation());

      isRunning()
          .onTrue(
              Commands.sequence(
                      AutoBuilder.resetOdom(startingPose),
                      AutoBuilder.followPath(SC_F),
                      robotContainer.prepScoreCoral(ElevatorConstants.L4_PREP_POSITION,ArmConstants.L4_PREP_POSITION),
                      robotContainer.scoreCoral(),
                      Commands.runOnce(() -> new FiveCycleProcessor2(robotContainer).schedule()))
                  .withName("Leave SC to score preload at F"));

    } catch (Exception e) {
      DriverStation.reportError("Path Not Found: " + e.getMessage(), e.getStackTrace());
    }
  }
}
