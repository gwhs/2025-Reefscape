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

public class FiveCycleNonProcessor2 extends PathPlannerAuto {

  public FiveCycleNonProcessor2(RobotContainer robotContainer) {
    super(Commands.run(() -> {}));

    try {
      PathPlannerPath I_CS = PathPlannerPath.fromPathFile("(5CC1) I-CoralStation");
      PathPlannerPath CS_J = PathPlannerPath.fromPathFile("(5CC1) CoralStation-J");
      PathPlannerPath J_CS = PathPlannerPath.fromPathFile("(5CC1) J-CoralStation");
      PathPlannerPath CS_K = PathPlannerPath.fromPathFile("(5CC1) CoralStation-K");
      PathPlannerPath K_CS = PathPlannerPath.fromPathFile("(5CC1) K-CoralStation");
      PathPlannerPath CS_L = PathPlannerPath.fromPathFile("(5CC1) CoralStation-L");
      PathPlannerPath L_CS = PathPlannerPath.fromPathFile("(5CC1) L-CoralStation");
      PathPlannerPath CS_A = PathPlannerPath.fromPathFile("(5CC1) CoralStation-A");

      double waitTime = 0.5;

      Pose2d startingPose =
          new Pose2d(I_CS.getPoint(0).position, I_CS.getIdealStartingState().rotation());

      isRunning()
          .onTrue(
              Commands.sequence(
                      AutoBuilder.resetOdom(startingPose),
                      AutoBuilder.followPath(I_CS).alongWith(robotContainer.prepCoralIntake()))
                  .withName("I to Coral Station"));

      event("atCS") // 0, 0 values are supposed to be L4
          .onTrue(
              Commands.sequence(
                      Commands.waitSeconds(waitTime),
                      AutoBuilder.followPath(CS_J),
                      robotContainer.prepScoreCoral(
                          ElevatorConstants.L4_PREP_POSITION, ArmConstants.L4_PREP_POSITION),
                      robotContainer.scoreCoral(),
                      AutoBuilder.followPath(J_CS).alongWith(robotContainer.prepCoralIntake()),
                      Commands.waitSeconds(waitTime),
                      AutoBuilder.followPath(CS_K),
                      robotContainer.prepScoreCoral(
                          ElevatorConstants.L4_PREP_POSITION, ArmConstants.L4_PREP_POSITION),
                      robotContainer.scoreCoral(),
                      AutoBuilder.followPath(K_CS).alongWith(robotContainer.prepCoralIntake()),
                      Commands.waitSeconds(waitTime),
                      AutoBuilder.followPath(CS_L),
                      robotContainer.prepScoreCoral(
                          ElevatorConstants.L4_PREP_POSITION, ArmConstants.L4_PREP_POSITION),
                      robotContainer.scoreCoral(),
                      AutoBuilder.followPath(L_CS).alongWith(robotContainer.prepCoralIntake()),
                      Commands.waitSeconds(waitTime),
                      AutoBuilder.followPath(CS_A),
                      robotContainer.prepScoreCoral(
                          ElevatorConstants.L4_PREP_POSITION, ArmConstants.L4_PREP_POSITION),
                      robotContainer.scoreCoral())
                  .withName("CS to J to CS, CS to K to CS, CS to L to CS, CS to A"));

    } catch (Exception e) {
      DriverStation.reportError("Path Not Found: " + e.getMessage(), e.getStackTrace());
    }
  }
}
