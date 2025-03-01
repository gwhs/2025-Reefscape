// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.EagleUtil;
import frc.robot.RobotContainer;
import frc.robot.subsystems.arm.ArmConstants;
import frc.robot.subsystems.elevator.ElevatorConstants;

public class FiveCycleProcessor extends PathPlannerAuto {

  private RobotContainer robotContainer;
  private double waitTime = .4;

  public FiveCycleProcessor(RobotContainer robotContainer) {
    super(Commands.run(() -> {}));

    this.robotContainer = robotContainer;

    try {
      PathPlannerPath SC_F = PathPlannerPath.fromPathFile("SC-F");
      PathPlannerPath F_CSP = PathPlannerPath.fromPathFile("F-CSP");
      PathPlannerPath CSP_E = PathPlannerPath.fromPathFile("CSP-E");
      PathPlannerPath E_CSP = PathPlannerPath.fromPathFile("E-CSP");
      PathPlannerPath CSP_D = PathPlannerPath.fromPathFile("CSP-D");
      PathPlannerPath D_CSP = PathPlannerPath.fromPathFile("D-CSP");
      PathPlannerPath CSP_C = PathPlannerPath.fromPathFile("CSP-C");
      PathPlannerPath C_CSP = PathPlannerPath.fromPathFile("C-CSP");
      PathPlannerPath CSP_B = PathPlannerPath.fromPathFile("CSP-B");
      PathPlannerPath B_CSP = PathPlannerPath.fromPathFile("B-CSP");

      Pose2d startingPose =
          new Pose2d(SC_F.getPoint(0).position, SC_F.getIdealStartingState().rotation());

      isRunning()
          .onTrue(
              Commands.sequence(
                  AutoBuilder.resetOdom(startingPose).onlyIf(() -> RobotBase.isSimulation()),
                  AutoBuilder.followPath(SC_F)
                      .deadlineFor(
                          Commands.sequence(
                              robotContainer.zeroElevator().onlyIf(() -> RobotBase.isReal()),
                              robotContainer.prepScoreCoral(
                                  ElevatorConstants.L4_PREP_POSITION,
                                  ArmConstants.L4_PREP_POSITION))),
                  robotContainer
                      .prepScoreCoral(
                          ElevatorConstants.L4_PREP_POSITION, ArmConstants.L4_PREP_POSITION)
                      .deadlineFor(
                          robotContainer.alignToPose(
                              () -> EagleUtil.getCachedReefPose(robotContainer.getRobotPose()))),
                  robotContainer.scoreCoral(),
                  AutoBuilder.followPath(F_CSP).alongWith(robotContainer.prepCoralIntake()),
                  autoHelper(CSP_E, E_CSP),
                  autoHelper(CSP_D, D_CSP) /*,
                  autoHelper(CSP_C, C_CSP)*/));

    } catch (Exception e) {
      DriverStation.reportError("Path Not Found: " + e.getMessage(), e.getStackTrace());
    }
  }

  public Command autoHelper(PathPlannerPath pathOne, PathPlannerPath pathTwo) {
    return Commands.sequence(
        Commands.waitSeconds(waitTime),
        AutoBuilder.followPath(pathOne)
            .deadlineFor(
                Commands.sequence(
                    Commands.waitSeconds(1.5),
                    robotContainer.prepScoreCoral(
                        ElevatorConstants.L4_PREP_POSITION, ArmConstants.L4_PREP_POSITION))),
        robotContainer
            .prepScoreCoral(ElevatorConstants.L4_PREP_POSITION, ArmConstants.L4_PREP_POSITION)
            .deadlineFor(
                robotContainer.alignToPose(
                    () -> EagleUtil.getCachedReefPose(robotContainer.getRobotPose()))),
        robotContainer.scoreCoral(),
        AutoBuilder.followPath(pathTwo).alongWith(robotContainer.prepCoralIntake()));
  }
}
