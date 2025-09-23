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
import frc.robot.subsystems.climb.ClimbSubsystem;
import frc.robot.subsystems.elevator.ElevatorConstants;
import frc.robot.subsystems.groundIntake.GroundIntakeConstants;
import frc.robot.subsystems.groundIntake.GroundIntakeSubsystem;

public class ChoreoFiveCycle extends PathPlannerAuto {

  private RobotContainer robotContainer;

  private double waitTime = 0;

  public ChoreoFiveCycle(
      RobotContainer robotContainer,
      GroundIntakeSubsystem groundIntakeSubsystem,
      ClimbSubsystem climbSubsystem,
      boolean nonProcessorSide) {
    super(Commands.run(() -> {}));

    this.robotContainer = robotContainer;

    try {
      PathPlannerPath SC_F = PathPlannerPath.fromChoreoTrajectory("SC-F");
      PathPlannerPath F_CSP = PathPlannerPath.fromChoreoTrajectory("F-CSP");
      PathPlannerPath CSP_E = PathPlannerPath.fromChoreoTrajectory("CSP-E");
      PathPlannerPath E_CSP = PathPlannerPath.fromChoreoTrajectory("E-CSP");
      PathPlannerPath CSP_D = PathPlannerPath.fromChoreoTrajectory("CSP-D");
      PathPlannerPath D_CSP = PathPlannerPath.fromChoreoTrajectory("D-CSP");
      PathPlannerPath CSP_C = PathPlannerPath.fromChoreoTrajectory("CSP-C");
      PathPlannerPath C_CSP = PathPlannerPath.fromChoreoTrajectory("C-CSP");
      PathPlannerPath CSP_B = PathPlannerPath.fromChoreoTrajectory("CSP-B");
      PathPlannerPath B_CSP = PathPlannerPath.fromChoreoTrajectory("B-CSP");

      if (nonProcessorSide) {
        SC_F = SC_F.mirrorPath();
        F_CSP = F_CSP.mirrorPath();
        CSP_E = CSP_E.mirrorPath();
        E_CSP = E_CSP.mirrorPath();
        CSP_D = CSP_D.mirrorPath();
        D_CSP = D_CSP.mirrorPath();
        CSP_C = CSP_C.mirrorPath();
        C_CSP = C_CSP.mirrorPath();
        CSP_B = CSP_B.mirrorPath();
        B_CSP = B_CSP.mirrorPath();
      }

      Pose2d startingPose =
          new Pose2d(SC_F.getPoint(0).position, SC_F.getIdealStartingState().rotation());

      isRunning()
          .onTrue(
              Commands.sequence(
                  groundIntakeSubsystem.setAngleAndAmp(
                      GroundIntakeConstants.INTAKE_CORAL_ANGLE, 0, 0),
                  AutoBuilder.resetOdom(startingPose).onlyIf(() -> RobotBase.isSimulation()),
                  AutoBuilder.followPath(SC_F)
                      .deadlineFor(
                          Commands.sequence(
                              climbSubsystem.stow(),
                              robotContainer.zeroElevator().onlyIf(() -> RobotBase.isReal()),
                              robotContainer.prepScoreCoral(
                                  ElevatorConstants.L4_PREP_POSITION,
                                  ArmConstants.L4_PREP_POSITION))),
                  Commands.sequence(
                          Commands.waitSeconds(.1)
                              .deadlineFor(
                                  robotContainer.prepScoreCoral(RobotContainer.CoralLevel.L4)),
                          robotContainer.autonScoreCoral())
                      .deadlineFor(
                          robotContainer.alignToPose(
                              () -> EagleUtil.getCachedReefPose(robotContainer.getRobotPose()))),
                  groundIntakeSubsystem.setAngleAndAmp(
                      GroundIntakeConstants.CORAL_STOW_ANGLE, 0, 0),
                  AutoBuilder.followPath(F_CSP).alongWith(robotContainer.prepCoralIntakeAuton()),
                  autoHelper(CSP_D, D_CSP),
                  autoHelper(CSP_C, C_CSP),
                  autoHelper(CSP_E, E_CSP)));

    } catch (Exception e) {
      DriverStation.reportError("Path Not Found: " + e.getMessage(), e.getStackTrace());
    }
  }

  public Command autoHelper(PathPlannerPath pathOne, PathPlannerPath pathTwo) {

    return Commands.sequence(
        // wait until coral is loaded
        // Commands.waitUntil(robotContainer.IS_CORAL_LOADED),
        // Commands.waitSeconds(waitTime),
        // drive to scoring position
        AutoBuilder.followPath(pathOne)
            .deadlineFor(
                Commands.sequence(
                    Commands.waitSeconds(.3),
                    robotContainer
                        .prepScoreCoral(
                            ElevatorConstants.INTAKE_METER, ArmConstants.L4_PREP_POSITION)
                        .withTimeout(0.02),
                    Commands.waitSeconds(0.4),
                    robotContainer.prepScoreCoral(
                        ElevatorConstants.L4_PREP_POSITION, ArmConstants.L4_PREP_POSITION)))
            .raceWith(
                Commands.waitSeconds(0.5)
                    .andThen(Commands.idle().onlyIf(robotContainer.IS_CORAL_LOADED))),
        Commands.sequence(
                Commands.waitUntil(robotContainer.IS_CORAL_LOADED)
                    .deadlineFor(
                        robotContainer.prepCoralIntakeAuton(),
                        robotContainer.alignToPose(
                            () -> EagleUtil.getClosestCoralStation(robotContainer.getRobotPose()))),
                AutoBuilder.followPath(pathOne)
                    .deadlineFor(
                        Commands.sequence(
                            Commands.waitSeconds(.3),
                            robotContainer
                                .prepScoreCoral(
                                    ElevatorConstants.INTAKE_METER, ArmConstants.L4_PREP_POSITION)
                                .withTimeout(0.02),
                            Commands.waitSeconds(0.5),
                            robotContainer.prepScoreCoral(
                                ElevatorConstants.L4_PREP_POSITION,
                                ArmConstants.L4_PREP_POSITION))))
            .onlyIf(robotContainer.IS_CORAL_LOADED.negate()),
        Commands.sequence(
                Commands.waitSeconds(.05)
                    .deadlineFor(robotContainer.prepScoreCoral(RobotContainer.CoralLevel.L4)),
                robotContainer.autonScoreCoral())
            .deadlineFor(
                robotContainer.alignToPose(
                    () -> EagleUtil.getCachedReefPose(robotContainer.getRobotPose()))),
        AutoBuilder.followPath(pathTwo).alongWith(robotContainer.prepCoralIntakeAuton()));
  }
}
