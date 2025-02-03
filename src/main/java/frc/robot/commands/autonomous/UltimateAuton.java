package frc.robot.commands.autonomous;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotContainer;
import frc.robot.subsystems.elevator.ElevatorSubsystem;

public class UltimateAuton extends PathPlannerAuto {

  public enum Reef {
    A(null, AutonPaths.CS_A, AutonPaths.CS_A),
    B(null, AutonPaths.B_CSP, AutonPaths.CSP_B),
    C(null, AutonPaths.C_CSP, AutonPaths.CSP_C),
    D(null, AutonPaths.D_CSP, AutonPaths.CSP_D),
    E(null, AutonPaths.E_CSP, AutonPaths.CSP_E),
    F(AutonPaths.SC_F, AutonPaths.F_CSP, null),
    G(null, null, null),
    H(null, null, null),
    I(AutonPaths.SL_I, AutonPaths.I_CS, null),
    J(null, AutonPaths.J_CS, AutonPaths.CS_J),
    K(null, AutonPaths.K_CS, AutonPaths.CS_K),
    L(null, AutonPaths.L_CS, AutonPaths.CS_L);

    public PathPlannerPath pathFromStartingLine;
    public PathPlannerPath pathToCoralStation;
    public PathPlannerPath pathToReef;

    Reef(
        PathPlannerPath pathFromStartingLine,
        PathPlannerPath pathToCoralStation,
        PathPlannerPath pathToReef) {
      this.pathFromStartingLine = pathFromStartingLine;
      this.pathToCoralStation = pathToCoralStation;
      this.pathToReef = pathToReef;
    }
  }

  double waitTime = 0.1;

  public UltimateAuton(RobotContainer robotContainer, boolean fromStatingLine, Reef... reefs) {
    super(Commands.run(() -> {}));

    if (reefs.length == 0) return;

    PathPlannerPath startingPath =
        fromStatingLine ? reefs[0].pathFromStartingLine : reefs[0].pathToCoralStation;
    Pose2d startingPose = startingPath.getStartingHolonomicPose().get();

    isRunning()
        .onTrue(
            Commands.sequence(
                    AutoBuilder.resetOdom(startingPose),
                    Commands.parallel(
                            AutoBuilder.followPath(startingPath),
                            robotContainer.prepScoreCoral(
                                ElevatorSubsystem.rotationsToMeters(57), 210))
                        .onlyIf(() -> fromStatingLine),
                    AutoBuilder.followPath(reefs[0].pathToCoralStation)
                        .alongWith(robotContainer.prepCoralIntake()))
                .withName("Leave Starting line, score reef, go to coral station"));

    for (int i = 1; i < reefs.length; i++) {
      PathPlannerPath prevPath = reefs[i - 1].pathToCoralStation;
      activePath(prevPath.name)
          .onFalse(
              Commands.sequence(
                      Commands.waitSeconds(waitTime),
                      Commands.parallel(
                          AutoBuilder.followPath(reefs[i].pathToReef),
                          robotContainer.prepScoreCoral(
                              ElevatorSubsystem.rotationsToMeters(57), 210)),
                      robotContainer.scoreCoral().withTimeout(0.5),
                      AutoBuilder.followPath(reefs[i].pathToCoralStation)
                          .alongWith(robotContainer.prepCoralIntake()))
                  .withName("coral station to score reef, back to coral station"));
    }
  }
}
