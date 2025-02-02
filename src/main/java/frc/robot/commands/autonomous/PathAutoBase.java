package frc.robot.commands.autonomous;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotContainer;

public class PathAutoBase extends PathPlannerAuto {
  public PathAutoBase() {
    super(Commands.run(() -> {}));
  }

  protected Pose2d getStartingPose(PathPlannerPath thePath) {
    return new Pose2d(thePath.getPoint(0).position, thePath.getIdealStartingState().rotation());
  }

  protected PathAutoBase sequencePathThenCommand(
      PathPlannerPath thePath, double scoringTime, Command then, String name) {
    isRunning()
        .onTrue(
            Commands.sequence(
                    AutoBuilder.resetOdom(getStartingPose(thePath)),
                    AutoBuilder.followPath(thePath),
                    Commands.waitSeconds(scoringTime),
                    Commands.runOnce(() -> then.schedule()))
                .withName(name));
    return this;
  }

  protected static PathAutoBase sequencePathThenCommand(
      String pathname, double scoringTime, Command then, String name) {
    PathAutoBase pathAuto = new PathAutoBase();
    try {
      PathPlannerPath thePath = PathPlannerPath.fromPathFile(pathname);
      return pathAuto.sequencePathThenCommand(thePath, scoringTime, then, name);
    } catch (Exception e) {
      DriverStation.reportError("Path Not Found: " + e.getMessage(), e.getStackTrace());
    }
    return pathAuto;
  }

  public static PathPlannerAuto fiveCycleNonProcessor(RobotContainer robotContainer) {
    return sequencePathThenCommand(
        "(5CC1) SL-I",
        0,
        new FiveCycleNonProcessor2(robotContainer),
        "Leave SL, score preload at I");
  }

  public static PathPlannerAuto fiveCycleProcessor(RobotContainer robotContainer) {
    return sequencePathThenCommand(
        "SC-F", 0.3, new FiveCycleProcessor2(robotContainer), "Leave SC to score preload at F");
  }

  public static PathPlannerAuto leaveNonProcessor(RobotContainer robotContainer) {
    return sequencePathThenCommand("Startline-Leave", 0, Commands.none(), "Leave Starting Line");
  }

  public static PathPlannerAuto leaveProcessor(RobotContainer robotContainer) {
    return sequencePathThenCommand("Startline-Leave2", 0, Commands.none(), "Leave Starting Line2");
  }
}
