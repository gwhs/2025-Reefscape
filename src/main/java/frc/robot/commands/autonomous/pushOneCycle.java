package frc.robot.commands.autonomous;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.EagleUtil;
import frc.robot.RobotContainer;
import frc.robot.subsystems.arm.ArmConstants;
import frc.robot.subsystems.elevator.ElevatorConstants;

public class PushOneCycle extends PathPlannerAuto {
  public PushOneCycle(RobotContainer robotContainer) {
    super(Commands.run(() -> {}));

    /* All your code should go inside this try-catch block */
    try {

      PathPlannerPath startLn_H = PathPlannerPath.fromPathFile("(P1C) Startline-H");

      Pose2d startingPose =
          new Pose2d(startLn_H.getPoint(0).position, startLn_H.getIdealStartingState().rotation());

      isRunning()
          .onTrue(
              Commands.sequence(
                      AutoBuilder.resetOdom(startingPose),
                      AutoBuilder.followPath(startLn_H),
                      robotContainer.prepScoreCoral(
                          RobotContainer.CoralLevel.L4).deadlineFor(
                            robotContainer.alignToPose(
                                () -> EagleUtil.getCachedReefPose(robotContainer.getRobotPose()))),
                      robotContainer.scoreCoral())
                  .withName("Leave Startline (Push) and score L1 at H"));

    } catch (Exception e) {
      DriverStation.reportError("Path Not Found: " + e.getMessage(), e.getStackTrace());
    }
  }
}
