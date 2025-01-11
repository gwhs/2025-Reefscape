package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;

public class AlignToPose extends Command {
  DriveCommand driveCommand;
  Pose2d targetPose;

  public AlignToPose(Pose2d Pose, DriveCommand driveCommand) {
    targetPose = Pose;
    this.driveCommand = driveCommand;
  }

  @Override
  public void initialize() {
    driveCommand.setIsAligningToPose(true);
    driveCommand.goToPoseWithPID(targetPose);
  }

  @Override
  public void execute() {
    // drive command is doing the real work
  }

  @Override
  public void end(boolean interrupted) {
    driveCommand.setIsAligningToPose(false);
  }

  @Override
  public boolean isFinished() {
    if (driveCommand.isAtTargetPose() == true) {
      return true;
    }
    return false;
  }
}
