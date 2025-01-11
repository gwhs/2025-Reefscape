package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.Supplier;

public class AlignToPose extends Command {
  DriveCommand driveCommand;
  Supplier<Pose2d> targetPose;

  public AlignToPose(Supplier<Pose2d> Pose, DriveCommand driveCommand) {
    targetPose = Pose;
    this.driveCommand = driveCommand;
  }

  @Override
  public void initialize() {
    driveCommand.setIsAligningToPose(true);
    driveCommand.goToPoseWithPID(targetPose.get());
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
