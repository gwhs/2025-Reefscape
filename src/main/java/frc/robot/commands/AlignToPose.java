package frc.robot.commands;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import dev.doglog.DogLog;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import java.util.function.Supplier;

public class AlignToPose extends Command {
  Pose2d currPose;
  double angularVelocity;
  DriveCommand driveCommand;
  Supplier<Pose2d> targetPose;
  private PIDController PIDX;
  private PIDController PIDY;
  private PIDController PIDRotation;
  private CommandSwerveDrivetrain drivetrain;
  private double MaxSpeed =
      TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12VoltsMps desired top speed
  private double MaxAngularRate =
      3.5 * Math.PI; // 3/4 of a rotation per second max angular velocity
  private final SwerveRequest.FieldCentric drive =
      new SwerveRequest.FieldCentric()
          .withDeadband(MaxSpeed * 0.0)
          .withRotationalDeadband(MaxAngularRate * 0.0) // Add a 10% deadband
          .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // I want field-centric

  public AlignToPose(Supplier<Pose2d> Pose, CommandSwerveDrivetrain drivetrain) {
    addRequirements(drivetrain);
    targetPose = Pose;
    PIDX = new PIDController(2.7, 0, 0); // same for now tune later
    PIDX.setTolerance(0.1);
    PIDY = new PIDController(2.7, 0, 0);
    PIDY.setTolerance(0.12);
    PIDRotation = new PIDController(0.05, 0, 0);
    PIDRotation.setTolerance(0.1);
    PIDRotation.enableContinuousInput(-180, 180);
    this.drivetrain = drivetrain;
  }

  public void goToPoseWithPID(Pose2d targetPose) {
    PIDX.setSetpoint(targetPose.getX());
    PIDY.setSetpoint(targetPose.getY());
    PIDRotation.setSetpoint(targetPose.getRotation().getDegrees());
  }

  public boolean isAtTargetPose() {
    boolean isatX = PIDX.atSetpoint();
    boolean isatY = PIDY.atSetpoint();
    boolean isatRotation = PIDRotation.atSetpoint();
    DogLog.log("atX", isatX);
    DogLog.log("atY", isatY);
    DogLog.log("atRotation", isatRotation);

    if (isatX && isatY && isatRotation) {return true;}
    return false;
  }

  @Override
  public void initialize() {
    goToPoseWithPID(targetPose.get());
  }

  @Override
  public void execute() {
    double xVelocity = 0;
    double yVelocity = 0;
    double angularVelocity = 0;
    currPose = drivetrain.getState().Pose;
    double currX = currPose.getX();
    double currY = currPose.getY();
    Double currRotation = currPose.getRotation().getDegrees();
    double PIDXOutput = PIDX.calculate(currX);
    xVelocity -= PIDXOutput;
    DogLog.log("PIDXOutput", PIDXOutput);
    double PIDYOutput = PIDY.calculate(currY);
    yVelocity -= PIDYOutput;
    DogLog.log("PIDYoutput", PIDYOutput);
    double PIDRotationOutput = PIDRotation.calculate(currRotation);
    angularVelocity += PIDRotationOutput;
    DogLog.log("PIDRotationoutput", PIDRotationOutput);
    xVelocity = xVelocity * MaxSpeed;
    yVelocity = yVelocity * MaxSpeed;
    angularVelocity = angularVelocity * MaxAngularRate;
    DogLog.log("Drive Command/xVelocity", xVelocity);
    DogLog.log("Drive Command/yVelocity", yVelocity);
    DogLog.log("Drive Command/angularVelocity", angularVelocity);
    drivetrain.setControl(
        drive
            .withVelocityX(xVelocity) // Drive forward with negative Y (forward)
            .withVelocityY(yVelocity) // Drive left with negative X (left)
            .withRotationalRate(angularVelocity)); // Drive counterclockwise with negative X (left)
  }

  @Override
  public void end(boolean interrupted) {
    drivetrain.setControl(drive.withVelocityX(0.00).withVelocityY(0.00).withRotationalRate(0.00));
  }

  @Override
  public boolean isFinished() {
    if (isAtTargetPose()) {
      return true;
    }
    return false;
  }
}
