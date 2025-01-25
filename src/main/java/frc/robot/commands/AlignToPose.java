package frc.robot.commands;

import static edu.wpi.first.units.Units.MetersPerSecond;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import dev.doglog.DogLog;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import java.util.function.Supplier;

public class AlignToPose extends Command {
  Supplier<Pose2d> targetPose;
  private PIDController PID_X;
  private PIDController PID_Y;
  private PIDController PID_Rotation;

  private CommandSwerveDrivetrain drivetrain;

  private double maxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
  private double maxAngularRate = 1.0 * Math.PI;

  public static final double PID_MAX = 0.35;

  private final SwerveRequest.FieldCentric drive =
      new SwerveRequest.FieldCentric()
          .withDeadband(maxSpeed * 0.0)
          .withRotationalDeadband(maxAngularRate * 0.0)
          .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

  public AlignToPose(Supplier<Pose2d> Pose, CommandSwerveDrivetrain drivetrain) {
    addRequirements(drivetrain);

    this.drivetrain = drivetrain;
    this.targetPose = Pose;

    PID_X = new PIDController(2.7, 0, 0); // same for now tune later
    PID_X.setTolerance(0.1);

    PID_Y = new PIDController(2.7, 0, 0);
    PID_Y.setTolerance(0.1);

    PID_Rotation = new PIDController(0.1, 0, 0);
    PID_Rotation.setTolerance(0.1);
    PID_Rotation.enableContinuousInput(-180, 180);
  }

  public void goToPoseWithPID(Pose2d targetPose) {
    PID_X.setSetpoint(targetPose.getX());
    PID_Y.setSetpoint(targetPose.getY());
    PID_Rotation.setSetpoint(targetPose.getRotation().getDegrees());
  }

  public boolean isAtTargetPose() {
    boolean isAtX = PID_X.atSetpoint();
    boolean isAtY = PID_Y.atSetpoint();
    boolean isAtRotation = PID_Rotation.atSetpoint();
    DogLog.log("Align/atX", isAtX);
    DogLog.log("Align/atY", isAtY);
    DogLog.log("Align/atRotation", isAtRotation);

    if (isAtX && isAtY && isAtRotation) {
      return true;
    }
    return false;
  }

  @Override
  public void initialize() {
    goToPoseWithPID(targetPose.get());

    DogLog.log("Align/Target Pose", targetPose.get());
  }

  @Override
  public void execute() {
    Pose2d currPose;
    currPose = drivetrain.getState().Pose;
    double currX = currPose.getX();
    double currY = currPose.getY();
    Double currRotation = currPose.getRotation().getDegrees();

    double PIDXOutput = PID_X.calculate(currX);
    double xVelocity = -PIDXOutput;
    DogLog.log("Align/PIDXOutput", PIDXOutput);

    double PIDYOutput = PID_Y.calculate(currY);
    double yVelocity = -PIDYOutput;
    DogLog.log("Align/PIDYoutput", PIDYOutput);

    double PIDRotationOutput =
        MathUtil.clamp(PID_Rotation.calculate(currRotation), -PID_MAX, PID_MAX);
    double angularVelocity = PIDRotationOutput;

    DogLog.log("Align/PIDRotationoutput", PIDRotationOutput);

    if (DriverStation.getAlliance().get() == DriverStation.Alliance.Blue) {
      xVelocity = -xVelocity * maxSpeed;
      yVelocity = -yVelocity * maxSpeed;
      angularVelocity = angularVelocity * maxAngularRate;
    } else {
      xVelocity = xVelocity * maxSpeed;
      yVelocity = yVelocity * maxSpeed;
      angularVelocity = angularVelocity * maxAngularRate;
    }

    angularVelocity = MathUtil.clamp(angularVelocity, -maxAngularRate, maxAngularRate);
    DogLog.log("Align/xVelocity", xVelocity);
    DogLog.log("Align/yVelocity", yVelocity);
    DogLog.log("Align/angularVelocity", angularVelocity);
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
