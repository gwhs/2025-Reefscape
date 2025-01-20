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
  private PIDController PIDX;
  private PIDController PIDY;
  private PIDController PIDRotation;

  private CommandSwerveDrivetrain drivetrain;

  private double maxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
  private double maxAngularRate = 3.5 * Math.PI;

  public static final double PID_MAX = 0.35;

  private final SwerveRequest.FieldCentric drive =
      new SwerveRequest.FieldCentric()
          .withDeadband(maxSpeed * 0.0)
          .withRotationalDeadband(maxAngularRate * 0.0)
          .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

  public AlignToPose(Supplier<Pose2d> Pose, CommandSwerveDrivetrain drivetrain) {
    addRequirements(drivetrain);
    targetPose = Pose;

    PIDX = new PIDController(2.7, 0, 0); // same for now tune later
    PIDX.setTolerance(0.1);

    PIDY = new PIDController(2.7, 0, 0);
    PIDY.setTolerance(0.1);

    PIDRotation = new PIDController(0.1, 0, 0);
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
    DogLog.log("Align/atX", isatX);
    DogLog.log("Align/atY", isatY);
    DogLog.log("Align/atRotation", isatRotation);

    if (isatX && isatY && isatRotation) {
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

    double PIDXOutput = PIDX.calculate(currX);
    double xVelocity = -PIDXOutput;
    DogLog.log("Align/PIDXOutput", PIDXOutput);

    double PIDYOutput = PIDY.calculate(currY);
    double yVelocity = -PIDYOutput;
    DogLog.log("Align/PIDYoutput", PIDYOutput);

    double PIDRotationOutput =
        MathUtil.clamp(PIDRotation.calculate(currRotation), -PID_MAX, PID_MAX);
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
