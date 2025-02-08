package frc.robot.commands;

import static edu.wpi.first.units.Units.MetersPerSecond;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import dev.doglog.DogLog;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class AlignToPose extends Command {
  Supplier<Pose2d> targetPose;
  private PIDController PID_X;
  private PIDController PID_Y;
  private PIDController PID_Rotation;

  private final double ELEVATOR_UP_SLEW_RATE = 0.5;

  private final SlewRateLimiter angularVelocityLimiter = new SlewRateLimiter(ELEVATOR_UP_SLEW_RATE);
  private final SlewRateLimiter xVelocityLimiter = new SlewRateLimiter(ELEVATOR_UP_SLEW_RATE) ;
  private final SlewRateLimiter yVelocityLimiter = new SlewRateLimiter(ELEVATOR_UP_SLEW_RATE);
  private final DoubleSupplier elevatorHeight;

  private boolean resetLimiter = true;

  private CommandSwerveDrivetrain drivetrain;

  private double maxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
  private double maxAngularRate = 1.0 * Math.PI;

  public static final double PID_MAX = 0.44;

  private final SwerveRequest.FieldCentric drive =
      new SwerveRequest.FieldCentric()
          .withDeadband(maxSpeed * 0.05)
          .withRotationalDeadband(maxAngularRate * 0.05)
          .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

  public AlignToPose(Supplier<Pose2d> Pose, CommandSwerveDrivetrain drivetrain, DoubleSupplier elevatorHeight) {
    addRequirements(drivetrain);

    this.drivetrain = drivetrain;
    this.targetPose = Pose;
    this.elevatorHeight = elevatorHeight;

    PID_X = new PIDController(2.7, 0, 0); // same for now tune later
    PID_X.setTolerance(0.02);

    PID_Y = new PIDController(2.7, 0, 0);
    PID_Y.setTolerance(0.02);

    PID_Rotation = new PIDController(0.05, 0, 0);
    PID_Rotation.setTolerance(0.5);
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

    double PIDXOutput = MathUtil.clamp(PID_X.calculate(currX), -PID_MAX, PID_MAX);
    double xVelocity = -PIDXOutput;
    DogLog.log("Align/PIDXOutput", PIDXOutput);

    double PIDYOutput = MathUtil.clamp(PID_Y.calculate(currY), -PID_MAX, PID_MAX);
    double yVelocity = -PIDYOutput;
    DogLog.log("Align/PIDYoutput", PIDYOutput);

    double PIDRotationOutput =
        MathUtil.clamp(PID_Rotation.calculate(currRotation), -PID_MAX, PID_MAX);
    double angularVelocity = PIDRotationOutput;
    DogLog.log("Align/PIDRotationoutput", PIDRotationOutput);

    if (elevatorHeight.getAsDouble() > 0.3) {
      if (resetLimiter) {
        resetLimiter = false;
        xVelocityLimiter.reset(xVelocity);
        yVelocityLimiter.reset(yVelocity);
        angularVelocityLimiter.reset(angularVelocity);
      }
      xVelocity = xVelocityLimiter.calculate(xVelocity);
      yVelocity = yVelocityLimiter.calculate(yVelocity);
      angularVelocity = angularVelocityLimiter.calculate(angularVelocity);
    } else {
      resetLimiter = true;
    }

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
