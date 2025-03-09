package frc.robot.commands;

import static edu.wpi.first.units.Units.MetersPerSecond;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import dev.doglog.DogLog;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class AlignToPose extends Command {

  Supplier<Pose2d> targetPose;
  private final double ELEVATOR_UP_SLEW_RATE = 1;

  private final SlewRateLimiter angularVelocityLimiter = new SlewRateLimiter(ELEVATOR_UP_SLEW_RATE);
  private final SlewRateLimiter xVelocityLimiter = new SlewRateLimiter(ELEVATOR_UP_SLEW_RATE);
  private final SlewRateLimiter yVelocityLimiter = new SlewRateLimiter(ELEVATOR_UP_SLEW_RATE);
  private final DoubleSupplier elevatorHeight;

  private boolean resetLimiter = true;
  private CommandXboxController driverController;

  private CommandSwerveDrivetrain drivetrain;

  private double maxSpeed = CommandSwerveDrivetrain.kSpeedAt12Volts.in(MetersPerSecond);
  private double maxAngularRate = 1.0 * Math.PI;

  public static final double PID_MAX = 0.44;

  private final SwerveRequest.FieldCentric drive =
      new SwerveRequest.FieldCentric()
          .withDeadband(maxSpeed * 0.05)
          .withRotationalDeadband(maxAngularRate * 0.05)
          .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

  public AlignToPose(
      Supplier<Pose2d> Pose,
      CommandSwerveDrivetrain drivetrain,
      DoubleSupplier elevatorHeight,
      CommandXboxController driverController) {
    addRequirements(drivetrain);

    this.drivetrain = drivetrain;
    this.targetPose = Pose;
    this.driverController = driverController;
    this.elevatorHeight = elevatorHeight;
  }

  /**
   * @return if it is at pose true if not false
   */
  public boolean isAtTargetPose() {
    boolean isAtX = drivetrain.PID_X.atSetpoint();
    boolean isAtY = drivetrain.PID_Y.atSetpoint();
    boolean isAtRotation = drivetrain.PID_Rotation.atSetpoint();
    DogLog.log("Align/atX", isAtX);
    DogLog.log("Align/atY", isAtY);
    DogLog.log("Align/atRotation", isAtRotation);

    if (isAtX && isAtY && isAtRotation) {
      return true;
    }
    return false;
  }

  public boolean isJoystickActive() {
    // get joystick x and y
    // if between certain values (absolute value of y > something), report true
    // repeat for absolute value of x, report true
    // else report false
    double xVelocity = driverController.getLeftY();
    double yVelocity = driverController.getLeftX();

    if (Math.abs(yVelocity) > 0.1 || Math.abs(xVelocity) > 0.1) {
      return true;
    }
    return false;
  }

  @Override
  public void initialize() {
    drivetrain.goToPoseWithPID(targetPose.get());
    DogLog.log("Align/Target Pose", targetPose.get());
  }

  @Override
  public void execute() {
    Pose2d currPose;
    currPose = drivetrain.getState().Pose;
    double currX = currPose.getX();
    double currY = currPose.getY();
    Double currRotation = currPose.getRotation().getDegrees();

    double PIDXOutput = MathUtil.clamp(drivetrain.PID_X.calculate(currX), -PID_MAX, PID_MAX);
    double xVelocity = -PIDXOutput;
    DogLog.log("Align/PIDXOutput", PIDXOutput);

    double PIDYOutput = MathUtil.clamp(drivetrain.PID_Y.calculate(currY), -PID_MAX, PID_MAX);
    double yVelocity = -PIDYOutput;
    DogLog.log("Align/PIDYoutput", PIDYOutput);

    double PIDRotationOutput =
        MathUtil.clamp(drivetrain.PID_Rotation.calculate(currRotation), -PID_MAX, PID_MAX);
    double angularVelocity = PIDRotationOutput;
    DogLog.log("Align/PIDRotationoutput", PIDRotationOutput);

    if (elevatorHeight.getAsDouble() > 0.3) {
      if (resetLimiter) {
        resetLimiter = false;
        xVelocityLimiter.reset(xVelocity);
        yVelocityLimiter.reset(yVelocity);
        angularVelocityLimiter.reset(angularVelocity);
      }
      xVelocity = MathUtil.clamp(xVelocity, -0.2, 0.2);
      yVelocity = MathUtil.clamp(yVelocity, -0.2, 0.2);
      angularVelocity = MathUtil.clamp(angularVelocity, -0.2, 0.2);

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
    if (isJoystickActive()) {
      return true;
    }
    return false;
  }
}
