package frc.robot.commands;

import static edu.wpi.first.units.Units.*;

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
import frc.robot.FieldConstants;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import java.util.function.DoubleSupplier;

public class DriveCommand extends Command {
  private static final double PID_MAX = 0.35;

  private final CommandSwerveDrivetrain drivetrain;
  private final CommandXboxController driverController;
  private final SlewRateLimiter angularVelocityLimiter;
  private final SlewRateLimiter xVelocityLimiter;
  private final SlewRateLimiter yVelocityLimiter;
  private final PIDController PID;

  private boolean isSlow = true;
  public boolean isRobotCentric = false;

  public boolean resetLimiter = true;

  private double maxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
  private double maxAngularRate = 3.5 * Math.PI;

  private final double RED_LEFT_STATION_ANGLE = 126;
  private final double RED_RIGHT_STATION_ANGLE = -126;
  private final double BLUE_LEFT_STATION_ANGLE = 54;
  private final double BLUE_RIGHT_STATION_ANGLE = -54;

  public final double ELEVATOR_UP_SLEW_RATE = 0.5;

  public final DoubleSupplier elevatorHeight;

  // Unit is meters
  private static final double halfWidthField = 4.0359;

  public enum targetMode {
    NORMAL,
    CORAL_STATION,
    REEF
  }

  private targetMode mode = targetMode.NORMAL;

  private final SwerveRequest.FieldCentric fieldCentricDrive =
      new SwerveRequest.FieldCentric()
          .withDeadband(maxSpeed * 0.1)
          .withRotationalDeadband(maxAngularRate * 0.1) // Add a 10% deadband
          .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // I want field-centric
  private final SwerveRequest.RobotCentric robotCentricDrive =
      new SwerveRequest.RobotCentric()
          .withDeadband(maxSpeed * 0.1)
          .withRotationalDeadband(maxAngularRate * 0.1) // Add a 10% deadband
          .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // I want robot-centric

  public DriveCommand(
      CommandXboxController driverController,
      CommandSwerveDrivetrain drivetrain,
      DoubleSupplier elevatorHeight) {
    this.driverController = driverController;
    this.drivetrain = drivetrain;

    this.PID = new PIDController(0.02, 0, 0);
    this.PID.setTolerance(0.1);
    this.PID.enableContinuousInput(-180, 180);

    angularVelocityLimiter = new SlewRateLimiter(ELEVATOR_UP_SLEW_RATE);
    xVelocityLimiter = new SlewRateLimiter(ELEVATOR_UP_SLEW_RATE);
    yVelocityLimiter = new SlewRateLimiter(ELEVATOR_UP_SLEW_RATE);

    this.elevatorHeight = elevatorHeight;

    addRequirements(drivetrain);
  }

  public double calculateSetpoint(Pose2d currentRobotPose, double currentRotation) {
    if (mode == targetMode.CORAL_STATION) {
      if (DriverStation.getAlliance().isPresent()
          && DriverStation.getAlliance().get() == DriverStation.Alliance.Blue) {
        // Blue Alliance
        if (currentRobotPose.getY() <= halfWidthField) {
          // Low Y => "Right" station for Blue
          return BLUE_LEFT_STATION_ANGLE;
        } else {
          // High Y => "Left" station for Blue
          return BLUE_RIGHT_STATION_ANGLE;
        }
      } else {
        // Red Alliance or invalid
        if (currentRobotPose.getY() <= halfWidthField) {
          return RED_LEFT_STATION_ANGLE;
        } else {
          return RED_RIGHT_STATION_ANGLE;
        }
      }

    } else {
      if (DriverStation.getAlliance().isPresent()
          && DriverStation.getAlliance().get() == DriverStation.Alliance.Blue) {
        Pose2d nearestPoint = currentRobotPose.nearest(FieldConstants.blueReefSetpointList);
        return nearestPoint.getRotation().getDegrees();
      } else {
        Pose2d nearestPoint = currentRobotPose.nearest(FieldConstants.redReefSetpointList);
        return nearestPoint.getRotation().getDegrees();
      }
    }
  }

  public void setTargetMode(targetMode mode) {
    this.mode = mode;
  }

  public void setModeSpeed(boolean isSlow) {
    this.isSlow = isSlow;
  }

  @Override
  public void execute() {
    Pose2d currentRobotPose = drivetrain.getState().Pose;
    double currentRotation = currentRobotPose.getRotation().getDegrees();

    double xVelocity = -driverController.getLeftY();
    double yVelocity = -driverController.getLeftX();

    double angularVelocity = -driverController.getRightX();

    if (isSlow) {
      double slowFactor = 0.25;
      xVelocity *= slowFactor;
      yVelocity *= slowFactor;
      angularVelocity *= slowFactor;
    }

    if (elevatorHeight.getAsDouble() > 1) {
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

    if (mode == targetMode.CORAL_STATION || mode == targetMode.REEF) {
      PID.setSetpoint(calculateSetpoint(currentRobotPose, currentRotation));
      double pidOutput = PID.calculate(currentRotation);
      pidOutput = MathUtil.clamp(pidOutput, -PID_MAX, PID_MAX);
      // override angular velocity
      angularVelocity = pidOutput;
      DogLog.log("Drive Command/CoralTrackingPIDOutput", pidOutput);
    }

    xVelocity *= maxSpeed;
    yVelocity *= maxSpeed;
    angularVelocity *= maxAngularRate;

    DogLog.log("Drive Command/xVelocity", xVelocity);
    DogLog.log("Drive Command/yVelocity", yVelocity);
    DogLog.log("Drive Command/angularVelocity", angularVelocity);
    DogLog.log("Drive Command/rotationSetpoint", PID.getSetpoint());
    DogLog.log("Drive Command/isSlow", isSlow);
    DogLog.log("Drive Command/targetMode", mode);
    DogLog.log("Drive Command/isRobotCentric", isRobotCentric);

    if (isRobotCentric) {
      drivetrain.setControl(
          robotCentricDrive
              .withVelocityX(xVelocity)
              .withVelocityY(yVelocity)
              .withRotationalRate(angularVelocity));
    } else {
      drivetrain.setControl(
          fieldCentricDrive
              .withVelocityX(xVelocity)
              .withVelocityY(yVelocity)
              .withRotationalRate(angularVelocity));
    }
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
