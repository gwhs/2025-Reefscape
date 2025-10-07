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
import frc.robot.EagleUtil;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.CommandSwerveDrivetrain.DriveMode;
import frc.robot.subsystems.CommandSwerveDrivetrain.FaceTarget;
import java.util.function.DoubleSupplier;

public class DriveCommand extends Command {
  private static final double PID_MAX = 0.70;

  private final CommandSwerveDrivetrain drivetrain;
  private final CommandXboxController driverController;
  private final SlewRateLimiter angularVelocityLimiter;
  private final SlewRateLimiter xVelocityLimiter;
  private final SlewRateLimiter yVelocityLimiter;
  private final PIDController PID;

  private final double DEAD_BAND = 0.1;
  private boolean resetLimiter = true;

  private double maxSpeed = CommandSwerveDrivetrain.kSpeedAt12Volts.in(MetersPerSecond);
  private double maxAngularRate = 2.5 * Math.PI;

  private final double RED_LEFT_STATION_ANGLE = 126;
  private final double RED_RIGHT_STATION_ANGLE = -126;
  private final double BLUE_LEFT_STATION_ANGLE = 54;
  private final double BLUE_RIGHT_STATION_ANGLE = -54;

  private final double BLUE_CAGE_ANGLE = 90;
  private final double RED_CAGE_ANGLE = -90;

  private final double ELEVATOR_UP_SLEW_RATE = 1;

  private final DoubleSupplier elevatorHeight;

  // Unit is meters
  private static final double halfWidthField = 4.0359;

  private final SwerveRequest.FieldCentric fieldCentricDrive =
      new SwerveRequest.FieldCentric()
          .withDeadband(maxSpeed * 0.0)
          .withRotationalDeadband(maxAngularRate * 0.0)
          .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
          
  private final SwerveRequest.RobotCentric robotCentricDrive =
      new SwerveRequest.RobotCentric()
          .withDeadband(maxSpeed * 0.0)
          .withRotationalDeadband(maxAngularRate * 0.0)
          .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

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

  /**
   * @param currentRobotPose the current pose of the robot via drivetrain.getpose();
   * @return returns the angle?
   */
  public double calculateSetpoint(Pose2d currentRobotPose) {
    if (drivetrain.getFaceTarget() == FaceTarget.CORAL_STATION) {
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

    } else if (drivetrain.getFaceTarget() == FaceTarget.FRONT_REEF) {
      return EagleUtil.getRotationCenterReef(currentRobotPose);
    } else if (drivetrain.getFaceTarget() == FaceTarget.BACK_REEF) {
      return EagleUtil.getRotationCenterReef(currentRobotPose) + 180;
    } else if (drivetrain.getFaceTarget() == FaceTarget.CAGE) {
      if (DriverStation.getAlliance().isPresent()
          && DriverStation.getAlliance().get() == DriverStation.Alliance.Blue) {
        return BLUE_CAGE_ANGLE;
      } else {
        return RED_CAGE_ANGLE;
      }
    } else if (drivetrain.getFaceTarget() == FaceTarget.PROCESSOR) {
      if (DriverStation.getAlliance().isPresent()
          && DriverStation.getAlliance().get() == DriverStation.Alliance.Blue) {
        return 90;
      } else {
        return -90;
      }
    } else if (drivetrain.getFaceTarget() == FaceTarget.FRONT_REEF_FACES) {
      Pose2d nearest = EagleUtil.getCachedReefPose(currentRobotPose);
      return nearest.getRotation().getDegrees();
    } else if (drivetrain.getFaceTarget() == FaceTarget.BACK_REEF_FACES) {
      Pose2d nearest = EagleUtil.getCachedReefPose(currentRobotPose);
      return nearest.getRotation().getDegrees() + 180;

    } else {
      return 0;
    }
  }

  @Override
  public void execute() {
    Pose2d currentRobotPose = drivetrain.getState().Pose;
    double currentRotation = currentRobotPose.getRotation().getDegrees();

    double X = -driverController.getLeftY();
    double Y = -driverController.getLeftX();
    if (drivetrain.isInverted() && drivetrain.getDriveMode() == DriveMode.ROBOT_CENTRIC) {
      X *= -1;
      Y *= -1;
    }

    double xVelocity = MathUtil.applyDeadband(X, 0.1);
    double yVelocity = MathUtil.applyDeadband(Y, 0.1);
    double angularVelocity = MathUtil.applyDeadband(-driverController.getRightX(), 0.1);

    if (drivetrain.isSlow()) {
      xVelocity *= drivetrain.getSlowFactor();
      yVelocity *= drivetrain.getSlowFactor();
      angularVelocity *= drivetrain.getSlowFactor();
    }

    if (Math.abs(driverController.getRightX()) < DEAD_BAND
        && drivetrain.getFaceTarget() != FaceTarget.NONE) {
      PID.setSetpoint(calculateSetpoint(currentRobotPose));
      double pidOutput = PID.calculate(currentRotation);
      pidOutput = MathUtil.clamp(pidOutput, -PID_MAX, PID_MAX);
      angularVelocity = pidOutput;
      DogLog.log("Drive Command/CoralTrackingPIDOutput", pidOutput);
    }

    if (elevatorHeight.getAsDouble() > 0.4) {
      if (resetLimiter) {
        resetLimiter = false;
        xVelocityLimiter.reset(xVelocity);
        yVelocityLimiter.reset(yVelocity);
        angularVelocityLimiter.reset(angularVelocity);
      }
      xVelocity = MathUtil.clamp(xVelocity, -0.2, 0.2);
      yVelocity = MathUtil.clamp(yVelocity, -0.2, 0.2);
      angularVelocity = MathUtil.clamp(angularVelocity, -0.4, 0.4);

      xVelocity = xVelocityLimiter.calculate(xVelocity);
      yVelocity = yVelocityLimiter.calculate(yVelocity);
      angularVelocity = angularVelocityLimiter.calculate(angularVelocity);
    } else {
      resetLimiter = true;
    }

    xVelocity *= maxSpeed;
    yVelocity *= maxSpeed;
    angularVelocity *= maxAngularRate;

    DogLog.log("Drive Command/xVelocity", xVelocity);
    DogLog.log("Drive Command/yVelocity", yVelocity);
    DogLog.log("Drive Command/angularVelocity", angularVelocity);
    DogLog.log("Drive Command/rotationSetpoint", PID.getSetpoint());
    DogLog.log("Drive Command/isSlow", drivetrain.isSlow());
    DogLog.log("Drive Command/targetMode", drivetrain.getFaceTarget());
    DogLog.log("Drive Command/Drive Mode", drivetrain.getDriveMode());
    DogLog.log("Drive Command/slowFactor", drivetrain.getSlowFactor());
    if (drivetrain.getDriveMode() == DriveMode.ROBOT_CENTRIC) {
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
