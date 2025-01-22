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

public class DriveCommand extends Command {
  private static final double PID_MAX = 0.35;

  private final CommandSwerveDrivetrain drivetrain;
  private final CommandXboxController driverController;
  private final PIDController PID;

  public boolean isSlow = true;
  public boolean isBackCoralStation = false;
  public boolean robotCentric = false;
  public boolean isAlignCoral = false;

  private double maxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
  private double maxAngularRate = 3.5 * Math.PI;

  private final double RED_LEFT_STATION_ANGLE = 126;
  private final double RED_RIGHT_STATION_ANGLE = -126;
  private final double BLUE_LEFT_STATION_ANGLE = 54;
  private final double BLUE_RIGHT_STATION_ANGLE = -54;

  public static final double ELEVATOR_UP_SLEW_RATE = 0.0;

  // Unit is meters
  private static final double halfWidthField = 4.0359;

  SlewRateLimiter limiter = new SlewRateLimiter(ELEVATOR_UP_SLEW_RATE);

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

  public DriveCommand(CommandXboxController driverController, CommandSwerveDrivetrain drivetrain) {
    this.driverController = driverController;
    this.drivetrain = drivetrain;

    this.PID = new PIDController(0.02, 0, 0);
    this.PID.setTolerance(0.1);
    this.PID.enableContinuousInput(-180, 180);

    addRequirements(drivetrain);
  }

  @Override
  public void execute() {
    Pose2d currentRobotPose = drivetrain.getState().Pose;
    double currentRotation = currentRobotPose.getRotation().getDegrees();

    double xVelocity = -driverController.getLeftY();
    double yVelocity = -driverController.getLeftX();

    double angularVelocity = -driverController.getRightX();

    SlewRateLimiter xVelocityLimiter = new SlewRateLimiter(0.5);  // Adjust ramp rate as needed
    SlewRateLimiter yVelocityLimiter = new SlewRateLimiter(0.5);
    SlewRateLimiter angularVelocityLimiter = new SlewRateLimiter(0.5);

    if (isSlow) {
      double slowFactor = 0.25;
      xVelocity *= slowFactor;
      yVelocity *= slowFactor;
      angularVelocity *= slowFactor;
    }

    double slowEleXVelocity = xVelocityLimiter.calculate(xVelocity);
    double slowEleYVelocity = yVelocityLimiter.calculate(yVelocity);
    double slowEleAngularVelocity = angularVelocityLimiter.calculate(angularVelocity);

    // Multiply by max speed/velocity to apply scaling factors
    slowEleXVelocity *= maxSpeed;
    slowEleYVelocity *= maxSpeed;
    slowEleAngularVelocity *= maxAngularRate;

    if (isBackCoralStation) {
      if (DriverStation.getAlliance().isPresent()
          && DriverStation.getAlliance().get() == DriverStation.Alliance.Blue) {
        // Blue Alliance
        if (currentRobotPose.getY() <= halfWidthField) {
          // Low Y => "Right" station for Blue
          PID.setSetpoint(BLUE_LEFT_STATION_ANGLE);
        } else {
          // High Y => "Left" station for Blue
          PID.setSetpoint(BLUE_RIGHT_STATION_ANGLE);
        }
      } else {
        // Red Alliance or invalid
        if (currentRobotPose.getY() <= halfWidthField) {
          PID.setSetpoint(RED_LEFT_STATION_ANGLE);
        } else {
          PID.setSetpoint(RED_RIGHT_STATION_ANGLE);
        }
      }

      // Feed the fixed angle into the PID
      double pidOutput = PID.calculate(currentRotation);
      pidOutput = MathUtil.clamp(pidOutput, -PID_MAX, PID_MAX);

      // Override the user's rotation with the PID result
      angularVelocity = pidOutput;

      DogLog.log("Drive Command/CoralTrackingPIDOutput", pidOutput);
    }

    if (isAlignCoral) {
      if (DriverStation.getAlliance().isPresent()
          && DriverStation.getAlliance().get() == DriverStation.Alliance.Blue) {
        Pose2d nearestPoint = currentRobotPose.nearest(FieldConstants.blueReefSetpointList);
        PID.setSetpoint(nearestPoint.getRotation().getDegrees());
      } else {
        Pose2d nearestPoint = currentRobotPose.nearest(FieldConstants.redReefSetpointList);
        PID.setSetpoint(nearestPoint.getRotation().getDegrees());
      }

      // Feed the fixed angle into the PID
      double pidOutput = PID.calculate(currentRotation);
      pidOutput = MathUtil.clamp(pidOutput, -PID_MAX, PID_MAX);

      // Override the user's rotation with the PID result
      angularVelocity = pidOutput;

      DogLog.log("Drive Command/ReefTrackingPIDOutput", pidOutput);
    }

    // Multiply by our maximum speeds/rates
    xVelocity *= maxSpeed;
    yVelocity *= maxSpeed;
    angularVelocity *= maxAngularRate;

    DogLog.log("Drive Command/xVelocity", xVelocity);
    DogLog.log("Drive Command/yVelocity", yVelocity);
    DogLog.log("Drive Command/angularVelocity", angularVelocity);
    DogLog.log("Drive Command/rotationSetpoint", PID.getSetpoint());

    if (robotCentric) {
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
    if (robotCentric) {
      drivetrain.setControl(
          robotCentricDrive
              .withVelocityX(slowEleXVelocity)
              .withVelocityY(slowEleYVelocity)
              .withRotationalRate(slowEleAngularVelocity));
  } else {
      drivetrain.setControl(
          fieldCentricDrive
              .withVelocityX(slowEleXVelocity)
              .withVelocityY(slowEleYVelocity)
              .withRotationalRate(slowEleAngularVelocity));
  }
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
