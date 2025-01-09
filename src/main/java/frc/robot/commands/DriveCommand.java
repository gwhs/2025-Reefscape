package frc.robot.commands;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import dev.doglog.DogLog;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class DriveCommand extends Command {
  private static final double PID_MAX = 0.35;


  private final CommandSwerveDrivetrain drivetrain;
  private final CommandXboxController driverController;
  private final PIDController PID;
  private Pose2d currPose;


  public boolean isBackCoralStation = false;
  public boolean isSlow = false;


  private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
  private double MaxAngularRate = 3.5 * Math.PI;


  private static final double halfWidthField = 4.0359;
  private static final double blueLeftCoralTargetX = 1.27;
  private static final double blueLeftCoralTargetY = 1.27;
  private static final double blueRightCoralTargetX = 1.27;
  private static final double blueRightCoralTargetY = 6.80;
  private static final double redLeftCoralTargetX = 16.5;
  private static final double redLeftCoralTargetY = 1.27;
  private static final double redRightCoralTargetX = 16.5;
  private static final double redRightCoralTargetY = 6.80;


  private final SwerveRequest.FieldCentric drive = 
      new SwerveRequest.FieldCentric()
          .withDeadband(MaxSpeed * 0.1)
          .withRotationalDeadband(MaxAngularRate * 0.1)
          .withDriveRequestType(DriveRequestType.OpenLoopVoltage);


  public DriveCommand(CommandXboxController driverController, CommandSwerveDrivetrain drivetrain) {
    this.driverController = driverController;
    this.drivetrain = drivetrain;


    this.PID = new PIDController(0.02, 0, 0);
    this.PID.setTolerance(0.1);
  
    this.PID.enableContinuousInput(-180, 180);


    addRequirements(drivetrain);
  }


  @Override
  public void initialize() {

    isBackCoralStation = true;
  }


  @Override
  public void execute() {
    double xVelocity = -driverController.getLeftY();
    double yVelocity = -driverController.getLeftX();
    double angularVelocity = -driverController.getRightX();


    currPose = drivetrain.getState().Pose;
    double currTheta = currPose.getRotation().getDegrees();


    if (isSlow) {
      double slowFactor = 0.25;
      xVelocity *= slowFactor;
      yVelocity *= slowFactor;
      angularVelocity *= slowFactor;
    }


  
    if (isBackCoralStation) {

      double coralX, coralY;
      if (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == DriverStation.Alliance.Blue) {
        if (currPose.getY() <= halfWidthField) {
          coralX = blueLeftCoralTargetX;
          coralY = blueLeftCoralTargetY;
        } else {
          coralX = blueRightCoralTargetX;
          coralY = blueRightCoralTargetY;
        }
      } else {
        // Red or invalid
        if (currPose.getY() <= halfWidthField) {
          coralX = redLeftCoralTargetX;
          coralY = redLeftCoralTargetY;
        } else {
          coralX = redRightCoralTargetX;
          coralY = redRightCoralTargetY;
        }
      }


      double desiredAngle = calculateBackAngleToTarget(currPose, coralX, coralY);
      PID.setSetpoint(desiredAngle);


      double pidOutput = PID.calculate(currTheta);
      pidOutput = MathUtil.clamp(pidOutput, -PID_MAX, PID_MAX);


      angularVelocity = pidOutput;


      SmartDashboard.putNumber("CoralTrackingDesiredAngle", desiredAngle);
      SmartDashboard.putNumber("CoralTrackingPIDOutput", pidOutput);
    }



    xVelocity *= MaxSpeed;
    yVelocity *= MaxSpeed;
    angularVelocity *= MaxAngularRate;



    DogLog.log("Drive Command/xVelocity", xVelocity);
    DogLog.log("Drive Command/yVelocity", yVelocity);
    DogLog.log("Drive Command/angularVelocity", angularVelocity);



    drivetrain.setControl(
        drive
            .withVelocityX(xVelocity)
            .withVelocityY(yVelocity)
            .withRotationalRate(angularVelocity)
    );
  }


  @Override
  public boolean isFinished() {
    return false;
  }


  public static double calculateBackAngleToTarget(Pose2d pose, double targetX, double targetY) {
    double dx = targetX - pose.getX();
    double dy = targetY - pose.getY();


    double angleToTargetRad = Math.atan2(dy, dx);  // angle from -π to π
    double angleToTargetDeg = Math.toDegrees(angleToTargetRad); // -180 to 180


    double backAngle = angleToTargetDeg + 180;


    return wrapDegrees(backAngle);
  }


  /**
   * Wrap angle to [-180, 180].
   */
  public static double wrapDegrees(double angleDeg) {
    double wrapped = angleDeg % 360.0;
    if (wrapped > 180.0) {
      wrapped -= 360.0;
    } else if (wrapped <= -180.0) {
      wrapped += 360.0;
    }
    return wrapped;
  }
}





