package frc.robot.commands;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import dev.doglog.DogLog;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
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

  private double maxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
  private double maxAngularRate = 3.5 * Math.PI;

  private final double REDLEFTSTATIONANGLE = 126;
  private final double REDRIGHTSTATIONANGLE = -126;
  private final double BLUELEFTSTATIONANGLE = 54;
  private final double BLUERIGHTSTATIONANGLE = -54;

  // Unit is meters
  private static final double halfWidthField = 4.0359;

  private final SwerveRequest.FieldCentric drive =
      new SwerveRequest.FieldCentric()
          .withDeadband(maxSpeed * 0.1)
          .withRotationalDeadband(maxAngularRate * 0.1)
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
      if (DriverStation.getAlliance().isPresent()
          && DriverStation.getAlliance().get() == DriverStation.Alliance.Blue) {
        // Blue Alliance
        if (currPose.getY() <= halfWidthField) {
          // Low Y => "Right" station for Blue
          PID.setSetpoint(BLUELEFTSTATIONANGLE);
        } else {
          // High Y => "Left" station for Blue
          PID.setSetpoint(BLUERIGHTSTATIONANGLE);
        }
      } else {
        // Red Alliance or invalid
        if (currPose.getY() <= halfWidthField) {
          PID.setSetpoint(REDLEFTSTATIONANGLE);
        } else {
          PID.setSetpoint(REDRIGHTSTATIONANGLE);
        }
      }

      // Feed the fixed angle into the PID
      double pidOutput = PID.calculate(currTheta);
      pidOutput = MathUtil.clamp(pidOutput, -PID_MAX, PID_MAX);

      // Override the user's rotation with the PID result
      angularVelocity = pidOutput;

      DogLog.log("Drive Command/CoralTrackingPIDOutput", pidOutput);
    }

    // Multiply by our maximum speeds/rates
    xVelocity *= maxSpeed;
    yVelocity *= maxSpeed;
    angularVelocity *= maxAngularRate;

    DogLog.log("Drive Command/xVelocity", xVelocity);
    DogLog.log("Drive Command/yVelocity", yVelocity);
    DogLog.log("Drive Command/angularVelocity", angularVelocity);

    drivetrain.setControl(
        drive
            .withVelocityX(xVelocity)
            .withVelocityY(yVelocity)
            .withRotationalRate(angularVelocity));
  }
}
