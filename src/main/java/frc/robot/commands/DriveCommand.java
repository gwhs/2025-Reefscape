package frc.robot.commands;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import dev.doglog.DogLog;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class DriveCommand extends Command {

  private double maxSpeed =
      TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12VoltsMps desired top speed
  private double maxAngularRate =
      3.5 * Math.PI; // 3/4 of a rotation per second max angular velocity

  public static final double PID_MAX = 0.35;

  private PIDController PID;

  private CommandSwerveDrivetrain drivetrain;
  private CommandXboxController driverController;

  public boolean isSlow = false;
  public boolean robotCentric = false;

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

  // driving in open loop

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

    if (isSlow) {
      double slowFactor = 0.25;
      xVelocity *= slowFactor;
      yVelocity *= slowFactor;
      angularVelocity *= slowFactor;
    }

    xVelocity = xVelocity * maxSpeed;
    yVelocity = yVelocity * maxSpeed;
    angularVelocity = angularVelocity * maxAngularRate;

    DogLog.log("Drive Command/xVelocity", xVelocity);
    DogLog.log("Drive Command/yVelocity", yVelocity);
    DogLog.log("Drive Command/angularVelocity", angularVelocity);

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
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
