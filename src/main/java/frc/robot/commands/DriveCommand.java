package frc.robot.commands;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import dev.doglog.DogLog;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.generated.TunerConstants;
import frc.robot.subsytems.CommandSwerveDrivetrain;

  
public class DriveCommand extends  Command{
  private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12VoltsMps desired top speed
  private double MaxAngularRate = 3.5 * Math.PI; // 3/4 of a rotation per second max angular velocity

  public static final double PID_MAX = 0.35;

  private PIDController PID;
  private CommandSwerveDrivetrain drivetrain;
  private CommandXboxController driverController;
  private Pose2d currPose;
  public boolean isSlow = false;
  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
      .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // I want field-centric
                                                               // driving in open loop
            

  public DriveCommand(CommandXboxController driverController, CommandSwerveDrivetrain drivetrain) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.driverController = driverController;
    this.drivetrain = drivetrain;

    this.PID = new PIDController(0.02, 0, 0);
    this.PID.setTolerance(0.1);
    this.PID.enableContinuousInput(-180, 180);

    addRequirements(drivetrain);
  }

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

    xVelocity = xVelocity * MaxSpeed;
    yVelocity = yVelocity * MaxSpeed;
    angularVelocity = angularVelocity * MaxAngularRate;

    DogLog.log("Drive Command/xVelocity", xVelocity);
    DogLog.log("Drive Command/yVelocity", yVelocity);
    DogLog.log("Drive Command/angularVelocity", angularVelocity);

    drivetrain.setControl(drive.withVelocityX(xVelocity) // Drive forward with negative Y (forward)
        .withVelocityY(yVelocity) // Drive left with negative X (left)
        .withRotationalRate(angularVelocity)); // Drive counterclockwise with negative X (left)
  }
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}   
