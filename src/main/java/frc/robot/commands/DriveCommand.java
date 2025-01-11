package frc.robot.commands;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.fasterxml.jackson.databind.util.TokenBufferReadContext;
import com.ctre.phoenix6.swerve.SwerveRequest;
import dev.doglog.DogLog;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class DriveCommand extends Command {

  private double MaxSpeed =
      TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12VoltsMps desired top speed
  private double MaxAngularRate =
      3.5 * Math.PI; // 3/4 of a rotation per second max angular velocity

  public static final double PID_MAX = 0.35;

  private PIDController PID;
  private PIDController PIDX;
  private PIDController PIDY;
  private PIDController PIDRotation;
  private CommandSwerveDrivetrain drivetrain;
  private CommandXboxController driverController;
  private Pose2d currPose;
  private boolean isSlow = false;
  private boolean isAligningToPose = false;
  private final SwerveRequest.FieldCentric drive =
      new SwerveRequest.FieldCentric()
          .withDeadband(MaxSpeed * 0.1)
          .withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
          .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // I want field-centric

  // driving in open loop

  public DriveCommand(CommandXboxController driverController, CommandSwerveDrivetrain drivetrain) {
    this.driverController = driverController;
    this.drivetrain = drivetrain;

    this.PID = new PIDController(0.02, 0, 0);
    this.PID.setTolerance(0.1);
    this.PID.enableContinuousInput(-180, 180);
    this.PIDX = new PIDController(2.7, 0, 0); // same for now tune later
    this.PIDX.setTolerance(0.1);
    this.PIDY = new PIDController(2.7, 0, 0);
    this.PIDY.setTolerance(0.1);
    this.PIDRotation = new PIDController(0.002, 0, 0);
    this.PIDRotation.setTolerance(0.1);
    this.PIDRotation.enableContinuousInput(Radians.fromBaseUnits(-180), Radians.fromBaseUnits(180));
    addRequirements(drivetrain);
    SmartDashboard.putData(
        "goto420",
        Commands.runOnce(
            () -> {
              isAligningToPose = true;
              goToPoseWithPID(new Pose2d(4.00, 2.00, new Rotation2d(3.00)));
            }));
  }

  public boolean getIsAligningToPose() {
    return isAligningToPose;
  }

  public void setIsAligningToPose(boolean isAligning) {
    isAligningToPose = isAligning;
  }

  public boolean isAtTargetPose() {
    if (isAligningToPose == false) {
      return true;
    }
    if (PIDX.atSetpoint() && PIDY.atSetpoint() && PIDRotation.atSetpoint()) {
      return true;
    }
    return false;
  }

  public void goToPoseWithPID(Pose2d targetPose) {
    PIDX.setSetpoint(targetPose.getX());
    PIDY.setSetpoint(targetPose.getY());
    PIDRotation.setSetpoint(targetPose.getRotation().getDegrees());
  }

  public void execute() {
    double xVelocity = -driverController.getLeftY();
    double yVelocity = -driverController.getLeftX();

    double angularVelocity = -driverController.getRightX();
    currPose = drivetrain.getState().Pose;

    if (isSlow) {
      double slowFactor = 0.25;
      xVelocity *= slowFactor;
      yVelocity *= slowFactor;
      angularVelocity *= slowFactor;
    }
    if (isAligningToPose) {
      double currX = currPose.getX();
      double currY = currPose.getY();
      Double currRotation = currPose.getRotation().getDegrees();
      double PIDXOutput = PIDX.calculate(currX);
      xVelocity -= PIDXOutput;
      DogLog.log("PIDXOutput", PIDXOutput);
      double PIDYOutput = PIDY.calculate(currY);
      yVelocity -= PIDYOutput;
      DogLog.log("PIDYoutput", PIDYOutput);
      double PIDRotationOutput = PIDRotation.calculate(currRotation);
      angularVelocity += PIDRotationOutput;
      DogLog.log("PIDRotationoutput", PIDRotationOutput);
    }
    xVelocity = xVelocity * MaxSpeed;
    yVelocity = yVelocity * MaxSpeed;
    angularVelocity = angularVelocity * MaxAngularRate;

    DogLog.log("Drive Command/xVelocity", xVelocity);
    DogLog.log("Drive Command/yVelocity", yVelocity);
    DogLog.log("Drive Command/angularVelocity", angularVelocity);

    drivetrain.setControl(
        drive
            .withVelocityX(xVelocity) // Drive forward with negative Y (forward)
            .withVelocityY(yVelocity) // Drive left with negative X (left)
            .withRotationalRate(angularVelocity)); // Drive counterclockwise with negative X (left)
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
