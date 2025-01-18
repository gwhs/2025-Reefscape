// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;

import com.pathplanner.lib.commands.PathfindingCommand;
import dev.doglog.DogLog;
import dev.doglog.DogLogOptions;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.DriveCommand;
import frc.robot.commands.autonomous.Templete;
import frc.robot.commands.autonomous.startLnLeave;
import frc.robot.commands.autonomous.startLnLeave2;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.AprilTagCam.AprilTagCam;
import frc.robot.subsystems.AprilTagCam.AprilTagCamConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

  private final CommandXboxController m_driverController = new CommandXboxController(0);
  private final CommandXboxController m_operatorController = new CommandXboxController(1);
  private final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
  private final DriveCommand driveCommand = new DriveCommand(m_driverController, drivetrain);
  private final Telemetry logger =
      new Telemetry(TunerConstants.kSpeedAt12Volts.in(MetersPerSecond));

  private final SendableChooser<Command> autoChooser = new SendableChooser<Command>();

  private AprilTagCam cam3 =
      new AprilTagCam(
          "cam3",
          AprilTagCamConstants.FRONT_RIGHT_CAMERA_LOCATION,
          drivetrain::addVisionMeasurent,
          () -> drivetrain.getState().Pose,
          () -> drivetrain.getState().Speeds);

  private AprilTagCam cam4 =
      new AprilTagCam(
          "cam4",
          AprilTagCamConstants.FRONT_LEFT_CAMERA_LOCATION,
          drivetrain::addVisionMeasurent,
          () -> drivetrain.getState().Pose,
          () -> drivetrain.getState().Speeds);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Setup DogLog
    DogLog.setOptions(
        new DogLogOptions().withNtPublish(true).withCaptureNt(true).withCaptureDs(true));
    DogLog.setPdh(new PowerDistribution());

    configureBindings();

    configureAutonomous();

    // Default Commands
    drivetrain.setDefaultCommand(driveCommand);

    drivetrain.registerTelemetry(logger::telemeterize);

    PathfindingCommand.warmupCommand().schedule();

    SmartDashboard.putData("Command Scheduler", CommandScheduler.getInstance());
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    m_driverController.a().whileTrue(drivetrain.sysIdDynamic(SysIdRoutine.Direction.kForward));
    m_driverController.b().whileTrue(drivetrain.sysIdDynamic(SysIdRoutine.Direction.kReverse));
    m_driverController.x().whileTrue(drivetrain.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    m_driverController.y().whileTrue(drivetrain.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));

    m_driverController.start().onTrue(Commands.runOnce(drivetrain::seedFieldCentric));
  }

  public void periodic() {

    cam3.updatePoseEstim();
    cam4.updatePoseEstim();
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }

  private void configureAutonomous() {
    autoChooser.setDefaultOption("S3-Leave", new Templete(this));

    autoChooser.addOption("startLnLeave", new startLnLeave(this));
    autoChooser.addOption("startLnLeave2", new startLnLeave2(this));

    // TODO: add more autonomous routines

    SmartDashboard.putData("autonomous", autoChooser);
  }
}
