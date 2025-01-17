// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;

import com.pathplanner.lib.commands.PathfindingCommand;
import dev.doglog.DogLog;
import dev.doglog.DogLogOptions;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.AlignToPose;
import frc.robot.commands.DriveCommand;
import frc.robot.commands.WheelRadiusCharacterization;
import frc.robot.commands.autonomous.*;
import frc.robot.commands.autonomous.Drivetrainpractice;
import frc.robot.commands.autonomous.SC_preloadScore;
import frc.robot.commands.autonomous.Template;
import frc.robot.commands.autonomous.auton_2_cycle;
import frc.robot.commands.autonomous.auton_2_cycle2;
import frc.robot.commands.autonomous.startLnLeave;
import frc.robot.commands.autonomous.startLnLeave2;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import java.util.function.Supplier;

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
    SmartDashboard.putData(
        "LockIn", alignToPose(() -> new Pose2d(2.00, 4.00, Rotation2d.fromDegrees(0))));
    SmartDashboard.putData(
        "LockOut", alignToPose(() -> new Pose2d(2.00, 4.00, Rotation2d.fromDegrees(180))));

    m_driverController
        .rightTrigger()
        .onTrue(
            alignToPose(
                    () -> {
                      Pose2d curPose = drivetrain.getState().Pose;
                      return new Pose2d(
                          curPose.getX() + 0.2, curPose.getY(), curPose.getRotation());
                    })
                .andThen(Commands.print("YAY")));

    m_driverController.start().onTrue(Commands.runOnce(drivetrain::seedFieldCentric));
  }

  public void periodic() {}

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }

  private void configureAutonomous() {

    autoChooser.setDefaultOption("S3-Leave", new Template(this));

    autoChooser.addOption(
        "Wheel Radius Characterization",
        WheelRadiusCharacterization.wheelRadiusCharacterization(drivetrain));

    autoChooser.setDefaultOption("auton_2_cycle", new auton_2_cycle(this));
    autoChooser.addOption("auton_2_cycle2", new auton_2_cycle2(this));
    autoChooser.addOption("SC_preloadScore", new SC_preloadScore(this));

    autoChooser.addOption("startLnLeave", new startLnLeave(this));
    autoChooser.addOption("TestPath", new Drivetrainpractice(this));
    autoChooser.addOption("startLnLeave2", new startLnLeave2(this));
    autoChooser.addOption("S1-Leave", new Template(this));

    // TODO: add more autonomous routines

    SmartDashboard.putData("autonomous", autoChooser);
  }

  public Command alignToPose(Supplier<Pose2d> Pose) {
    return new AlignToPose(Pose, drivetrain);
  }
}
