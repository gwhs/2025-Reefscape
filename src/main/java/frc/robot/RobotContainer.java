// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;

import com.ctre.phoenix6.signals.NeutralModeValue;
import com.pathplanner.lib.commands.PathfindingCommand;
import dev.doglog.DogLog;
import dev.doglog.DogLogOptions;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.AlignToPose;
import frc.robot.commands.DriveCommand;
import frc.robot.commands.autonomous.*;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.aprilTagCam.AprilTagCam;
import frc.robot.subsystems.aprilTagCam.AprilTagCamConstants;
import frc.robot.subsystems.arm.ArmConstants;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.elevator.ElevatorConstants;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.led.LedSubsystem;
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

  private final Telemetry logger =
      new Telemetry(TunerConstants.kSpeedAt12Volts.in(MetersPerSecond));

  private final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
  private final ElevatorSubsystem elevator = new ElevatorSubsystem();
  private final ArmSubsystem arm = new ArmSubsystem();
  private final LedSubsystem led = new LedSubsystem();

  private final SendableChooser<Command> autoChooser = new SendableChooser<Command>();

  public static final Trigger IS_DISABLED = new Trigger(() -> DriverStation.isDisabled());

  private final RobotVisualizer robotVisualizer = new RobotVisualizer(elevator, arm);

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

  private final DriveCommand driveCommand =
      new DriveCommand(m_driverController, drivetrain, () -> elevator.getHeightMeters());

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
    SmartDashboard.putData("Robot Command/Prep Coral Intake", prepCoralIntake());
    SmartDashboard.putData("Robot Command/Coral Handoff", coralHandoff());

    EagleUtil.calculateRedReefSetPoints();
    EagleUtil.calculateBlueReefSetPoints();

    DogLog.log("Field Constants/Blue Reef", FieldConstants.blueReefSetpoints);
    DogLog.log("Field Constants/Red Reef", FieldConstants.redReefSetpoints);
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
    IS_DISABLED.onTrue(
        Commands.runOnce(
                () -> {
                  drivetrain.configNeutralMode(NeutralModeValue.Coast);
                  led.setColor(LEDPattern.solid(Color.kRed));
                })
            .ignoringDisable(true));

    IS_DISABLED.onFalse(
        Commands.runOnce(
                () -> {
                  drivetrain.configNeutralMode(NeutralModeValue.Brake);
                  led.setColor(LEDPattern.solid(Color.kGreen));
                })
            .ignoringDisable(false));

    m_driverController
        .x()
        .whileTrue(
            Commands.startEnd(
                    () -> driveCommand.isBackCoralStation = true,
                    () -> driveCommand.isBackCoralStation = false)
                .withName("Face Coral Station"));

    m_driverController
        .rightTrigger()
        .onTrue(alignToPose(() -> new Pose2d(1.00, 1.00, new Rotation2d(1.00))));

    m_driverController.leftBumper().onTrue(setLEDToAllianceColor());

    m_driverController.start().onTrue(Commands.runOnce(drivetrain::seedFieldCentric));

    m_driverController
        .b()
        .whileTrue(
            alignToPose(
                () -> {
                  if (DriverStation.getAlliance().isPresent()
                      && DriverStation.getAlliance().get() == DriverStation.Alliance.Blue) {
                    return drivetrain.getState().Pose.nearest(FieldConstants.blueReefSetpointList);
                  } else {
                    return drivetrain.getState().Pose.nearest(FieldConstants.redReefSetpointList);
                  }
                }));
  }

  public void periodic() {

    robotVisualizer.update();
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
    autoChooser.setDefaultOption("FIVE_CYCLE_PROCESSOR", new FiveCycleProcessor(this));
    autoChooser.addOption("Five_Cycle_Processor_2", new FiveCycleProcessor2(this));
    autoChooser.addOption("Two_Cycle_Processor", new TwoCycleProcessor(this));
    autoChooser.addOption("Two_Cycle_Processor_2", new TwoCycleProcessor2(this));
    autoChooser.addOption("Score_Preload_One_Cycle", new ScorePreloadOneCycle(this));
    autoChooser.addOption("Leave_Non_Processor", new LeaveNonProcessor(this));
    autoChooser.addOption("Drivetrain_Practice", new DrivetrainPractice(this));
    autoChooser.addOption("Leave_Processor", new LeaveProcessor(this));
    autoChooser.addOption("Five_Cycle_Non_Processor", new FiveCycleNonProcessor(this));
    autoChooser.addOption("Five_Cycle_Non_Processor_2", new FiveCycleNonProcessor2(this));

    SmartDashboard.putData("autonomous", autoChooser);
  }

  public Command alignToPose(Supplier<Pose2d> Pose) {
    return new AlignToPose(Pose, drivetrain);
  }

  public Command setLEDToAllianceColor() {
    return Commands.run(() -> led.setColor(LEDPattern.solid(getAllianceColor())));
  }

  public Color getAllianceColor() {
    if (DriverStation.getAlliance().get() == DriverStation.Alliance.Blue) {
      return Color.kBlue;
    }
    return Color.kRed;
  }

  public Command coralHandoff() {
    return Commands.sequence(
            elevator.setHeight(ElevatorConstants.STOW_METER).withTimeout(0.5),
            arm.setAngle(ArmConstants.ARM_INTAKE_ANGLE).withTimeout(1),
            elevator.setHeight(ElevatorConstants.INTAKE_METER).withTimeout(1),
            elevator.setHeight(ElevatorConstants.STOW_METER).withTimeout(1))
        .withName("Coral HandOff");
  }

  public Command prepCoralIntake() {
    return Commands.sequence(
            elevator.setHeight(ElevatorConstants.STOW_METER).withTimeout(0.5),
            arm.setAngle(ArmConstants.ARM_INTAKE_ANGLE).withTimeout(1))
        .withName("Prepare Coral Intake");
  }
}
