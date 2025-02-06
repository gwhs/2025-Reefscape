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
import edu.wpi.first.wpilibj.DriverStation;
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
import frc.robot.commands.autonomous.*;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.aprilTagCam.AprilTagCam;
import frc.robot.subsystems.aprilTagCam.AprilTagCamConstants;
import frc.robot.subsystems.arm.ArmConstants;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.elevator.ElevatorConstants;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
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
  private final EagleUtil utils = new EagleUtil();

  private final SendableChooser<Command> autoChooser = new SendableChooser<Command>();

  public enum CoralLevel {
    L1,
    L2,
    L3,
    L4
  }

  public static CoralLevel coralLevel = CoralLevel.L4;
  public static final Trigger IS_L1 = new Trigger(() -> coralLevel == CoralLevel.L1);
  public static final Trigger IS_L2 = new Trigger(() -> coralLevel == CoralLevel.L2);
  public static final Trigger IS_L3 = new Trigger(() -> coralLevel == CoralLevel.L3);
  public static final Trigger IS_L4 = new Trigger(() -> coralLevel == CoralLevel.L4);

  public static final Trigger IS_DISABLED = new Trigger(() -> DriverStation.isDisabled());

  private final RobotVisualizer robotVisualizer = new RobotVisualizer(elevator, arm);

  private AprilTagCam cam3 =
      new AprilTagCam(
          "cam3",
          AprilTagCamConstants.FRONT_LEFT_CAMERA_LOCATION,
          drivetrain::addVisionMeasurent,
          () -> drivetrain.getState().Pose,
          () -> drivetrain.getState().Speeds);

  private AprilTagCam cam4 =
      new AprilTagCam(
          "cam4",
          AprilTagCamConstants.FRONT_RIGHT_CAMERA_LOCATION,
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
    SmartDashboard.putData("Robot Command/Score Coral", scoreCoral());
    SmartDashboard.putData("Robot Command/Prep Score Coral", prepScoreCoral(0, 0));

    // Calculate reef setpoints at startup
    EagleUtil.calculateBlueReefSetPoints();
    EagleUtil.calculateRedReefSetPoints();
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
                })
            .ignoringDisable(true));

    IS_DISABLED.onFalse(
        Commands.runOnce(
                () -> {
                  drivetrain.configNeutralMode(NeutralModeValue.Brake);
                })
            .ignoringDisable(false));

    m_driverController
        .x()
        .whileTrue(
            Commands.startEnd(
                    () -> driveCommand.setTargetMode(DriveCommand.TargetMode.CORAL_STATION),
                    () -> driveCommand.setTargetMode(DriveCommand.TargetMode.REEF))
                .withName("Face Coral Station"));

    m_driverController.x().whileTrue(prepCoralIntake()).onFalse(coralHandoff());

    IS_L4
        .and(m_driverController.rightTrigger())
        .whileTrue(prepScoreCoral(ElevatorSubsystem.rotationsToMeters(57), 210));
    IS_L3.and(m_driverController.rightTrigger()).whileTrue(prepScoreCoral(0.0, 220));
    IS_L2.and(m_driverController.rightTrigger()).whileTrue(prepScoreCoral(0.0, 210));
    IS_L1.and(m_driverController.rightTrigger()).whileTrue(prepScoreCoral(0.0, 210));

    m_driverController.rightTrigger().onFalse(scoreCoral());

    m_driverController.start().onTrue(Commands.runOnce(drivetrain::seedFieldCentric));

    m_driverController
        .rightTrigger()
        .whileTrue(
            Commands.startEnd(
                    () -> {
                      driveCommand.setDriveMode(DriveCommand.DriveMode.ROBOT_CENTRIC);
                      driveCommand.setSlowMode(true);
                    },
                    () -> {
                      driveCommand.setDriveMode(DriveCommand.DriveMode.FIELD_CENTRIC);
                      driveCommand.setSlowMode(false);
                    })
                .withName("Slow and Robot Centric"));

    m_driverController
        .a()
        .whileTrue(alignToPose(() -> EagleUtil.getCachedReefPose(drivetrain.getState().Pose)));

    m_operatorController.y().onTrue(Commands.runOnce(() -> coralLevel = CoralLevel.L4));
    m_operatorController.b().onTrue(Commands.runOnce(() -> coralLevel = CoralLevel.L3));
    m_operatorController.a().onTrue(Commands.runOnce(() -> coralLevel = CoralLevel.L2));
    m_operatorController.x().onTrue(Commands.runOnce(() -> coralLevel = CoralLevel.L1));
  }

  public void periodic() {
    robotVisualizer.update();
    cam3.updatePoseEstim();
    cam4.updatePoseEstim();
    DogLog.log("Desired Reef", coralLevel);
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

  // grabs coral from the intake
  public Command coralHandoff() {
    return Commands.sequence(
            elevator.setHeight(ElevatorConstants.STOW_METER).withTimeout(0.5),
            arm.setAngle(ArmConstants.ARM_INTAKE_ANGLE).withTimeout(1),
            elevator.setHeight(ElevatorConstants.INTAKE_METER).withTimeout(1),
            elevator.setHeight(ElevatorConstants.STOW_METER).withTimeout(1))
        .withName("Coral HandOff");
  }

  // Sets it to the right height and arm postion to intake coral
  public Command prepCoralIntake() {
    return Commands.sequence(
            elevator.setHeight(ElevatorConstants.STOW_METER).withTimeout(0.5),
            arm.setAngle(ArmConstants.ARM_INTAKE_ANGLE).withTimeout(1))
        .withName("Prepare Coral Intake");
  }

  // Sets elevator and arm to postion
  public Command prepScoreCoral(double elevatorHeight, double armAngle) {
    return Commands.sequence(
            elevator.setHeight(elevatorHeight).withTimeout(0.5),
            arm.setAngle(armAngle).withTimeout(1))
        .withName("Prepare Score Coral");
  }

  // scores coral
  public Command scoreCoral() {
    return Commands.sequence(
            arm.setAngle(ArmConstants.ARM_INTAKE_ANGLE).withTimeout(1),
            elevator.setHeight(ElevatorConstants.STOW_METER).withTimeout(0.5))
        .withName("Score Coral");
  }

  public Command prepScoreCoraL3() {
    double elevatorHeight = ElevatorConstants.L3_PREP_POSITION;
    double armAngle = ElevatorConstants.L3_PREP_POSITION;
    return Commands.sequence(
            elevator.setHeight(elevatorHeight).withTimeout(0.5),
            arm.setAngle(armAngle).withTimeout(1))
        .withName("Prepare Score Coral L3");
  }

  public Command scoreCoralL3Command() {
    return Commands.sequence(
            arm.setAngle(ArmConstants.ARM_INTAKE_ANGLE).withTimeout(1),
            elevator.setHeight(ElevatorConstants.STOW_METER).withTimeout(0.5))
        .withName("Score Coral L3");
  }

  public Command prepScoreCoralL4() {
    double elevatorHeight = ElevatorConstants.L4_PREP_POSITION;
    double armAngle = ArmConstants.L4_PREP_POSITION;
    return Commands.sequence(
            elevator.setHeight(elevatorHeight).withTimeout(0.5),
            arm.setAngle(armAngle).withTimeout(1))
        .withName("Prepare Score Coral L4");
  }

  public Command scoreCoralL4Command() {
    return Commands.sequence(
            arm.setAngle(ArmConstants.ARM_INTAKE_ANGLE).withTimeout(1),
            elevator.setHeight(ElevatorConstants.STOW_METER).withTimeout(0.5))
        .withName("Score Coral L4");
  }
}
