// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;

import com.ctre.phoenix6.signals.NeutralModeValue;
import com.pathplanner.lib.commands.PathfindingCommand;
import dev.doglog.DogLog;
<<<<<<< HEAD
import dev.doglog.DogLogOptions;
import edu.wpi.first.hal.simulation.RoboRioDataJNI;
=======
>>>>>>> dev
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.RobotController;
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
import frc.robot.commands.DriveCommand.TargetMode;
import frc.robot.commands.WheelRadiusCharacterization;
import frc.robot.commands.autonomous.*;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.aprilTagCam.AprilTagCam;
import frc.robot.subsystems.aprilTagCam.AprilTagCamConstants;
import frc.robot.subsystems.arm.ArmConstants;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.climb.ClimbSubsystem;
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
  private final Alert batteryUnderTwelveVolts = new Alert("BATTERY UNDER 12V", AlertType.kWarning);
  private final Telemetry logger =
      new Telemetry(TunerConstants.kSpeedAt12Volts.in(MetersPerSecond));

  private final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
  private final ElevatorSubsystem elevator = new ElevatorSubsystem();
  private final ArmSubsystem arm = new ArmSubsystem();
  private final LedSubsystem led = new LedSubsystem();
  private final ClimbSubsystem climb = new ClimbSubsystem();
  private final DriveCommand driveCommand =
      new DriveCommand(m_driverController, drivetrain, () -> elevator.getHeightMeters());
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
  public static final Trigger IS_TELEOP = new Trigger(() -> DriverStation.isTeleopEnabled());
  public final Trigger IS_CLOSE_TO_REEF =
      new Trigger(
          () ->
              EagleUtil.getDistanceBetween(
                      drivetrain.getPose(), EagleUtil.getCachedReefPose(drivetrain.getPose()))
                  < 1.25);
  public final Trigger IS_AT_POSE = new Trigger(() -> driveCommand.isAtSetPoint());
  public final Trigger BROWN_OUT = new Trigger(() -> RobotController.isBrownedOut());

  private final RobotVisualizer robotVisualizer = new RobotVisualizer(elevator, arm);

  private AprilTagCam cam3 =
      new AprilTagCam(
          AprilTagCamConstants.FRONT_LEFT_CAMERA_DEV_NAME,
          AprilTagCamConstants.FRONT_LEFT_CAMERA_LOCATION_COMP,
          drivetrain::addVisionMeasurent,
          () -> drivetrain.getState().Pose,
          () -> drivetrain.getState().Speeds);

  private AprilTagCam cam4 =
      new AprilTagCam(
          AprilTagCamConstants.FRONT_RIGHT_CAMERA_DEV_NAME,
          AprilTagCamConstants.FRONT_RIGHT_CAMERA_LOCATION_COMP,
          drivetrain::addVisionMeasurent,
          () -> drivetrain.getState().Pose,
          () -> drivetrain.getState().Speeds);

  public final Trigger IS_REEFMODE =
      new Trigger(() -> driveCommand.getTargetMode() == TargetMode.REEF);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    configureBindings();

    configureAutonomous();

    // Default Commands
    drivetrain.setDefaultCommand(driveCommand);

    drivetrain.registerTelemetry(logger::telemeterize);

    PathfindingCommand.warmupCommand().schedule();

    SmartDashboard.putData("Command Scheduler", CommandScheduler.getInstance());
    SmartDashboard.putData("Robot Command/Prep Coral Intake", prepCoralIntake());
    SmartDashboard.putData("Robot Command/Coral Handoff", coralHandoff());
    SmartDashboard.putData(
        "Robot Command/prep score", prepScoreCoral(ElevatorSubsystem.rotationsToMeters(57), 210));
    SmartDashboard.putData("Robot Command/Score L4", scoreCoral());
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

    drivetrain
        .IS_ALIGNING_TO_POSE
        .and(drivetrain.IS_AT_TARGET_POSE)
        .onTrue(led.setPattern(LEDPattern.solid(Color.kGreen)));
    drivetrain
        .IS_ALIGNING_TO_POSE
        .and(drivetrain.IS_AT_TARGET_POSE.negate())
        .onTrue(led.setPattern(LEDPattern.solid(Color.kBlack)));


    BROWN_OUT.onTrue( Commands.runOnce(() -> drivetrain.setDriveMotorCurrentLimit()));

    IS_DISABLED.onTrue(
        Commands.runOnce(
                () -> {
                  drivetrain.configNeutralMode(NeutralModeValue.Coast);
                  elevator.setNeutralMode(NeutralModeValue.Coast);
                  driveCommand.stopDrivetrain();
                })
            .ignoringDisable(true));

    IS_DISABLED
        .and(() -> RobotController.getBatteryVoltage() >= 12)
        .onTrue(led.setPattern(LEDPattern.solid(Color.kGreen)));

    IS_DISABLED
        .and(() -> RobotController.getBatteryVoltage() < 12)
        .onTrue(led.setPattern(LEDPattern.solid(Color.kRed)));

    IS_DISABLED.onFalse(
        Commands.runOnce(
                () -> {
                  drivetrain.configNeutralMode(NeutralModeValue.Brake);
                  elevator.setNeutralMode(NeutralModeValue.Brake);
                })
            .ignoringDisable(false));

    IS_DISABLED
        .and(() -> RobotController.getBatteryVoltage() < 12)
        .onTrue(EagleUtil.triggerAlert(batteryUnderTwelveVolts));

    m_driverController
        .x()
        .whileTrue(
            Commands.startEnd(
                    () -> driveCommand.setTargetMode(DriveCommand.TargetMode.CORAL_STATION),
                    () -> driveCommand.setTargetMode(DriveCommand.TargetMode.REEF))
                .withName("Face Coral Station"));

    m_driverController.x().onTrue(Commands.runOnce(() -> driveCommand.setSlowMode(true, 0.25)));

    m_driverController.x().onFalse(Commands.runOnce(() -> driveCommand.setSlowMode(false, 0.25)));

    m_driverController.x().whileTrue(prepCoralIntake()).onFalse(coralHandoff());

    m_driverController
        .leftBumper()
        .onTrue(
            Commands.runOnce(() -> driveCommand.setTargetMode(DriveCommand.TargetMode.NORMAL))
                .withName("Back to Original State"));

    // IS_TELEOP
    //     .and(IS_REEFMODE)
    //     .and(IS_CLOSE_TO_REEF)
    //     .onTrue(
    //         prepScoreCoral(ElevatorConstants.STOW_METER, 220).withName("auto prep score coral"));

    IS_L4
        .and(m_driverController.rightTrigger())
        .whileTrue(
            prepScoreCoral(ElevatorConstants.L4_PREP_POSITION, ArmConstants.L4_PREP_POSITION));
    IS_L3
        .and(m_driverController.rightTrigger())
        .whileTrue(
            prepScoreCoral(ElevatorConstants.L3_PREP_POSITION, ArmConstants.L3_PREP_POSITION));
    IS_L2
        .and(m_driverController.rightTrigger())
        .whileTrue(
            prepScoreCoral(ElevatorConstants.L2_PREP_POSITION, ArmConstants.L2_PREP_POSITION));
    IS_L1
        .and(m_driverController.rightTrigger())
        .whileTrue(
            prepScoreCoral(ElevatorConstants.L1_PREP_POSITION, ArmConstants.L1_PREP_POSITION));

    IS_L4.and(m_driverController.rightTrigger().negate()).onTrue(scoreCoral());
    IS_L3.and(m_driverController.rightTrigger().negate()).onTrue(scoreCoral());
    IS_L2.and(m_driverController.rightTrigger().negate()).onTrue(scoreCoral());
    IS_L1.and(m_driverController.rightTrigger().negate()).onTrue(scoreCoral());

    m_driverController.start().onTrue(Commands.runOnce(drivetrain::seedFieldCentric));

    m_driverController
        .rightTrigger()
        .whileTrue(
            Commands.startEnd(
                    () -> {
                      driveCommand.setDriveMode(DriveCommand.DriveMode.ROBOT_CENTRIC);
                      driveCommand.setSlowMode(true, 0.25);
                    },
                    () -> {
                      driveCommand.setDriveMode(DriveCommand.DriveMode.FIELD_CENTRIC);
                      driveCommand.setSlowMode(false, 0.25);
                    })
                .withName("Slow and Robot Centric"));

    m_driverController
        .a()
        .whileTrue(alignToPose(() -> EagleUtil.getCachedReefPose(drivetrain.getState().Pose)));

    m_driverController
        .b()
        .whileTrue(alignToPose(() -> EagleUtil.closestReefSetPoint(drivetrain.getPose(), 1)));

    m_operatorController.y().onTrue(Commands.runOnce(() -> coralLevel = CoralLevel.L4));
    m_operatorController.b().onTrue(Commands.runOnce(() -> coralLevel = CoralLevel.L3));
    m_operatorController.a().onTrue(Commands.runOnce(() -> coralLevel = CoralLevel.L2));
    m_operatorController.x().onTrue(Commands.runOnce(() -> coralLevel = CoralLevel.L1));

    // m_operatorController.y().whileTrue(elevator.sysIdQuasistatic(Direction.kForward));
    // m_operatorController.b().whileTrue(elevator.sysIdQuasistatic(Direction.kReverse));
    // m_operatorController.a().whileTrue(elevator.sysIdDynamic(Direction.kForward));
    // m_operatorController.x().whileTrue(elevator.sysIdDynamic(Direction.kReverse));
  }

  public void periodic() {
    DogLog.log("nearest", EagleUtil.closestReefSetPoint(drivetrain.getPose(), 0));
    robotVisualizer.update();
    cam3.updatePoseEstim();
    cam4.updatePoseEstim();
    DogLog.log("Desired Reef", coralLevel);
    DogLog.log("Canivore Bus Utilization", (TunerConstants.kCANBus.getStatus()).BusUtilization);
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
    autoChooser.addOption(
        "Wheel_Radius_Chracterizaton",
        WheelRadiusCharacterization.wheelRadiusCharacterization(drivetrain));

    SmartDashboard.putData("autonomous", autoChooser);
  }

  public Command alignToPose(Supplier<Pose2d> Pose) {
    return new AlignToPose(Pose, drivetrain, () -> elevator.getHeightMeters());
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
        .withName(
            "Prepare Score Coral; Elevator Height: " + elevatorHeight + " Arm Angle: " + armAngle);
  }

  // scores coral
  public Command scoreCoral() {
    return Commands.sequence(
            arm.setAngle(ArmConstants.ARM_INTAKE_ANGLE).withTimeout(1),
            elevator.setHeight(ElevatorConstants.STOW_METER).withTimeout(0.5))
        .withName("Score Coral");
  }
}
