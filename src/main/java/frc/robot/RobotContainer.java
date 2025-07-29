// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;

import com.ctre.phoenix6.CANBus.CANBusStatus;
import com.pathplanner.lib.commands.PathfindingCommand;
import dev.doglog.DogLog;
import edu.wpi.first.hal.HALUtil;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.DriveCommand;
import frc.robot.commands.WheelRadiusCharacterization;
import frc.robot.commands.autonomous.*;
import frc.robot.generated.TunerConstants_Comp;
import frc.robot.generated.TunerConstants_WALLE;
import frc.robot.generated.TunerConstants_practiceDrivetrain;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.aprilTagCam.AprilTagCam;
import frc.robot.subsystems.aprilTagCam.AprilTagCamConstants;
import frc.robot.subsystems.arm.ArmConstants;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.climb.ClimbSubsystem;
import frc.robot.subsystems.elevator.ElevatorConstants;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.endEffector.EndEffectorSubsystem;
import frc.robot.subsystems.groundIntake.GroundIntakeSubsystem;
import java.util.function.BiConsumer;

public class RobotContainer {

  private static Alert roborioError =
      new Alert(
          "roborio unrecognized. here is the serial number:" + RobotController.getSerialNumber(),
          Alert.AlertType.kError);

  public enum Robot {
    WALLE,
    DEV,
    COMP
  }

  public enum RobotState {
    IDLE,
    INTAKE,
    PREPSCORE
  }

  public static RobotState robotState = RobotState.IDLE;

  public static final Trigger IS_IDLE = new Trigger(() -> robotState == RobotState.IDLE);
  public static final Trigger IS_INTAKE = new Trigger(() -> robotState == RobotState.INTAKE);
  public static final Trigger IS_PREPSCORE = new Trigger(() -> robotState == RobotState.PREPSCORE);

  public static Robot getRobot() {
    if (RobotController.getSerialNumber().equals("032414F0")) {
      return Robot.COMP;
    } else if (RobotController.getSerialNumber().equals("0323CA18")) {
      return Robot.DEV;
    } else if (RobotController.getSerialNumber().equals("03223849")) {
      return Robot.WALLE;
    } else {
      roborioError.set(true);
      return Robot.COMP;
    }
  }

  private final CommandXboxController m_driverController = new CommandXboxController(0);
  private final CommandXboxController m_operatorController = new CommandXboxController(1);
  private final Telemetry logger =
      new Telemetry(TunerConstants_Comp.kSpeedAt12Volts.in(MetersPerSecond));

  private final CommandSwerveDrivetrain drivetrain;
  private final ElevatorSubsystem elevator = new ElevatorSubsystem();
  private final ArmSubsystem arm = new ArmSubsystem();
  private final EndEffectorSubsystem endEffector = new EndEffectorSubsystem();
  // private final LedSubsystem led = new LedSubsystem();
  private final GroundIntakeSubsystem groundIntake = new GroundIntakeSubsystem();
  private final ClimbSubsystem climb = new ClimbSubsystem();
  private final DriveCommand driveCommand;

  private final CommandFactory commandFactory;

  public enum CoralLevel {
    L1(ElevatorConstants.L1_PREP_POSITION, ArmConstants.L1_PREP_POSITION),
    L2(ElevatorConstants.L2_PREP_POSITION, ArmConstants.L2_PREP_POSITION),
    L3(ElevatorConstants.L3_PREP_POSITION, ArmConstants.L3_PREP_POSITION),
    L4(ElevatorConstants.L4_PREP_POSITION, ArmConstants.L4_PREP_POSITION);

    public final double elevatorHeight;
    public final double armAngle;

    private CoralLevel(double elevatorHeight, double armAngle) {
      this.elevatorHeight = elevatorHeight;
      this.armAngle = armAngle;
    }
  }

  public static CoralLevel coralLevel = CoralLevel.L4;

  public static final Trigger IS_L1 = new Trigger(() -> coralLevel == CoralLevel.L1);
  public static final Trigger IS_L2 = new Trigger(() -> coralLevel == CoralLevel.L2);
  public static final Trigger IS_L3 = new Trigger(() -> coralLevel == CoralLevel.L3);
  public static final Trigger IS_L4 = new Trigger(() -> coralLevel == CoralLevel.L4);
  public static final Trigger IS_DISABLED = new Trigger(() -> DriverStation.isDisabled());
  public static final Trigger IS_TELEOP = new Trigger(() -> DriverStation.isTeleopEnabled());
  public static final Trigger BATTERY_BROWN_OUT = new Trigger(() -> RobotController.isBrownedOut());
  public static Trigger ALGAE_HIGH;

  public final Trigger IS_CORAL_LOADED;

  private final SendableChooser<Command> autoChooser = new SendableChooser<Command>();

  private AprilTagCam frontLeftCam;

  private AprilTagCam frontRightCam;

  private AprilTagCam backRightCam;

  private AprilTagCam elevatorCam;

  private final RobotVisualizer robotVisualizer = new RobotVisualizer(elevator, arm, groundIntake);

  private final BiConsumer<Runnable, Double> addPeriodic;

  public RobotContainer(BiConsumer<Runnable, Double> addPeriodic) {

    this.addPeriodic = addPeriodic;

    switch (getRobot()) {
      case COMP:
        drivetrain = TunerConstants_Comp.createDrivetrain();
        frontLeftCam =
            new AprilTagCam(
                AprilTagCamConstants.FRONT_LEFT_CAMERA_COMP_NAME,
                AprilTagCamConstants.FRONT_LEFT_CAMERA_LOCATION_COMP,
                drivetrain::addVisionMeasurent,
                () -> drivetrain.getPose(),
                () -> drivetrain.getState().Speeds);

        frontRightCam =
            new AprilTagCam(
                AprilTagCamConstants.FRONT_RIGHT_CAMERA_COMP_NAME,
                AprilTagCamConstants.FRONT_RIGHT_CAMERA_LOCATION_COMP,
                drivetrain::addVisionMeasurent,
                () -> drivetrain.getPose(),
                () -> drivetrain.getState().Speeds);

        backRightCam =
            new AprilTagCam(
                AprilTagCamConstants.BACK_RIGHT_CAMERA_COMP_NAME,
                AprilTagCamConstants.BACK_RIGHT_CAMERA_LOCATION_COMP,
                drivetrain::addVisionMeasurent,
                () -> drivetrain.getPose(),
                () -> drivetrain.getState().Speeds);

        // elevatorCam =
        //     new AprilTagCam(
        //         AprilTagCamConstants.ELEVATOR_CAMERA_COMP_NAME,
        //         AprilTagCamConstants.ELEVATOR_CAMERA_LOCATION_COMP,
        //         drivetrain::addVisionMeasurent,
        //         () -> drivetrain.getPose(),
        //         () -> drivetrain.getState().Speeds);

        break;
      case DEV:
        drivetrain = TunerConstants_practiceDrivetrain.createDrivetrain();
        frontLeftCam =
            new AprilTagCam(
                AprilTagCamConstants.FRONT_LEFT_CAMERA_DEV_NAME,
                AprilTagCamConstants.FRONT_LEFT_CAMERA_LOCATION_DEV,
                drivetrain::addVisionMeasurent,
                () -> drivetrain.getPose(),
                () -> drivetrain.getState().Speeds);

        frontRightCam =
            new AprilTagCam(
                AprilTagCamConstants.FRONT_RIGHT_CAMERA_DEV_NAME,
                AprilTagCamConstants.FRONT_RIGHT_CAMERA_LOCATION_DEV,
                drivetrain::addVisionMeasurent,
                () -> drivetrain.getPose(),
                () -> drivetrain.getState().Speeds);
        break;
      case WALLE:
        drivetrain = TunerConstants_WALLE.createDrivetrain();
        break;
      default:
        drivetrain = TunerConstants_Comp.createDrivetrain(); // Fallback
        break;
    }

    driveCommand =
        new DriveCommand(m_driverController, drivetrain, () -> elevator.getHeightMeters());

    ALGAE_HIGH = new Trigger(() -> EagleUtil.isHighAlgae(drivetrain.getPose()));
    IS_CORAL_LOADED = new Trigger(() -> endEffector.coralLoaded()).debounce(0.06);

    configureAutonomous();
    configureBindings();

    // Default Commands
    drivetrain.setDefaultCommand(driveCommand);

    drivetrain.registerTelemetry(logger::telemeterize);

    PathfindingCommand.warmupCommand().schedule();

    commandFactory =
        new CommandFactory(
            drivetrain,
            arm,
            elevator,
            endEffector,
            climb,
            groundIntake,
            m_driverController,
            m_operatorController,
            driveCommand);

    SmartDashboard.putData("Command Scheduler", CommandScheduler.getInstance());
    SmartDashboard.putData("Unprep Climb", commandFactory.unPrepClimbCommand());

    // Calculate reef setpoints at startup
    EagleUtil.calculateBlueReefSetPoints();
    EagleUtil.calculateRedReefSetPoints();

    addPeriodic.accept(
        () -> {
          CANBusStatus status = TunerConstants_Comp.kCANBus.getStatus();
          DogLog.log("Canivore/Canivore Bus Utilization", status.BusUtilization);
          DogLog.log("Canivore/Status Code on Canivore", status.Status.toString());
        },
        0.5);
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
    TriggerBindings.configureBindings(
        drivetrain,
        arm,
        elevator,
        endEffector,
        climb,
        groundIntake,
        m_driverController,
        m_operatorController,
        driveCommand,
        commandFactory);
  }

  public void periodic() {
    double startTime = HALUtil.getFPGATime();

    DogLog.log("nearest", EagleUtil.closestReefSetPoint(drivetrain.getPose(), 0));
    // 1
    DogLog.log(
        "Loop Time/Robot Container/Log Closest Reef Set Point",
        (HALUtil.getFPGATime() - startTime) / 1000);

    startTime = HALUtil.getFPGATime();

    robotVisualizer.update();
    // 2
    DogLog.log(
        "Loop Time/Robot Container/Robot Visualizer", (HALUtil.getFPGATime() - startTime) / 1000);

    startTime = HALUtil.getFPGATime();

    if (frontLeftCam != null) {
      frontLeftCam.updatePoseEstim();
      // 3
      DogLog.log("Loop Time/Robot Container/Cam3", (HALUtil.getFPGATime() - startTime) / 1000);

      startTime = HALUtil.getFPGATime();
    }
    if (frontRightCam != null) {
      frontRightCam.updatePoseEstim();
      // 4
      DogLog.log("Loop Time/Robot Container/Cam4", (HALUtil.getFPGATime() - startTime) / 1000);
    }
    if (backRightCam != null) {
      backRightCam.updatePoseEstim();
    }
    if (elevatorCam != null) {
      elevatorCam.updatePoseEstim();
    }

    startTime = HALUtil.getFPGATime();

    DogLog.log("Desired Reef", coralLevel);

    // Log Triggers
    DogLog.log("Trigger/At L1", IS_L1.getAsBoolean());
    DogLog.log("Trigger/At L2", IS_L2.getAsBoolean());
    DogLog.log("Trigger/At L3", IS_L3.getAsBoolean());
    DogLog.log("Trigger/At L4", IS_L4.getAsBoolean());
    DogLog.log("Trigger/Is Disabled", IS_DISABLED.getAsBoolean());
    DogLog.log("Trigger/Is Telop", IS_TELEOP.getAsBoolean());
    DogLog.log("Current Robot", getRobot().toString());

    DogLog.log("Trigger/Algae High", ALGAE_HIGH.getAsBoolean());
    DogLog.log("Trigger/Is Coral Loaded", IS_CORAL_LOADED.getAsBoolean());

    DogLog.log("Trigger/Elevator At Goal Height", elevator.AT_GOAL_HEIGHT.getAsBoolean());
    DogLog.log("Trigger/Arm At Goal Angle", arm.AT_GOAL_ANGLE.getAsBoolean());
    DogLog.log("Trigger/Is Prepscore", IS_PREPSCORE.getAsBoolean());

    DogLog.log("Match Timer", DriverStation.getMatchTime());
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
    autoChooser.setDefaultOption(
        "Five_Cycle_Processor", new FiveCycle(this, commandFactory, false, drivetrain));
    autoChooser.addOption(
        "Five_Cycle_Non_Processor", new FiveCycle(this, commandFactory, true, drivetrain));
    autoChooser.addOption(
        "Score_Preload_One_Cycle", new ScorePreloadOneCycle(this, commandFactory, drivetrain));
    autoChooser.addOption("Leave_Non_Processor", new LeaveNonProcessor(this));
    autoChooser.addOption("Leave_Processor", new LeaveProcessor(this));
    autoChooser.addOption("Push_One_Cycle", new PushOneCycle(this, commandFactory, drivetrain));
    autoChooser.addOption(
        "Wheel_Radius_Chracterizaton",
        WheelRadiusCharacterization.wheelRadiusCharacterization(drivetrain));
    autoChooser.addOption("Do Notion", Commands.none());

    SmartDashboard.putData("autonomous", autoChooser);
  }
}
