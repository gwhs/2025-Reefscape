package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;

import com.ctre.phoenix6.CANBus.CANBusStatus;
import com.pathplanner.lib.commands.PathfindingCommand;
import dev.doglog.DogLog;
import edu.wpi.first.hal.HALUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.AlignToPose;
import frc.robot.commands.DriveCommand;
import frc.robot.commands.DriveCommand.TargetMode;
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
import frc.robot.subsystems.endEffector.EndEffectorConstants;
import frc.robot.subsystems.endEffector.EndEffectorSubsystem;
import frc.robot.subsystems.groundIntake.GroundIntakeConstants;
import frc.robot.subsystems.groundIntake.GroundIntakeSubsystem;
import java.util.function.BiConsumer;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

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

  private boolean isPreppedClimb = false;

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
  private final Alert batteryUnderTwelveVolts = new Alert("BATTERY UNDER 12V", AlertType.kWarning);
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

  public enum CoralLevel {
    L1(ElevatorConstants.L1_PREP_POSITION, ArmConstants.L1_PREP_POSITION),
    L2(ElevatorConstants.L2_PREP_POSITION, ArmConstants.L2_PREP_POSITION),
    L3(ElevatorConstants.L3_PREP_POSITION, ArmConstants.L3_PREP_POSITION),
    L4(ElevatorConstants.L4_PREP_POSITION, ArmConstants.L4_PREP_POSITION),
    ALGAE(0, 0);

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
  public static final Trigger IS_ALGAE_MODE = new Trigger(() -> coralLevel == CoralLevel.ALGAE);
  public static final Trigger IS_DISABLED = new Trigger(() -> DriverStation.isDisabled());
  public static final Trigger IS_TELEOP = new Trigger(() -> DriverStation.isTeleopEnabled());
  public static final Trigger BATTERY_BROWN_OUT = new Trigger(() -> RobotController.isBrownedOut());
  public static Trigger ALGAE_HIGH;
  public final Trigger IS_NEAR_CORAL_STATION;

  public final Trigger IS_CORAL_LOADED;

  private final SendableChooser<Command> autoChooser = new SendableChooser<Command>();

  private final Trigger IS_REEF_MODE;

  private final Trigger IS_CLOSE_TO_REEF;

  private final Trigger IS_CONTROLLER_LEFT;

  private final Trigger IS_CONTROLLER_RIGHT;

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

    IS_REEF_MODE = new Trigger(() -> driveCommand.getTargetMode() == TargetMode.REEF);

    IS_CLOSE_TO_REEF =
        new Trigger(
            () ->
                EagleUtil.getDistanceBetween(
                        drivetrain.getPose(), EagleUtil.getCachedReefPose(drivetrain.getPose()))
                    < 1.25);

    IS_NEAR_CORAL_STATION =
        new Trigger(
            () ->
                EagleUtil.getDistanceBetween(
                        drivetrain.getPose(), EagleUtil.getClosetStationGen(drivetrain.getPose()))
                    < 0.4);

    IS_CONTROLLER_LEFT = new Trigger(() -> m_driverController.getLeftX() <= -0.5);

    IS_CONTROLLER_RIGHT = new Trigger(() -> m_driverController.getLeftX() >= 0.5);

    ALGAE_HIGH = new Trigger(() -> EagleUtil.isHighAlgae(getRobotPose()));
    IS_CORAL_LOADED = new Trigger(() -> endEffector.coralLoaded()).debounce(0.06);

    configureAutonomous();
    configureBindings();

    // Default Commands
    drivetrain.setDefaultCommand(driveCommand);

    drivetrain.registerTelemetry(logger::telemeterize);

    PathfindingCommand.warmupCommand().schedule();

    SmartDashboard.putData("Command Scheduler", CommandScheduler.getInstance());
    SmartDashboard.putData("Unprep Climb", unPrepClimbCommand());
    SmartDashboard.putData("Home Elevator", zeroElevator());
    // SmartDashboard.putData("Arm Angle Increase", arm.increaseAngle(3.0));
    // SmartDashboard.putData("Arm Angle Decrease", arm.decreaseAngle(3.0));
    // SmartDashboard.putData("Elevator Increase", elevator.increaseHeight(0.02));
    // SmartDashboard.putData("Elevator Decrease", elevator.decreaseHeight(0.02));
    // SmartDashboard.putData("Ground Intake UP", groundIntake.increaseAngle(3));
    // SmartDashboard.putData("Ground Intake DOWN", groundIntake.decreaseAngle(3));

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
    // BATTERY_BROWN_OUT.onTrue(drivetrain.setDriveMotorCurrentLimit());

    // drivetrain
    //     .IS_ALIGNING_TO_POSE
    //     .and(drivetrain.IS_AT_TARGET_POSE)
    //     .onTrue(led.setPattern(LEDPattern.solid(Color.kGreen)));
    // drivetrain
    //     .IS_ALIGNING_TO_POSE
    //     .and(drivetrain.IS_AT_TARGET_POSE.negate())
    //     .onTrue(led.setPattern(LEDPattern.solid(Color.kBlack)));

    IS_DISABLED.onTrue(
        Commands.runOnce(
                () -> {
                  // drivetrain.configNeutralMode(NeutralModeValue.Coast);
                  // elevator.setNeutralMode(NeutralModeValue.Coast);
                  driveCommand.stopDrivetrain();
                })
            .ignoringDisable(true));

    IS_CONTROLLER_LEFT
        .and(m_driverController.a().or(m_driverController.b()).or(m_driverController.y()))
        .onTrue(alignToPose(() -> EagleUtil.getClosestLeftReef(drivetrain.getPose(0.25))));

    m_driverController
        .a()
        .or(m_driverController.b())
        .or(m_driverController.y())
        .onFalse(driveCommand);

    IS_CONTROLLER_RIGHT
        .and(m_driverController.a().or(m_driverController.b()).or(m_driverController.y()))
        .onTrue(alignToPose(() -> EagleUtil.getClosestRightReef(drivetrain.getPose(0.25))));

    m_driverController
        .x()
        .whileTrue(
            prepCoralIntake(
                ElevatorConstants.INTAKE_METER_BACKUP, ArmConstants.ARM_INTAKE_ANGLE_BACKUP))
        .onFalse(stopIntake());

    // m_driverController
    //     .leftBumper()
    //     .onTrue(
    //         Commands.runOnce(() -> driveCommand.setTargetMode(DriveCommand.TargetMode.NORMAL))
    //             .withName("Back to Original State"));

    // make this force release

    m_driverController.povDown().whileTrue(deployGroundIntake()).onFalse(retractGroundIntake());

    IS_L1.and(m_driverController.rightTrigger()).onTrue(scoreGroundIntake());

    m_driverController
        .rightTrigger()
        .onFalse(
            Commands.either(scoreCoral(), scoreAlgeaNet(), IS_L2.or(IS_L3).or(IS_L4))
                .onlyIf(IS_L2.or(IS_L3).or(IS_L4).or(IS_ALGAE_MODE))
                .withInterruptBehavior(InterruptionBehavior.kCancelIncoming)
                .withName("score on trigger release"));

    m_driverController
        .rightTrigger()
        .negate()
        .and(ALGAE_HIGH.negate())
        .and(m_driverController.leftTrigger())
        .onTrue(prepDealgaeLow());

    m_driverController
        .rightTrigger()
        .negate()
        .and(ALGAE_HIGH)
        .and(m_driverController.leftTrigger())
        .onTrue(prepDealgaeHigh());

    m_driverController
        .rightTrigger()
        .negate()
        .and(m_driverController.leftTrigger())
        .whileTrue(alignToPose(() -> EagleUtil.getNearestAlgaePoint(drivetrain.getPose())))
        .whileTrue(
            Commands.startEnd(
                    () -> {
                      driveCommand.setDriveMode(DriveCommand.DriveMode.ROBOT_CENTRIC);
                    },
                    () -> {
                      driveCommand.setDriveMode(DriveCommand.DriveMode.FIELD_CENTRIC);
                    })
                .withName("DeALgae Robot Centric"));

    m_driverController.leftTrigger().onFalse(dealgae());

    m_driverController
        .leftBumper()
        .onTrue(
            Commands.runOnce(
                () -> {
                  driveCommand.setTargetMode(DriveCommand.TargetMode.NORMAL);
                }));

    IS_L4
        .and(m_driverController.y())
        .whileTrue(prepScoreCoral(CoralLevel.L4))
        .onFalse(stowArmAndElevator());
    IS_L3
        .and(m_driverController.b())
        .whileTrue(prepScoreCoral(CoralLevel.L3))
        .onFalse(stowArmAndElevator());
    IS_L2
        .and(m_driverController.a())
        .whileTrue(prepScoreCoral(CoralLevel.L2))
        .onFalse(stowArmAndElevator());

    IS_L2
        .or(IS_L3)
        .or(IS_L4)
        .and(IS_REEF_MODE)
        .onTrue(
            Commands.runOnce(
                () -> {
                  driveCommand.setTargetMode(DriveCommand.TargetMode.REEF);
                  driveCommand.setReefMode(DriveCommand.ReefPositions.FRONT_REEF);
                }));

    m_driverController
        .rightBumper()
        .whileTrue(
            Commands.startEnd(
                    () -> {
                      driveCommand.setSlowMode(true, 0.25);
                    },
                    () -> {
                      driveCommand.setSlowMode(false, 0.25);
                    })
                .withName("Slow Mode"));

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

    drivetrain
        .IS_AT_TARGET_POSE
        .and(drivetrain.IS_ALIGNING_TO_POSE)
        .and(IS_PREPSCORE)
        .and(elevator.AT_GOAL_HEIGHT)
        .and(arm.AT_GOAL_ANGLE)
        .and(IS_TELEOP)
        .debounce(0.33)
        .onTrue(scoreCoral());

    // IS_L2
    //     .and(m_driverController.a())
    //     .whileTrue(alignToPose(() ->
    // EagleUtil.getClosestLeftReefBack(drivetrain.getPose(0.25))));

    m_driverController
        .b()
        .or(m_driverController.y())
        .or(m_driverController.a())
        .whileTrue(alignToPose(() -> EagleUtil.getClosestRightReef(drivetrain.getPose(0.25))));

    // IS_L4
    //     .or(IS_L3)
    //     .or(IS_L2)
    //     .and(m_driverController.b())
    //     .whileTrue(alignToPose(() -> EagleUtil.getClosestRightReef(drivetrain.getPose(0.25))));

    // IS_L2
    //     .and(m_driverController.b())
    //     .whileTrue(alignToPose(() ->
    // EagleUtil.getClosestRightReefBack(drivetrain.getPose(0.25))));

    IS_L1
        .and(m_driverController.b())
        .whileTrue(alignToPose(() -> EagleUtil.getClosestRightReefBack(drivetrain.getPose(0.25))));

    m_driverController
        .povLeft()
        .whileTrue(
            alignToPose(() -> EagleUtil.getClosestCoralStation(this.getRobotPose()))); // TODO

    // m_driverController
    //     .x()
    //     .onTrue(
    //         Commands.runOnce(
    //             () -> {
    //               coralLevel = CoralLevel.L1;
    //               driveCommand.setInverted(true);
    //             }));

    m_driverController
        .a()
        .onTrue(
            Commands.runOnce(
                () -> {
                  coralLevel = CoralLevel.L2;
                  driveCommand.setInverted(false);
                  coralLevel = CoralLevel.L2;
                }));

    m_driverController
        .a()
        .onFalse(
            groundIntake
                .setAngleAndAmp(GroundIntakeConstants.CORAL_STOW_ANGLE, 0, 0)
                .onlyIf(IS_L2));

    m_driverController
        .b()
        .onTrue(
            Commands.runOnce(
                () -> {
                  coralLevel = CoralLevel.L3;
                  driveCommand.setInverted(false);
                }));

    m_driverController
        .y()
        .onTrue(
            Commands.runOnce(
                () -> {
                  coralLevel = CoralLevel.L4;
                  driveCommand.setInverted(false);
                }));

    m_driverController.start().onTrue(climb());
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
    DogLog.log("Trigger/Is Close to Reef", IS_CLOSE_TO_REEF.getAsBoolean());
    DogLog.log("Current Robot", getRobot().toString());
    DogLog.log("Trigger/Is Reefmode", IS_REEF_MODE.getAsBoolean());

    DogLog.log("Trigger/Algae High", ALGAE_HIGH.getAsBoolean());
    DogLog.log("Trigger/Is Near Coarl Station", IS_NEAR_CORAL_STATION.getAsBoolean());
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
        "Five_Cycle_Processor", new FiveCycle(this, groundIntake, climb, false));
    autoChooser.addOption(
        "Five_Cycle_Non_Processor", new FiveCycle(this, groundIntake, climb, true));
    autoChooser.addOption(
        "Choreo_Five_Cycle_Non_Processor", new ChoreoFiveCycle(this, groundIntake, climb, true));
    autoChooser.addOption("Score_Preload_One_Cycle", new ScorePreloadOneCycle(this));
    autoChooser.addOption("Leave_Non_Processor", new LeaveNonProcessor(this));
    autoChooser.addOption("Leave_Processor", new LeaveProcessor(this));
    autoChooser.addOption("Push_One_Cycle", new PushOneCycle(this, groundIntake, climb));
    autoChooser.addOption(
        "Wheel_Radius_Chracterizaton",
        WheelRadiusCharacterization.wheelRadiusCharacterization(drivetrain));
    autoChooser.addOption("Do Notion", Commands.none());

    SmartDashboard.putData("autonomous", autoChooser);
  }

  public Pose2d getRobotPose() {
    return drivetrain.getPose();
  }

  public Command zeroElevator() {
    return elevator.homingCommand();
  }

  /**
   * this is a wrapper for the command of the same name
   *
   * @param Pose pose to go to
   * @return run the command
   */
  public Command alignToPose(Supplier<Pose2d> Pose) {
    return new AlignToPose(Pose, drivetrain, () -> elevator.getHeightMeters(), m_driverController);
  }

  /**
   * @return prep to pickup coral
   */
  public Command prepCoralIntake(double elevatorHeight, double armAngle) {
    return Commands.parallel(
            Commands.runOnce(
                () -> {
                  driveCommand.setTargetMode(DriveCommand.TargetMode.CORAL_STATION);
                }),
            endEffector.intake(),
            elevator.setHeight(elevatorHeight).withTimeout(0.5),
            arm.setAngle(armAngle).withTimeout(1),
            Commands.runOnce(() -> robotState = RobotState.INTAKE))
        .withName("Prepare Coral Intake");
  }

  public Command prepCoralIntakeAuton() {
    return Commands.parallel(
            endEffector.intake(),
            elevator.setHeight(ElevatorConstants.INTAKE_METER_AUTON).withTimeout(0.5),
            arm.setAngle(ArmConstants.ARM_INTAKE_ANGLE).withTimeout(1))
        .withName("Prepare Coral Intake Auton");
  }

  public Command stopIntake() {
    return Commands.parallel(
            Commands.runOnce(
                () -> {
                  driveCommand.setReefMode(DriveCommand.ReefPositions.FRONT_REEF);
                  driveCommand.setTargetMode(DriveCommand.TargetMode.REEF);
                }),
            arm.setAngle(ArmConstants.ARM_STOW_ANGLE),
            elevator.setHeight(ElevatorConstants.STOW_METER),
            endEffector.holdCoral(),
            Commands.runOnce(() -> robotState = RobotState.IDLE))
        .withName("stop Intake");
  }

  /**
   * @param elevatorHeight how tall should the elavator be?
   * @param armAngle what angle should the arm be at
   * @return run the command
   */
  public Command prepScoreCoral(double elevatorHeight, double armAngle) {
    return Commands.sequence(
            groundIntake
                .setAngleAndAmp(GroundIntakeConstants.INTAKE_CORAL_ANGLE, 0, 0)
                .onlyIf(IS_L2),
            Commands.parallel(
                endEffector.holdCoral(),
                elevator.setHeight(elevatorHeight).withTimeout(1.5),
                arm.setAngle(armAngle).withTimeout(1.5),
                Commands.runOnce(() -> robotState = RobotState.PREPSCORE)),
            groundIntake.setAngleAndAmp(GroundIntakeConstants.CORAL_STOW_ANGLE, 0, 0).onlyIf(IS_L2))
        .withName(
            "Prepare Score Coral; Elevator Height: " + elevatorHeight + " Arm Angle: " + armAngle);
  }

  public Command prepScoreCoral(DoubleSupplier elevatorHeight, DoubleSupplier armAngle) {
    return Commands.sequence(
        groundIntake
            .setAngleAndAmp(GroundIntakeConstants.INTAKE_CORAL_ANGLE + 100, 0, 0)
            .onlyIf(IS_L2),
        Commands.parallel(
                Commands.runOnce(
                    () -> driveCommand.setTargetMode(DriveCommand.TargetMode.REEF_FACES)),
                endEffector.holdCoral(),
                elevator.setHeightSupplier(elevatorHeight).withTimeout(.5),
                arm.setAngleSupplier(armAngle).withTimeout(.5),
                Commands.runOnce(() -> robotState = RobotState.PREPSCORE))
            .withName(
                "Prepare Score Coral; Elevator Height: "
                    + elevatorHeight
                    + " Arm Angle: "
                    + armAngle));
  }

  public Command stowArmAndElevator() {
    return Commands.parallel(
            arm.setAngle(ArmConstants.ARM_STOW_ANGLE).withTimeout(1.0),
            elevator.setHeight(ElevatorConstants.STOW_METER).withTimeout(1.0),
            endEffector.holdCoral())
        .withName("Stow Arm and Elevator");
  }

  public Command prepScoreCoral(CoralLevel level) {
    DoubleSupplier elevatorHeightSupplier =
        () -> EagleUtil.getOffsetElevatorHeight(level, drivetrain.getPose());
    DoubleSupplier armAngleSupplier =
        () -> EagleUtil.getOffsetArmAngle(level, drivetrain.getPose());
    return prepScoreCoral(elevatorHeightSupplier, armAngleSupplier).repeatedly();
  }

  public Command autonScoreCoral() {
    return Commands.sequence(
        endEffector.shoot(EndEffectorConstants.VOLTAGE_L4), Commands.waitSeconds(0.05));
  }

  /**
   * @return score the coral
   */
  public Command scoreCoral() {
    Command scoreCoral =
        Commands.sequence(
                Commands.runOnce(
                    () -> driveCommand.setTargetMode(DriveCommand.TargetMode.REEF_FACES)),
                endEffector.shoot(EndEffectorConstants.VOLTAGE_L4).onlyIf(IS_L4),
                endEffector.shoot(EndEffectorConstants.VOLTAGE_L3).onlyIf(IS_L3),
                endEffector.shoot(EndEffectorConstants.VOLTAGE_L2).onlyIf(IS_L2),
                endEffector.shoot(EndEffectorConstants.VOLTAGE_L1).onlyIf(IS_L1),
                Commands.waitSeconds(0.05),
                drivetrain.driveBackward(1).withTimeout(0.2).onlyIf(IS_L2),
                arm.setAngle(ArmConstants.ARM_STOW_ANGLE).withTimeout(0.0),
                elevator.setHeight(ElevatorConstants.STOW_METER).withTimeout(0.0),
                endEffector.stopMotor(),
                Commands.runOnce(() -> driveCommand.setTargetMode(DriveCommand.TargetMode.REEF)))
            .withTimeout(0.5);

    Command deAlgae =
        Commands.sequence(
                endEffector.shoot(EndEffectorConstants.VOLTAGE_L4).onlyIf(IS_L4),
                endEffector.shoot(EndEffectorConstants.VOLTAGE_L3).onlyIf(IS_L3),
                endEffector.shoot(EndEffectorConstants.VOLTAGE_L2).onlyIf(IS_L2),
                endEffector.shoot(EndEffectorConstants.VOLTAGE_L1).onlyIf(IS_L1),
                Commands.waitSeconds(0.04), // .05
                endEffector.stopMotor(),
                alignToPose(() -> EagleUtil.getNearestAlgaePoint(drivetrain.getPose()))
                    .withTimeout(0.6)
                    .alongWith(
                        arm.setAngle(90)
                            .alongWith(
                                Commands.either(
                                    elevator.setHeight(ElevatorConstants.DEALGAE_HIGH_POSITION),
                                    elevator.setHeight(ElevatorConstants.DEALGAE_LOW_POSITION),
                                    ALGAE_HIGH))
                            .withTimeout(0.4)), // .5
                Commands.either(prepDealgaeHigh(), prepDealgaeLow(), ALGAE_HIGH)
                    .withTimeout(.1), // .6
                Commands.runOnce(() -> driveCommand.setTargetMode(DriveCommand.TargetMode.REEF))
                    .deadlineFor(
                        alignToPose(() -> EagleUtil.getNearestAlgaePoint(drivetrain.getPose()))),
                dealgae())
            .withInterruptBehavior(InterruptionBehavior.kCancelIncoming);

    return Commands.sequence(
        Commands.either(deAlgae, scoreCoral, m_driverController.leftTrigger())
            .withName("Score Coral/deAlgae"),
        Commands.runOnce(() -> robotState = RobotState.IDLE)
            .withInterruptBehavior(InterruptionBehavior.kCancelIncoming));
  }

  // DeAlgae Commands
  public Command prepDealgaeLow() {
    return Commands.parallel(
            Commands.runOnce(() -> driveCommand.setTargetMode(DriveCommand.TargetMode.REEF_FACES)),
            elevator.setHeight(ElevatorConstants.DEALGAE_LOW_POSITION),
            arm.setAngle(ArmConstants.PRE_DEALGAE_ANGLE),
            endEffector.setVoltage(0))
        .withName("prep Delalgae low");
  }

  public Command prepDealgaeHigh() {
    return Commands.parallel(
            Commands.runOnce(() -> driveCommand.setTargetMode(DriveCommand.TargetMode.REEF_FACES)),
            elevator.setHeight(ElevatorConstants.DEALGAE_HIGH_POSITION),
            arm.setAngle(ArmConstants.PRE_DEALGAE_ANGLE),
            endEffector.setVoltage(0))
        .withName("prep Dealgae high");
  }

  public Command dealgae() {
    return Commands.sequence(
            arm.setAngle(ArmConstants.DEALGAE_ANGLE)
                .alongWith(elevator.decreaseHeight(0.1))
                .withTimeout(0.2),
            drivetrain.driveBackward(1).withTimeout(0.6),
            Commands.parallel(
                elevator.setHeight(ElevatorConstants.STOW_METER).withTimeout(.1),
                arm.setAngle(ArmConstants.ARM_STOW_ANGLE).withTimeout(.1),
                endEffector.stopMotor()),
            Commands.runOnce(() -> driveCommand.setTargetMode(DriveCommand.TargetMode.REEF)))
        .withName("Dealgae");
  }

  public Command unPrepClimbCommand() {
    return Commands.sequence(
            arm.setAngle(ArmConstants.CLIMB_ANGLE).withTimeout(1),
            Commands.runOnce(() -> driveCommand.setTargetMode(DriveCommand.TargetMode.REEF)),
            climb.stow().withTimeout(5),
            elevator.setHeight(ElevatorConstants.STOW_METER + 0.1).withTimeout(1),
            arm.setAngle(ArmConstants.ARM_STOW_ANGLE))
        .withInterruptBehavior(InterruptionBehavior.kCancelIncoming)
        .withName("unPrepClimb");
  }

  public Command climb() {
    Command climbCommand =
        Commands.parallel(
                climb.climb(),
                elevator.setHeight(0),
                arm.setAngle(ArmConstants.CLIMB_ANGLE),
                Commands.runOnce(() -> driveCommand.setTargetMode(DriveCommand.TargetMode.NORMAL)))
            .withTimeout(0.5)
            .andThen(Commands.runOnce(() -> isPreppedClimb = false))
            .withName("Climb");

    Command prepClimb =
        Commands.sequence(
                Commands.runOnce(() -> driveCommand.setTargetMode(DriveCommand.TargetMode.CAGE)),
                endEffector.stopMotor(),
                groundIntake.setAngleAndAmp(GroundIntakeConstants.CLIMB_ANGLE, 0, 0).withTimeout(1),
                arm.setAngle(ArmConstants.PREP_CLIMB_ANGLE).withTimeout(1),
                elevator.setHeight(0).withTimeout(1),
                climb.latch().withTimeout(1),
                Commands.runOnce(() -> isPreppedClimb = true))
            .withName("Prep Climb");

    return Commands.either(climbCommand, prepClimb, () -> isPreppedClimb)
        .withInterruptBehavior(InterruptionBehavior.kCancelIncoming)
        .withName("Climb Sequence");
  }

  public Command deployGroundIntake() {
    return Commands.parallel(
        Commands.runOnce(
            () -> {
              driveCommand.setTargetMode(DriveCommand.TargetMode.NORMAL);
              coralLevel = CoralLevel.L1;
            }),
        endEffector.stopMotor(),
        groundIntake
            .setAngleAndAmp(
                GroundIntakeConstants.INTAKE_CORAL_ANGLE,
                GroundIntakeConstants.INTAKE_CORAL_AMP,
                GroundIntakeConstants.INTAKE_CORAL_DUTYCYCLE)
            .withName("Ground Intake Extend"));
  }

  public Command retractGroundIntake() {
    return Commands.parallel(
        Commands.runOnce(
            () -> {
              driveCommand.setTargetMode(DriveCommand.TargetMode.REEF_FACES);
              driveCommand.setReefMode(DriveCommand.ReefPositions.BACK_REEF);
            }),
        groundIntake
            .setAngleAndAmp(
                GroundIntakeConstants.CORAL_STOW_ANGLE,
                GroundIntakeConstants.HOLD_CORAL_AMP,
                GroundIntakeConstants.HOLD_CORAL_DUTYCYCLE)
            .withName("Ground Intake Extend"));
  }

  public Command scoreGroundIntake() {
    return Commands.sequence(
        groundIntake.setAngleAndAmp(
            GroundIntakeConstants.SCORE_CORAL_ANGLE,
            GroundIntakeConstants.HOLD_CORAL_AMP,
            GroundIntakeConstants.HOLD_CORAL_DUTYCYCLE),
        Commands.waitUntil(m_driverController.rightTrigger().negate()),
        Commands.waitSeconds(0.3),
        groundIntake.setAngleAndAmp(
            GroundIntakeConstants.SCORE_CORAL_ANGLE,
            GroundIntakeConstants.SCORE_CORAL_AMP,
            GroundIntakeConstants.SCORE_CORAL_DUTYCYCLE),
        Commands.waitSeconds(0.5),
        groundIntake.setAngleAndAmp(GroundIntakeConstants.CORAL_STOW_ANGLE, 0, 0),
        Commands.runOnce(
            () -> {
              driveCommand.setTargetMode(DriveCommand.TargetMode.NORMAL);
            }));
  }

  public Command scoreAlgeaNet() {
    return Commands.none();
    // TODO
  }
}
