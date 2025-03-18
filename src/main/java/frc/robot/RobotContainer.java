// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

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
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
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
  public final Trigger IS_NEAR_CORAL_STATION;

  public final Trigger IS_CORAL_LOADED;

  private final SendableChooser<Command> autoChooser = new SendableChooser<Command>();

  private final Trigger IS_REEF_MODE;

  private final Trigger IS_CLOSE_TO_REEF;

  private AprilTagCam leftCam;

  private AprilTagCam rightCam;

  private final RobotVisualizer robotVisualizer = new RobotVisualizer(elevator, arm, groundIntake);

  private final BiConsumer<Runnable, Double> addPeriodic;

  public RobotContainer(BiConsumer<Runnable, Double> addPeriodic) {

    this.addPeriodic = addPeriodic;

    switch (getRobot()) {
      case COMP:
        drivetrain = TunerConstants_Comp.createDrivetrain();
        leftCam =
            new AprilTagCam(
                AprilTagCamConstants.FRONT_LEFT_CAMERA_COMP_NAME,
                AprilTagCamConstants.FRONT_LEFT_CAMERA_LOCATION_COMP,
                drivetrain::addVisionMeasurent,
                () -> drivetrain.getState().Pose,
                () -> drivetrain.getState().Speeds);

        rightCam =
            new AprilTagCam(
                AprilTagCamConstants.FRONT_RIGHT_CAMERA_COMP_NAME,
                AprilTagCamConstants.FRONT_RIGHT_CAMERA_LOCATION_COMP,
                drivetrain::addVisionMeasurent,
                () -> drivetrain.getState().Pose,
                () -> drivetrain.getState().Speeds);
        break;
      case DEV:
        drivetrain = TunerConstants_practiceDrivetrain.createDrivetrain();
        leftCam =
            new AprilTagCam(
                AprilTagCamConstants.FRONT_LEFT_CAMERA_DEV_NAME,
                AprilTagCamConstants.FRONT_LEFT_CAMERA_LOCATION_DEV,
                drivetrain::addVisionMeasurent,
                () -> drivetrain.getState().Pose,
                () -> drivetrain.getState().Speeds);

        rightCam =
            new AprilTagCam(
                AprilTagCamConstants.FRONT_RIGHT_CAMERA_DEV_NAME,
                AprilTagCamConstants.FRONT_RIGHT_CAMERA_LOCATION_DEV,
                drivetrain::addVisionMeasurent,
                () -> drivetrain.getState().Pose,
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

    ALGAE_HIGH = new Trigger(() -> EagleUtil.isHighAlgae(getRobotPose()));
    IS_CORAL_LOADED = new Trigger(() -> endEffector.coralLoaded());

    configureAutonomous();
    configureBindings();

    // Default Commands
    drivetrain.setDefaultCommand(driveCommand);

    drivetrain.registerTelemetry(logger::telemeterize);

    PathfindingCommand.warmupCommand().schedule();

    SmartDashboard.putData("Command Scheduler", CommandScheduler.getInstance());

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

    // IS_DISABLED
    //     .and(() -> RobotController.getBatteryVoltage() >= 12)
    //     .onTrue(led.setPattern(LEDPattern.solid(Color.kGreen)));

    // IS_DISABLED
    //     .and(() -> RobotController.getBatteryVoltage() < 12)
    //     .onTrue(led.setPattern(LEDPattern.solid(Color.kRed)));

    IS_DISABLED.onFalse(
        Commands.runOnce(
                () -> {
                  // drivetrain.configNeutralMode(NeutralModeValue.Brake);
                  // elevator.setNeutralMode(NeutralModeValue.Brake);
                })
            .andThen(groundIntake.setAngleAndVoltage(GroundIntakeConstants.CORAL_STOW_ANGLE, 0))
            .ignoringDisable(false));

    // IS_DISABLED
    //     .and(() -> RobotController.getBatteryVoltage() < 12)
    //     .onTrue(EagleUtil.triggerAlert(batteryUnderTwelveVolts));

    m_driverController
        .x()
        .whileTrue(
            Commands.startEnd(
                    () -> driveCommand.setTargetMode(DriveCommand.TargetMode.CORAL_STATION),
                    () -> {
                      driveCommand.setTargetMode(DriveCommand.TargetMode.REEF);
                      driveCommand.setReefMode(DriveCommand.ReefPositions.FRONT_REEF);
                    })
                .withName("Face Coral Station"));

    // m_driverController
    //     .x()
    //     .and(IS_NEAR_CORAL_STATION)
    //     .onTrue(Commands.runOnce(() -> driveCommand.setSlowMode(true, 0.25)))
    //     .onFalse(Commands.runOnce(() -> driveCommand.setSlowMode(false, 0)));

    m_driverController.x().whileTrue(prepCoralIntake()).onFalse(stopIntake());
    m_driverController
        .y()
        .whileTrue(
            prepCoralIntake(
                ElevatorConstants.INTAKE_METER_BACKUP, ArmConstants.ARM_INTAKE_ANGLE_BACKUP))
        .onFalse(stopIntake());

    // IS_TELEOP
    //     .and(IS_REEFMODE)
    //     .and(IS_CLOSE_TO_REEF)
    //     .onTrue(
    //         prepScoreCoral(ElevatorConstants.STOW_METER, 220).withName("auto prep score coral"));

    m_driverController
        .leftBumper()
        .onTrue(
            Commands.runOnce(() -> driveCommand.setTargetMode(DriveCommand.TargetMode.NORMAL))
                .withName("Back to Original State"));

    m_driverController
        .rightTrigger()
        .onFalse(
            Commands.either(scoreCoral(), groundIntakeScoreL1(), IS_L1.negate())
                .withInterruptBehavior(InterruptionBehavior.kCancelIncoming)
                .withName("score Coral"));

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
        .whileTrue(alignToPose(() -> EagleUtil.getNearestAlgaePoint(drivetrain.getState().Pose)))
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

    IS_L4.and(m_driverController.rightTrigger()).whileTrue(prepScoreCoral(CoralLevel.L4));
    IS_L3.and(m_driverController.rightTrigger()).whileTrue(prepScoreCoral(CoralLevel.L3));
    IS_L2.and(m_driverController.rightTrigger()).whileTrue(prepScoreCoral(CoralLevel.L2));
    IS_L1
        .and(m_driverController.rightTrigger())
        .whileTrue(groundIntake.setAngleAndVoltage(GroundIntakeConstants.SCORE_CORAL_ANGLE, -1));

    m_operatorController
        .leftStick()
        .whileTrue(groundIntake.setAngleAndVoltage(GroundIntakeConstants.INTAKE_ALGAE_ANGLE, 6))
        .onFalse(groundIntake.setAngleAndVoltage(GroundIntakeConstants.ALGAE_STOW_ANGLE, 1));

    m_operatorController
        .rightStick()
        .whileTrue(groundIntake.setAngleAndVoltage(GroundIntakeConstants.SCORE_ALGAE_ANGLE, -2))
        .onFalse(scoreAlgae());

    m_operatorController
        .rightStick()
        .whileTrue(
            Commands.startEnd(
                    () -> driveCommand.setTargetMode(DriveCommand.TargetMode.PROCESSOR),
                    () -> driveCommand.setTargetMode(DriveCommand.TargetMode.REEF))
                .withName("Face Processor"));

    m_operatorController
        .leftStick()
        .whileTrue(
            Commands.startEnd(
                    () -> driveCommand.setTargetMode(DriveCommand.TargetMode.NORMAL),
                    () -> driveCommand.setTargetMode(DriveCommand.TargetMode.REEF))
                .withName("Ground Intake normal"));

    m_operatorController
        .x()
        .whileTrue(
            Commands.startEnd(
                    () -> driveCommand.setTargetMode(DriveCommand.TargetMode.NORMAL),
                    () -> driveCommand.setTargetMode(DriveCommand.TargetMode.REEF))
                .withName("Algae Normal"));

    IS_L1
        .and(IS_REEF_MODE)
        .onTrue(
            Commands.runOnce(
                () -> {
                  driveCommand.setReefMode(DriveCommand.ReefPositions.BACK_REEF);
                }));

    IS_L2
        .or(IS_L3)
        .or(IS_L4)
        .and(IS_REEF_MODE)
        .onTrue(
            Commands.runOnce(
                () -> {
                  driveCommand.setReefMode(DriveCommand.ReefPositions.FRONT_REEF);
                }));

    m_driverController.start().onTrue(Commands.runOnce(drivetrain::seedFieldCentric));

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

    m_driverController
        .a()
        .whileTrue(alignToPose(() -> EagleUtil.getCachedReefPose(drivetrain.getState().Pose)));

    m_driverController
        .b()
        .whileTrue(alignToPose(() -> EagleUtil.closestReefSetPoint(drivetrain.getPose(), 1)));

    m_operatorController.start().onTrue(elevator.homingCommand());

    m_operatorController
        .x()
        .whileTrue(groundIntake.setAngleAndVoltage(GroundIntakeConstants.INTAKE_CORAL_ANGLE, -12))
        .onFalse(
            groundIntake.setAngleAndVoltage(GroundIntakeConstants.CORAL_STOW_ANGLE, -3)); // TODO

    // m_operatorController.y().onTrue(Commands.runOnce(() -> coralLevel = CoralLevel.L4));
    // m_operatorController.b().onTrue(Commands.runOnce(() -> coralLevel = CoralLevel.L3));
    // m_operatorController.a().onTrue(Commands.runOnce(() -> coralLevel = CoralLevel.L2));
    // m_operatorController.x().onTrue(Commands.runOnce(() -> coralLevel = CoralLevel.L1));

    m_operatorController.y().whileTrue(elevator.sysIdQuasistatic(Direction.kForward));
    m_operatorController.b().whileTrue(elevator.sysIdQuasistatic(Direction.kReverse));
    m_operatorController.a().whileTrue(elevator.sysIdDynamic(Direction.kForward));
    m_operatorController.x().whileTrue(elevator.sysIdDynamic(Direction.kReverse));

    m_operatorController.povRight().onTrue(arm.increaseAngle(3.0));
    m_operatorController.povLeft().onTrue(arm.decreaseAngle(3.0));
    m_operatorController.povUp().onTrue(elevator.increaseHeight(0.02));
    m_operatorController.povDown().onTrue(elevator.decreaseHeight(0.02));

    m_operatorController.leftBumper().onTrue(groundIntake.decreaseAngle(3));
    m_operatorController.rightBumper().onTrue(groundIntake.increaseAngle(3));

    m_operatorController.leftTrigger().onTrue(climb());
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

    if (leftCam != null) {
      leftCam.updatePoseEstim();
      // 3
      DogLog.log("Loop Time/Robot Container/Cam3", (HALUtil.getFPGATime() - startTime) / 1000);

      startTime = HALUtil.getFPGATime();
    }
    if (rightCam != null) {
      rightCam.updatePoseEstim();
    }
    // 4
    DogLog.log("Loop Time/Robot Container/Cam4", (HALUtil.getFPGATime() - startTime) / 1000);

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
    autoChooser.setDefaultOption("Five_Cycle_Processor", new FiveCycle(this, false));
    autoChooser.addOption("Five_Cycle_Non_Processor", new FiveCycle(this, true));
    autoChooser.addOption("Score_Preload_One_Cycle", new ScorePreloadOneCycle(this));
    autoChooser.addOption("Leave_Non_Processor", new LeaveNonProcessor(this));
    autoChooser.addOption("Leave_Processor", new LeaveProcessor(this));
    autoChooser.addOption("Push_One_Cycle", new PushOneCycle(this));
    autoChooser.addOption(
        "Wheel_Radius_Chracterizaton",
        WheelRadiusCharacterization.wheelRadiusCharacterization(drivetrain));

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
    return Commands.sequence(
            endEffector.intake(),
            elevator.setHeight(elevatorHeight).withTimeout(0.5),
            arm.setAngle(armAngle).withTimeout(1))
        .withName("Prepare Coral Intake");
  }

  public Command prepCoralIntake() {
    return prepCoralIntake(ElevatorConstants.INTAKE_METER, ArmConstants.ARM_INTAKE_ANGLE);
  }

  public Command stopIntake() {
    return Commands.parallel(
            arm.setAngle(ArmConstants.ARM_STOW_ANGLE),
            elevator.setHeight(ElevatorConstants.STOW_METER),
            endEffector.holdCoral())
        .withName("stop Intake");
  }

  public Command groundIntakeScoreL1() {
    return Commands.sequence(
            groundIntake
                .setAngleAndVoltage(GroundIntakeConstants.SCORE_CORAL_ANGLE, 6)
                .withTimeout(0.5),
            Commands.waitSeconds(0.3),
            groundIntake
                .setAngleAndVoltage(GroundIntakeConstants.CORAL_STOW_ANGLE, 0)
                .withTimeout(0.5))
        .withName("Ground Intake Score Coral L1");
  }

  public Command scoreAlgae() {
    return Commands.sequence(
            groundIntake
                .setAngleAndVoltage(GroundIntakeConstants.SCORE_ALGAE_ANGLE, 5)
                .withTimeout(0.5),
            Commands.waitSeconds(0.7),
            groundIntake
                .setAngleAndVoltage(GroundIntakeConstants.ALGAE_STOW_ANGLE, 0)
                .withTimeout(0.5))
        .withName("Score Algae");
  }

  /**
   * @param elevatorHeight how tall should the elavator be?
   * @param armAngle what angle should the arm be at
   * @return run the command
   */
  public Command prepScoreCoral(double elevatorHeight, double armAngle) {
    return Commands.parallel(
            endEffector.holdCoral(),
            elevator.setHeight(elevatorHeight).withTimeout(1.5),
            arm.setAngle(armAngle).withTimeout(1.5))
        .withName(
            "Prepare Score Coral; Elevator Height: " + elevatorHeight + " Arm Angle: " + armAngle);
  }

  public Command prepScoreCoral(DoubleSupplier elevatorHeight, DoubleSupplier armAngle) {
    return Commands.parallel(
            endEffector.holdCoral(),
            elevator.setHeightSupplier(elevatorHeight).withTimeout(.5),
            arm.setAngleSupplier(armAngle).withTimeout(.5))
        .withName(
            "Prepare Score Coral; Elevator Height: " + elevatorHeight + " Arm Angle: " + armAngle);
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
            endEffector.shoot(),
            Commands.waitSeconds(0.05),
            arm.setAngle(ArmConstants.ARM_STOW_ANGLE).withTimeout(0.05),
            elevator.setHeight(ElevatorConstants.STOW_METER).withTimeout(0.0),
            endEffector.stopMotor())
        .withTimeout(0.5);
  }

  /**
   * @return score the coral
   */
  public Command scoreCoral() {
    Command scoreCoral =
        Commands.sequence(
                endEffector.shoot(),
                Commands.waitSeconds(0.1),
                arm.setAngle(ArmConstants.ARM_STOW_ANGLE).withTimeout(0.1),
                elevator.setHeight(ElevatorConstants.STOW_METER).withTimeout(0.0),
                endEffector.stopMotor())
            .withTimeout(0.5);

    Command deAlgae =
        Commands.sequence(
                endEffector.shoot(),
                Commands.waitSeconds(0.1),
                endEffector.stopMotor(),
                alignToPose(() -> EagleUtil.getNearestAlgaePoint(drivetrain.getState().Pose))
                    .withTimeout(1),
                Commands.either(prepDealgaeHigh(), prepDealgaeLow(), ALGAE_HIGH)
                    .withTimeout(0.5)
                    .deadlineFor(
                        alignToPose(
                            () -> EagleUtil.getNearestAlgaePoint(drivetrain.getState().Pose))),
                dealgae())
            .withInterruptBehavior(InterruptionBehavior.kCancelIncoming);

    return Commands.sequence(
        Commands.either(deAlgae, scoreCoral, m_driverController.leftTrigger())
            .withName("Score Coral/deAlgae"));
  }

  // DeAlgae Commands
  public Command prepDealgaeLow() {
    return Commands.parallel(
            elevator.setHeight(ElevatorConstants.DEALGAE_LOW_POSITION),
            arm.setAngle(ArmConstants.PRE_DEALGAE_ANGLE),
            endEffector.setVoltage(0))
        .withName("prep Delalgae low");
  }

  public Command prepDealgaeHigh() {
    return Commands.parallel(
            elevator.setHeight(ElevatorConstants.DEALGAE_HIGH_POSITION),
            arm.setAngle(ArmConstants.PRE_DEALGAE_ANGLE),
            endEffector.setVoltage(0))
        .withName("prep Dealgae high");
  }

  public Command dealgae() {
    return Commands.sequence(
            arm.setAngle(ArmConstants.DEALGAE_ANGLE).withTimeout(0.2),
            drivetrain.driveBackward(1).withTimeout(0.6),
            Commands.parallel(
                elevator.setHeight(ElevatorConstants.STOW_METER).withTimeout(.1),
                arm.setAngle(ArmConstants.ARM_STOW_ANGLE).withTimeout(.1),
                endEffector.stopMotor()))
        .withName("Dealgae");
  }

  public Command climb() {
    Trigger unprepclimbTrigger = m_operatorController.leftTrigger().negate();
    Trigger climbTrigger = m_operatorController.rightTrigger();

    Command unPrepClimbCommand =
        Commands.sequence(
            Commands.parallel(
                elevator.setHeight(0).withTimeout(1),
                arm.setAngle(90).withTimeout(1),
                Commands.runOnce(() -> driveCommand.setTargetMode(DriveCommand.TargetMode.REEF)),
                climb.stow()));

    Command climbCommand =
        Commands.parallel(
                climb.climb(),
                elevator.setHeight(0),
                arm.setAngle(ArmConstants.CLIMB_ANGLE),
                Commands.runOnce(() -> driveCommand.setTargetMode(DriveCommand.TargetMode.NORMAL)))
            .withTimeout(0.5);

    return Commands.sequence(
            Commands.sequence(
                groundIntake
                    .setAngleAndVoltage(GroundIntakeConstants.CLIMB_ANGLE, 0)
                    .withTimeout(1),
                arm.setAngle(ArmConstants.PREP_CLIMB_ANGLE).withTimeout(1),
                elevator.setHeight(0).withTimeout(1),
                climb.latch().withTimeout(1),
                Commands.runOnce(() -> driveCommand.setTargetMode(DriveCommand.TargetMode.CAGE))),
            Commands.waitUntil(unprepclimbTrigger.or(climbTrigger)),
            Commands.either(unPrepClimbCommand, climbCommand, unprepclimbTrigger))
        .withInterruptBehavior(InterruptionBehavior.kCancelIncoming)
        .withName("Climb");
  }
}
