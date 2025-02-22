package frc.robot.subsystems;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.generated.TunerConstants_Comp;
import frc.robot.generated.TunerSwerveDrivetrain;
import frc.robot.subsystems.aprilTagCam.AprilTagHelp;

import static edu.wpi.first.units.Units.MetersPerSecond;

import java.util.function.Supplier;

/**
 * Class that extends the Phoenix 6 SwerveDrivetrain class and implements Subsystem so it can easily
 * be used in command-based projects.
 */
public class CommandSwerveDrivetrain extends TunerSwerveDrivetrain implements Subsystem {
  public Trigger IS_ALIGNING_TO_POSE =
      new Trigger(
          () -> {
            if (this.getCurrentCommand() != null) {
              return this.getCurrentCommand().getName().equals("AlignToPose");
            } else {
              return false;
            }
          });

  SwerveModule<TalonFX, TalonFX, CANcoder> mod_1 = getModule(0);
  SwerveModule<TalonFX, TalonFX, CANcoder> mod_2 = getModule(1);
  SwerveModule<TalonFX, TalonFX, CANcoder> mod_3 = getModule(2);
  SwerveModule<TalonFX, TalonFX, CANcoder> mod_4 = getModule(3);
  TalonFX m_1 = mod_1.getDriveMotor();
  TalonFX m_2 = mod_2.getDriveMotor();
  TalonFX m_3 = mod_3.getDriveMotor();
  TalonFX m_4 = mod_4.getDriveMotor();
  TalonFXConfiguration m1_config = new TalonFXConfiguration();
  TalonFXConfiguration m2_config = new TalonFXConfiguration();
  TalonFXConfiguration m3_config = new TalonFXConfiguration();
  TalonFXConfiguration m4_config = new TalonFXConfiguration();
  CurrentLimitsConfigs m1_current_config = new CurrentLimitsConfigs();
  CurrentLimitsConfigs m2_current_config = new CurrentLimitsConfigs();
  CurrentLimitsConfigs m3_current_config = new CurrentLimitsConfigs();
  CurrentLimitsConfigs m4_current_config = new CurrentLimitsConfigs();

  public PIDController PID_X = new PIDController(1.7, 0, 0);
  public PIDController PID_Y = new PIDController(1.7, 0, 0);
  public PIDController PID_Rotation = new PIDController(0.1, 0, 0);
  public Trigger IS_AT_TARGET_POSE =
      new Trigger(() -> PID_X.atSetpoint() && PID_Y.atSetpoint() && PID_Rotation.atSetpoint());

  private static final double kSimLoopPeriod = 0.005; // 5 ms
  private Notifier m_simNotifier = null;
  private double m_lastSimTime;
  public static final LinearVelocity kSpeedAt12Volts = MetersPerSecond.of(4.73);

  public final double DRIVE_BASE_RADIUS =
      Math.max(
          Math.max(
              Math.hypot(
                  TunerConstants_Comp.FrontLeft.LocationX, TunerConstants_Comp.FrontLeft.LocationY),
              Math.hypot(
                  TunerConstants_Comp.FrontRight.LocationX,
                  TunerConstants_Comp.FrontRight.LocationY)),
          Math.max(
              Math.hypot(
                  TunerConstants_Comp.BackLeft.LocationX, TunerConstants_Comp.BackLeft.LocationY),
              Math.hypot(
                  TunerConstants_Comp.BackRight.LocationX,
                  TunerConstants_Comp.BackRight.LocationY)));

  /* Blue alliance sees forward as 0 degrees (toward red alliance wall) */
  private static final Rotation2d kBlueAlliancePerspectiveRotation = Rotation2d.kZero;
  /* Red alliance sees forward as 180 degrees (toward blue alliance wall) */
  private static final Rotation2d kRedAlliancePerspectiveRotation = Rotation2d.k180deg;
  /* Keep track if we've ever applied the operator perspective before or not */
  private boolean m_hasAppliedOperatorPerspective = false;

  /** Swerve request to apply during robot-centric path following */
  private final SwerveRequest.ApplyRobotSpeeds m_pathApplyRobotSpeeds =
      new SwerveRequest.ApplyRobotSpeeds();

  /**
   * Constructs a CTRE SwerveDrivetrain using the specified constants.
   *
   * <p>This constructs the underlying hardware devices, so users should not construct the devices
   * themselves. If they need the devices, they can access them through getters in the classes.
   *
   * @param drivetrainConstants Drivetrain-wide constants for the swerve drive
   * @param modules Constants for each specific module
   */
  public CommandSwerveDrivetrain(
      SwerveDrivetrainConstants drivetrainConstants, SwerveModuleConstants<?, ?, ?>... modules) {
    super(drivetrainConstants, modules);
    if (Utils.isSimulation()) {
      startSimThread();
    }
    PID_Rotation.setTolerance(0.5);
    PID_Rotation.enableContinuousInput(-180, 180);
    PID_Y.setTolerance(0.02);
    PID_X.setTolerance(0.02);
    configureAutoBuilder();
  }

  /**
   * Constructs a CTRE SwerveDrivetrain using the specified constants.
   *
   * <p>This constructs the underlying hardware devices, so users should not construct the devices
   * themselves. If they need the devices, they can access them through getters in the classes.
   *
   * @param drivetrainConstants Drivetrain-wide constants for the swerve drive
   * @param odometryUpdateFrequency The frequency to run the odometry loop. If unspecified or set to
   *     0 Hz, this is 250 Hz on CAN FD, and 100 Hz on CAN 2.0.
   * @param modules Constants for each specific module
   */
  public CommandSwerveDrivetrain(
      SwerveDrivetrainConstants drivetrainConstants,
      double odometryUpdateFrequency,
      SwerveModuleConstants<?, ?, ?>... modules) {
    super(drivetrainConstants, odometryUpdateFrequency, modules);
    if (Utils.isSimulation()) {
      startSimThread();
    }
    configureAutoBuilder();
  }

  public void goToPoseWithPID(Pose2d targetPose) {
    PID_X.setSetpoint(targetPose.getX());
    PID_Y.setSetpoint(targetPose.getY());
    PID_Rotation.setSetpoint(targetPose.getRotation().getDegrees());
  }

  public Command setDriveMotorCurrentLimit() {

    return Commands.runOnce(
        () -> {
          m1_current_config.withStatorCurrentLimitEnable(true);
          m1_current_config.withStatorCurrentLimit(35);
          m_1.getConfigurator().apply(m1_current_config);
          m_2.getConfigurator().apply(m1_current_config);
          m_3.getConfigurator().apply(m1_current_config);
          m_4.getConfigurator().apply(m1_current_config);
        });
  }

  /**
   * Constructs a CTRE SwerveDrivetrain using the specified constants.
   *
   * <p>This constructs the underlying hardware devices, so users should not construct the devices
   * themselves. If they need the devices, they can access them through getters in the classes.
   *
   * @param drivetrainConstants Drivetrain-wide constants for the swerve drive
   * @param odometryUpdateFrequency The frequency to run the odometry loop. If unspecified or set to
   *     0 Hz, this is 250 Hz on CAN FD, and 100 Hz on CAN 2.0.
   * @param odometryStandardDeviation The standard deviation for odometry calculation in the form
   *     [x, y, theta]ᵀ, with units in meters and radians
   * @param visionStandardDeviation The standard deviation for vision calculation in the form [x, y,
   *     theta]ᵀ, with units in meters and radians
   * @param modules Constants for each specific module
   */
  public CommandSwerveDrivetrain(
      SwerveDrivetrainConstants drivetrainConstants,
      double odometryUpdateFrequency,
      Matrix<N3, N1> odometryStandardDeviation,
      Matrix<N3, N1> visionStandardDeviation,
      SwerveModuleConstants<?, ?, ?>... modules) {
    super(
        drivetrainConstants,
        odometryUpdateFrequency,
        odometryStandardDeviation,
        visionStandardDeviation,
        modules);
    if (Utils.isSimulation()) {
      startSimThread();
    }
    configureAutoBuilder();
  }

  private void configureAutoBuilder() {
    try {
      var config = RobotConfig.fromGUISettings();
      AutoBuilder.configure(
          () -> getState().Pose, // Supplier of current robot pose
          this::resetPose, // Consumer for seeding pose against auto
          () -> getState().Speeds, // Supplier of current robot speeds
          // Consumer of ChassisSpeeds and feedforwards to drive the robot
          (speeds, feedforwards) ->
              setControl(
                  m_pathApplyRobotSpeeds
                      .withSpeeds(speeds)
                      .withWheelForceFeedforwardsX(feedforwards.robotRelativeForcesXNewtons())
                      .withWheelForceFeedforwardsY(feedforwards.robotRelativeForcesYNewtons())),
          new PPHolonomicDriveController(
              // PID constants for translation
              new PIDConstants(10, 0, 0),
              // PID constants for rotation
              new PIDConstants(7, 0, 0)),
          config,
          // Assume the path needs to be flipped for Red vs Blue, this is normally the case
          () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red,
          this // Subsystem for requirements
          );
    } catch (Exception ex) {
      DriverStation.reportError(
          "Failed to load PathPlanner config and configure AutoBuilder", ex.getStackTrace());
    }
  }

  /**
   * Returns a command that applies the specified control request to this swerve drivetrain.
   *
   * @param request Function returning the request to apply
   * @return Command to run
   */
  public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
    return run(() -> this.setControl(requestSupplier.get()));
  }

  @Override
  public void periodic() {
    /*
     * Periodically try to apply the operator perspective.
     * If we haven't applied the operator perspective before, then we should apply it regardless of DS state.
     * This allows us to correct the perspective in case the robot code restarts mid-match.
     * Otherwise, only check and apply the operator perspective if the DS is disabled.
     * This ensures driving behavior doesn't change until an explicit disable event occurs during testing.
     */
    if (!m_hasAppliedOperatorPerspective || DriverStation.isDisabled()) {
      DriverStation.getAlliance()
          .ifPresent(
              allianceColor -> {
                setOperatorPerspectiveForward(
                    allianceColor == Alliance.Red
                        ? kRedAlliancePerspectiveRotation
                        : kBlueAlliancePerspectiveRotation);
                m_hasAppliedOperatorPerspective = true;
              });
    }
  }

  private void startSimThread() {
    m_lastSimTime = Utils.getCurrentTimeSeconds();

    /* Run simulation at a faster rate so PID gains behave more reasonably */
    m_simNotifier =
        new Notifier(
            () -> {
              final double currentTime = Utils.getCurrentTimeSeconds();
              double deltaTime = currentTime - m_lastSimTime;
              m_lastSimTime = currentTime;

              /* use the measured time delta, get battery voltage from WPILib */
              updateSimState(deltaTime, RobotController.getBatteryVoltage());
            });
    m_simNotifier.startPeriodic(kSimLoopPeriod);
  }

  public void addVisionMeasurent(AprilTagHelp helper) {

    Pose2d pos = helper.pos;
    Matrix<N3, N1> sd = helper.sd;
    double timestamp = helper.timestamp;

    super.addVisionMeasurement(pos, timestamp, sd);
  }

  public Pose2d getPose() {
    return getState().Pose;
  }

  public Rotation2d getRotation() {
    return getPose().getRotation();
  }

  /** Returns the position of each module in radians. */
  public double[] getWheelRadiusCharacterizationPositions() {
    double[] values = new double[4];
    for (int i = 0; i < 4; i++) {
      values[i] =
          Units.rotationsToRadians(
              getModule(i).getDriveMotor().getPosition(true).getValueAsDouble()
                  / 6.746031746031747);
    }
    return values;
  }

  public void runVelocity(ChassisSpeeds speeds) {
    setControl(m_pathApplyRobotSpeeds.withSpeeds(speeds));
  }
}
