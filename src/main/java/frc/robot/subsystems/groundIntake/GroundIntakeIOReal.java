package frc.robot.subsystems.groundIntake;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.*;
import dev.doglog.DogLog;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DutyCycleEncoder;

public class GroundIntakeIOReal implements GroundIntakeIO {

  private final TalonFX spinMotor = new TalonFX(GroundIntakeConstants.SPIN_MOTOR_ID, "rio");
  private final TalonFX pivotMotor = new TalonFX(GroundIntakeConstants.PIVOT_MOTOR_ID, "rio");
  private final DutyCycleEncoder pivotEncoder =
      new DutyCycleEncoder(GroundIntakeConstants.PIVOT_ENCODER_ID);

  private final Alert spinMotorConnected =
      new Alert("ground intake spin motor not connected", AlertType.kError);

  private final Alert pivotMotorConnected =
      new Alert("ground intake pivot motor not connected", AlertType.kError);

  private final Alert pivotEncoderConnected =
      new Alert("ground intake pivot encoder not connected", AlertType.kError);

  private final StatusSignal<Voltage> spinMotorVoltage = spinMotor.getMotorVoltage();
  private final StatusSignal<Voltage> pivotMotorVoltage = pivotMotor.getMotorVoltage();
  private final StatusSignal<Temperature> spinMotorTemperature = spinMotor.getDeviceTemp();
  private final StatusSignal<Temperature> pivotMotorTemperature = pivotMotor.getDeviceTemp();
  private final StatusSignal<Current> spinMotorStatorCurrent = spinMotor.getStatorCurrent();
  private final StatusSignal<Current> pivotMotorStatorCurrent = pivotMotor.getStatorCurrent();
  private final StatusSignal<Angle> pivotMotorPosition = pivotMotor.getPosition();
  private final StatusSignal<Double> groundIntakePIDGoal = pivotMotor.getClosedLoopReference();

  private final MotionMagicVoltage m_request = new MotionMagicVoltage(0);

  public GroundIntakeIOReal() {
    // configuration for the pivot motor

    TalonFXConfiguration talonFXConfigs = new TalonFXConfiguration();
    MotorOutputConfigs motorOutput = talonFXConfigs.MotorOutput;
    MotionMagicConfigs motionMagicConfigs = talonFXConfigs.MotionMagic;
    Slot0Configs slot0Configs = talonFXConfigs.Slot0;
    SoftwareLimitSwitchConfigs softwareLimitSwitch = talonFXConfigs.SoftwareLimitSwitch;
    CurrentLimitsConfigs currentConfig = talonFXConfigs.CurrentLimits;
    FeedbackConfigs feedbackConfigs = talonFXConfigs.Feedback;
    m_request.EnableFOC = true; // add FOC

    slot0Configs.kS = 0.18205; // Add 0.25 V output to overcome static friction
    slot0Configs.kG = 0.09885; // Add 0 V to overcome gravity
    slot0Configs.kV = 7.2427; // A velocity target of 1 rps results in 0.12 V output
    slot0Configs.kA = 0.086264; // An acceleration of 1 rps/s requires 0.01 V output
    slot0Configs.kP = 57.759; // A position error of 2.5 rotations results in 12 V output
    slot0Configs.kI = 0; // no output for integrated error
    slot0Configs.kD = 8.4867; // A velocity error of 1 rps results in 0.1 V output
    slot0Configs.withGravityType(GravityTypeValue.Arm_Cosine);

    feedbackConfigs.FeedbackRotorOffset = 0;
    feedbackConfigs.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
    feedbackConfigs.RotorToSensorRatio = 1;
    feedbackConfigs.SensorToMechanismRatio = GroundIntakeConstants.PIVOT_GEAR_RATIO;

    motionMagicConfigs.MotionMagicCruiseVelocity = GroundIntakeConstants.MAX_VELOCITY;
    motionMagicConfigs.MotionMagicAcceleration = GroundIntakeConstants.MAX_ACCELERATION;
    motionMagicConfigs.MotionMagicJerk = 0; // Target jerk of 1600 rps/s/s (0.1 seconds)

    motorOutput.NeutralMode = NeutralModeValue.Brake;
    motorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    softwareLimitSwitch.ForwardSoftLimitEnable = true;
    softwareLimitSwitch.ForwardSoftLimitThreshold =
        Units.degreesToRotations(GroundIntakeConstants.GROUND_INTAKE_UPPER_BOUND);
    softwareLimitSwitch.ReverseSoftLimitEnable = true;
    softwareLimitSwitch.ReverseSoftLimitThreshold =
        Units.degreesToRotations(GroundIntakeConstants.GROUND_INTAKE_LOWER_BOUND);

    currentConfig.withStatorCurrentLimitEnable(true);
    currentConfig.withStatorCurrentLimit(40);

    StatusCode status = StatusCode.StatusCodeNotInitialized;
    for (int i = 0; i < 5; i++) {
      status = pivotMotor.getConfigurator().apply(talonFXConfigs);
      if (status.isOK()) break;
    }
    if (!status.isOK()) {
      System.out.println("Could not configure device. Error: " + status.toString());
    }

    // configuration for the spin motor

    talonFXConfigs = new TalonFXConfiguration();
    motorOutput = talonFXConfigs.MotorOutput;
    currentConfig = talonFXConfigs.CurrentLimits;

    motorOutput.NeutralMode = NeutralModeValue.Coast;
    motorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    currentConfig.withStatorCurrentLimitEnable(true);
    currentConfig.withStatorCurrentLimit(40);

    status = StatusCode.StatusCodeNotInitialized;
    for (int i = 0; i < 5; i++) {
      status = spinMotor.getConfigurator().apply(talonFXConfigs);
      if (status.isOK()) break;
    }
    if (!status.isOK()) {
      System.out.println("Could not configure device. Error: " + status.toString());
    }

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0,
        spinMotorStatorCurrent,
        pivotMotorPosition,
        pivotMotorStatorCurrent,
        groundIntakePIDGoal);
  }

  @Override
  public void setSpinMotorVoltage(double voltage) {
    spinMotor.setVoltage(voltage);
  }

  @Override
  public void setPivotMotorVoltage(double voltage) {
    pivotMotor.setVoltage(voltage);
  }

  @Override
  public double getPivotAngle() {
    return Units.rotationsToDegrees(pivotEncoder.get());
  }

  @Override
  public void setAngle(double angle) {
    pivotMotor.setControl(m_request.withPosition(Units.degreesToRotations(angle)));
  }

  @Override
  public void update() {
    BaseStatusSignal.refreshAll(
        spinMotorVoltage,
        pivotMotorVoltage,
        pivotMotorTemperature,
        spinMotorTemperature,
        pivotMotorStatorCurrent,
        spinMotorStatorCurrent,
        pivotMotorPosition);

    DogLog.log("groundIntake/Spin/voltage", spinMotorVoltage.getValueAsDouble());
    DogLog.log("groundIntake/Pivot/voltage", pivotMotorVoltage.getValueAsDouble());
    DogLog.log("groundIntake/Spin/temperature", spinMotorTemperature.getValueAsDouble());
    DogLog.log("groundIntake/Pivot/temperature", pivotMotorTemperature.getValueAsDouble());
    DogLog.log("groundIntake/Spin/statorCurrent", spinMotorStatorCurrent.getValueAsDouble());
    DogLog.log("groundIntake/Pivot/statorCurrent", pivotMotorStatorCurrent.getValueAsDouble());
    DogLog.log("groundIntake/Pivot/PIDGoal", groundIntakePIDGoal.getValueAsDouble());
    DogLog.log("groundIntake/Pivot/position", pivotMotorPosition.getValueAsDouble());
    DogLog.log("groundIntake/Spin/Motor Connected", spinMotor.isConnected());
    DogLog.log("groundIntake/Pivot/Motor Connected", pivotMotor.isConnected());
    DogLog.log("groundIntake/Pivot/Encoder Connected", pivotEncoder.isConnected());

    spinMotorConnected.set(!spinMotor.isConnected());
    pivotMotorConnected.set(!pivotMotor.isConnected());
    pivotEncoderConnected.set(!pivotEncoder.isConnected());
  }

  @Override
  public void resetPivotEncoder() {
    double encoderAngle = pivotEncoder.get();
    pivotMotor.setPosition(encoderAngle);
  }
}
