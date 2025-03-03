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
import edu.wpi.first.wpilibj.DutyCycleEncoder;

public class groundIntakeIOReal implements groundIntakeIO {

  private final TalonFX spinMotor = new TalonFX(groundIntakeConstants.SPIN_MOTOR_ID, "rio");
  private final TalonFX pivotMotor = new TalonFX(groundIntakeConstants.PIVOT_MOTOR_ID, "rio");
  private final DutyCycleEncoder pivotEncoder =
      new DutyCycleEncoder(groundIntakeConstants.PIVOT_ENCODER_ID);
  private final StatusSignal<Voltage> spinMotorVoltage = spinMotor.getMotorVoltage();
  private final StatusSignal<Voltage> pivotMotorVoltage = pivotMotor.getMotorVoltage();
  private final StatusSignal<Temperature> spinMotorTemperature = spinMotor.getDeviceTemp();
  private final StatusSignal<Temperature> pivotMotorTemperature = pivotMotor.getDeviceTemp();
  private final StatusSignal<Current> spinMotorStatorCurrent = spinMotor.getStatorCurrent();
  private final StatusSignal<Current> pivotMotorStatorCurrent = pivotMotor.getStatorCurrent();
  private final StatusSignal<Angle> pivotMotorPosition = pivotMotor.getPosition();
  private final StatusSignal<Double> groundIntakePIDGoal = pivotMotor.getClosedLoopReference();
  private final MotionMagicVoltage m_request = new MotionMagicVoltage(0);

  public groundIntakeIOReal() {
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
    feedbackConfigs.RotorToSensorRatio = groundIntakeConstants.PIVOT_GEAR_RATIO;
    feedbackConfigs.SensorToMechanismRatio = 1;

    motionMagicConfigs.MotionMagicCruiseVelocity = groundIntakeConstants.MAX_VELOCITY;
    motionMagicConfigs.MotionMagicAcceleration = groundIntakeConstants.MAX_ACCELERATION;
    motionMagicConfigs.MotionMagicJerk = 1600; // Target jerk of 1600 rps/s/s (0.1 seconds)

    motorOutput.NeutralMode = NeutralModeValue.Coast;
    motorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    softwareLimitSwitch.ForwardSoftLimitEnable = true;
    softwareLimitSwitch.ForwardSoftLimitThreshold =
        Units.degreesToRotations(groundIntakeConstants.GROUND_INTAKE_LOWER_BOUND);
    softwareLimitSwitch.ReverseSoftLimitEnable = true;
    softwareLimitSwitch.ReverseSoftLimitThreshold =
        Units.degreesToRotations(groundIntakeConstants.GROUND_INTAKE_LOWER_BOUND);

    currentConfig.withStatorCurrentLimitEnable(true);
    currentConfig.withStatorCurrentLimit(20);

    StatusCode status = StatusCode.StatusCodeNotInitialized;
    for (int i = 0; i < 5; i++) {
      status = pivotMotor.getConfigurator().apply(talonFXConfigs);
      if (status.isOK()) break;
    }
    if (!status.isOK()) {
      System.out.println("Could not configure device. Error: " + status.toString());
    }

    BaseStatusSignal.setUpdateFrequencyForAll(50.0, spinMotorStatorCurrent, pivotMotorPosition);
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
  public void update() {
    boolean groundIntakeisConnected =
        (BaseStatusSignal.refreshAll(
                spinMotorVoltage,
                pivotMotorVoltage,
                pivotMotorTemperature,
                spinMotorTemperature,
                pivotMotorStatorCurrent,
                spinMotorStatorCurrent,
                pivotMotorPosition))
            .isOK();
    DogLog.log("groundIntake/Spin/voltage", spinMotorVoltage.getValueAsDouble());
    DogLog.log("groundIntake/Pivot/voltage", pivotMotorVoltage.getValueAsDouble());
    DogLog.log("groundIntake/Spin/temperature", spinMotorTemperature.getValueAsDouble());
    DogLog.log("groundIntake/Pivot/temperature", pivotMotorTemperature.getValueAsDouble());
    DogLog.log("groundIntake/Spin/statorCurrent", spinMotorStatorCurrent.getValueAsDouble());
    DogLog.log("groundIntake/Pivot/statorCurrent", pivotMotorStatorCurrent.getValueAsDouble());
    DogLog.log("groundIntake/Pivot/PIDGoal", groundIntakePIDGoal.getValueAsDouble());
    DogLog.log("groundIntake/Pivot/position", pivotMotorPosition.getValueAsDouble());
    DogLog.log("groundIntake/Connected", groundIntakeisConnected);
  }
}
