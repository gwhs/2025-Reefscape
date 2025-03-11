package frc.robot.subsystems.arm;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import dev.doglog.DogLog;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;

public class ArmIOReal implements ArmIO {
  private TalonFX armMotor = new TalonFX(ArmConstants.ARM_MOTOR_ID, "rio");
  private CANcoder armEncoder = new CANcoder(ArmConstants.ARM_ENCODER_ID, "rio");
  private final MotionMagicVoltage m_request = new MotionMagicVoltage(0);
  private final VoltageOut m_voltReq = new VoltageOut(0.0);

  private final StatusSignal<Double> armPIDGoal = armMotor.getClosedLoopReference();
  private final StatusSignal<Voltage> armMotorVoltage = armMotor.getMotorVoltage();
  private final StatusSignal<Voltage> armSupplyVoltage = armMotor.getSupplyVoltage();
  private final StatusSignal<Temperature> armDeviceTemp = armMotor.getDeviceTemp();
  private final StatusSignal<Current> armStatorCurrent = armMotor.getStatorCurrent();
  private final StatusSignal<Angle> armEncoderPosition = armEncoder.getPosition();
  private final StatusSignal<Angle> armPosition = armMotor.getPosition();

  private final Alert armMotorConnectedAlert =
      new Alert("Arm motor not connected", AlertType.kError);

  private final Alert armEncoderConnectedAlert =
      new Alert("Arm CANcoder not connected", AlertType.kError);

  public ArmIOReal() {
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
    feedbackConfigs.FeedbackRemoteSensorID = ArmConstants.ARM_ENCODER_ID;
    feedbackConfigs.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
    feedbackConfigs.RotorToSensorRatio = ArmConstants.ARM_GEAR_RATIO;
    feedbackConfigs.SensorToMechanismRatio = 1;

    motionMagicConfigs.MotionMagicCruiseVelocity = ArmConstants.MAX_VELOCITY;
    motionMagicConfigs.MotionMagicAcceleration = ArmConstants.MAX_ACCELERATION;
    motionMagicConfigs.MotionMagicJerk = 1600; // Target jerk of 1600 rps/s/s (0.1 seconds)

    motorOutput.NeutralMode = NeutralModeValue.Brake;
    motorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    softwareLimitSwitch.ForwardSoftLimitEnable = true;
    softwareLimitSwitch.ForwardSoftLimitThreshold =
        Units.degreesToRotations(ArmConstants.ARM_UPPER_BOUND);
    softwareLimitSwitch.ReverseSoftLimitEnable = true;
    softwareLimitSwitch.ReverseSoftLimitThreshold =
        Units.degreesToRotations(ArmConstants.ARM_LOWER_BOUND);

    currentConfig.withStatorCurrentLimitEnable(true);
    currentConfig.withStatorCurrentLimit(20);

    StatusCode status = StatusCode.StatusCodeNotInitialized;
    for (int i = 0; i < 5; i++) {
      status = armMotor.getConfigurator().apply(talonFXConfigs);
      if (status.isOK()) break;
    }
    if (!status.isOK()) {
      System.out.println("Could not configure device. Error: " + status.toString());
    }

    BaseStatusSignal.setUpdateFrequencyForAll(50.0, armPIDGoal, armStatorCurrent);

    CANcoderConfiguration cc_cfg = new CANcoderConfiguration();
    cc_cfg.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 1;
    cc_cfg.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
    cc_cfg.MagnetSensor.withMagnetOffset(Units.degreesToRotations(311.46875));

    for (int i = 0; i < 5; i++) {
      status = armEncoder.getConfigurator().apply(cc_cfg);
      if (status.isOK()) break;
    }
    if (!status.isOK()) {
      System.out.println("Could not configure device. Error: " + status.toString());
    }
  }

  // set arm angle in degrees
  @Override
  public void setAngle(double angle) {
    armMotor.setControl(m_request.withPosition(Units.degreesToRotations(angle)));
  }

  @Override
  public double getPosition() {
    return Units.rotationsToDegrees(armEncoderPosition.getValueAsDouble());
  }

  /**
   * @param volts the voltage to set to
   */
  public void setVoltage(double volts) {
    armMotor.setControl(m_voltReq.withOutput(volts));
  }

  @Override
  public void update() {
    boolean armConnected =
        (BaseStatusSignal.refreshAll(
                armPIDGoal,
                armMotorVoltage,
                armSupplyVoltage,
                armDeviceTemp,
                armStatorCurrent,
                armPosition,
                armEncoderPosition)
            .isOK());
    DogLog.log("Arm/Motor/pid goal", Units.rotationsToDegrees(armPIDGoal.getValueAsDouble()));
    DogLog.log("Arm/Motor/motor voltage", armMotorVoltage.getValueAsDouble());
    DogLog.log("Arm/Motor/supply voltage", armSupplyVoltage.getValueAsDouble());
    DogLog.log("Arm/Motor/device temp", armDeviceTemp.getValueAsDouble());
    DogLog.log("Arm/Motor/stator current", armStatorCurrent.getValueAsDouble());
    DogLog.log("Arm/Motor/Connected", armConnected);
    DogLog.log("Arm/Encoder/encoder position", getPosition());

    armMotorConnectedAlert.set(!armConnected);
    armEncoderConnectedAlert.set(!armEncoder.isConnected());
  }
}
