package frc.robot.subsystems.elevator;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.HardwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.ForwardLimitSourceValue;
import com.ctre.phoenix6.signals.ForwardLimitTypeValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.ForwardLimitValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.ReverseLimitSourceValue;
import com.ctre.phoenix6.signals.ReverseLimitTypeValue;
import com.ctre.phoenix6.signals.ReverseLimitValue;
import dev.doglog.DogLog;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;

public class ElevatorIOReal implements ElevatorIO {
  private TalonFX m_leftElevatorMotor =
      new TalonFX(ElevatorConstants.LEFT_ELEVATOR_MOTOR_ID, "rio");
  public TalonFX m_rightElevatorMotor =
      new TalonFX(ElevatorConstants.RIGHT_ELEVATOR_MOTOR_ID, "rio");

  private final MotionMagicVoltage m_requestLeft = new MotionMagicVoltage(0);
  private final Follower m_requestRight =
      new Follower(ElevatorConstants.LEFT_ELEVATOR_MOTOR_ID, true);

  private final VoltageOut m_requestLeftVoltage = new VoltageOut(0);

  private final StatusSignal<Double> leftElevatorMotorPIDGoal =
      m_leftElevatorMotor.getClosedLoopReference();
  private final StatusSignal<Voltage> leftElevatorMotorVoltage =
      m_leftElevatorMotor.getMotorVoltage();
  private final StatusSignal<Voltage> leftElevatorMotorSupplyVoltage =
      m_leftElevatorMotor.getSupplyVoltage();
  private final StatusSignal<Temperature> leftElevatorMotorDeviceTemp =
      m_leftElevatorMotor.getDeviceTemp();
  private final StatusSignal<Current> leftElevatorMotorStatorCurrent =
      m_leftElevatorMotor.getStatorCurrent();
  private final StatusSignal<Angle> leftElevatorMotorPosition = m_leftElevatorMotor.getPosition();

  private final StatusSignal<ForwardLimitValue> forwardLimit =
      m_leftElevatorMotor.getForwardLimit();
  private final StatusSignal<ReverseLimitValue> reverseLimit =
      m_leftElevatorMotor.getReverseLimit();

  private final StatusSignal<Double> rightElevatorMotorPIDGoal =
      m_rightElevatorMotor.getClosedLoopReference();
  private final StatusSignal<Voltage> rightElevatorMotorVoltage =
      m_rightElevatorMotor.getMotorVoltage();
  private final StatusSignal<Voltage> rightElevatorMotorSupplyVoltage =
      m_rightElevatorMotor.getSupplyVoltage();
  private final StatusSignal<Temperature> rightElevatorMotorDeviceTemp =
      m_rightElevatorMotor.getDeviceTemp();
  private final StatusSignal<Current> rightElevatorMotorStatorCurrent =
      m_rightElevatorMotor.getStatorCurrent();
  private final StatusSignal<Angle> rightElevatorMotorPosition = m_rightElevatorMotor.getPosition();

  public ElevatorIOReal() {
    TalonFXConfiguration talonFXConfigs = new TalonFXConfiguration();
    MotorOutputConfigs motorOutput = talonFXConfigs.MotorOutput;
    CurrentLimitsConfigs currentConfig = talonFXConfigs.CurrentLimits;
    Slot0Configs slot0Configs = talonFXConfigs.Slot0;
    MotionMagicConfigs motionMagicConfigs = talonFXConfigs.MotionMagic;
    HardwareLimitSwitchConfigs hardwareLimitSwitchConfigs = talonFXConfigs.HardwareLimitSwitch;
    SoftwareLimitSwitchConfigs softwareLimitSwitchConfigs = talonFXConfigs.SoftwareLimitSwitch;

    slot0Configs.kS = 0.25; // Add 0.25 V output to overcome static friction
    slot0Configs.kG = 0; // Add 0 voltage to overcome gravity
    slot0Configs.kV = 0.12; // A velocity target of 1 rps results in 0.12 V output
    slot0Configs.kA = 0.01; // An acceleration of 1 rps/s requires 0.01 V output
    slot0Configs.kP = 4.8; // A position error of 2.5 rotations results in 12 V output
    slot0Configs.kI = 0; // no output for integrated error
    slot0Configs.kD = 0.1; // A velocity error of 1 rps results in 0.1 V output
    slot0Configs.withGravityType(GravityTypeValue.Elevator_Static);
    motionMagicConfigs.MotionMagicCruiseVelocity = 80; // Target cruise velocity of 80 rps
    motionMagicConfigs.MotionMagicAcceleration =
        160; // Target acceleration of 160 rps/s (0.5 seconds)
    motionMagicConfigs.MotionMagicJerk = 1600; // Target jerk of 1600 rps/s/s (0.1 seconds)

    currentConfig.withStatorCurrentLimitEnable(true);
    currentConfig.withStatorCurrentLimit(60);
    motorOutput.NeutralMode = NeutralModeValue.Coast;
    motorOutput.Inverted = InvertedValue.Clockwise_Positive;

    softwareLimitSwitchConfigs.ForwardSoftLimitEnable = true;
    softwareLimitSwitchConfigs.ReverseSoftLimitEnable = true;
    softwareLimitSwitchConfigs.ForwardSoftLimitThreshold =
        ElevatorSubsystem.metersToRotations(ElevatorConstants.TOP_METER);
    softwareLimitSwitchConfigs.ReverseSoftLimitThreshold = ElevatorSubsystem.metersToRotations(0);

    TalonFXConfigurator rightElevatorConfigurator = m_rightElevatorMotor.getConfigurator();
    rightElevatorConfigurator.apply(talonFXConfigs);

    // Additional config for left motor

    hardwareLimitSwitchConfigs.ForwardLimitSource = ForwardLimitSourceValue.LimitSwitchPin;
    hardwareLimitSwitchConfigs.ReverseLimitSource = ReverseLimitSourceValue.LimitSwitchPin;
    hardwareLimitSwitchConfigs.ForwardLimitEnable = false;
    hardwareLimitSwitchConfigs.ReverseLimitEnable = true;
    hardwareLimitSwitchConfigs.ForwardLimitType = ForwardLimitTypeValue.NormallyOpen;
    hardwareLimitSwitchConfigs.ReverseLimitType = ReverseLimitTypeValue.NormallyOpen;
    hardwareLimitSwitchConfigs.ReverseLimitAutosetPositionEnable = true;
    hardwareLimitSwitchConfigs.ReverseLimitAutosetPositionValue = 0;

    motorOutput.Inverted = InvertedValue.Clockwise_Positive;

    TalonFXConfigurator leftElevatorConfigurator = m_leftElevatorMotor.getConfigurator();
    leftElevatorConfigurator.apply(talonFXConfigs);

    // Set right motor to follow left motor
    m_rightElevatorMotor.setControl(m_requestRight);
  }

  public void setRotation(double rotation) {
    m_leftElevatorMotor.setControl(m_requestLeft.withPosition(rotation));
    m_rightElevatorMotor.setControl(m_requestRight);
  }

  public double getRotation() {
    return leftElevatorMotorPosition.getValueAsDouble();
  }

  public boolean getForwardLimit() {
    return forwardLimit.getValue().value == 0;
  }

  public boolean getReverseLimit() {
    return reverseLimit.getValue().value == 0;
  }

  public void setVoltage(double voltage) {
    m_leftElevatorMotor.setControl(m_requestLeftVoltage.withOutput(voltage));
    m_rightElevatorMotor.setControl(m_requestRight);
  }

  @Override
  public void update() {
    boolean leftElevatorConnected =
        (BaseStatusSignal.refreshAll(
                leftElevatorMotorPIDGoal,
                leftElevatorMotorVoltage,
                leftElevatorMotorSupplyVoltage,
                leftElevatorMotorDeviceTemp,
                leftElevatorMotorStatorCurrent,
                leftElevatorMotorPosition,
                forwardLimit,
                reverseLimit)
            .isOK());

    boolean rightElevatorConnected =
        (BaseStatusSignal.refreshAll(
                rightElevatorMotorPIDGoal,
                rightElevatorMotorVoltage,
                rightElevatorMotorSupplyVoltage,
                rightElevatorMotorDeviceTemp,
                rightElevatorMotorStatorCurrent,
                rightElevatorMotorPosition)
            .isOK());

    DogLog.log("Elevator/Left Motor/pid goal", leftElevatorMotorPIDGoal.getValueAsDouble());
    DogLog.log("Elevator/Left Motor/motor voltage", leftElevatorMotorVoltage.getValueAsDouble());
    DogLog.log(
        "Elevator/Left Motor/supply voltage", leftElevatorMotorSupplyVoltage.getValueAsDouble());
    DogLog.log("Elevator/Left Motor/device temp", leftElevatorMotorDeviceTemp.getValueAsDouble());
    DogLog.log(
        "Elevator/Left Motor/stator current", leftElevatorMotorStatorCurrent.getValueAsDouble());
    DogLog.log("Elevator/Left Motor/position", leftElevatorMotorPosition.getValueAsDouble());

    DogLog.log("Elevator/Left Motor/Connected", leftElevatorConnected);
    DogLog.log("Elevator/Right Motor/Connected", rightElevatorConnected);

    DogLog.log("Elevator/Right Motor/pid goal", rightElevatorMotorPIDGoal.getValueAsDouble());
    DogLog.log("Elevator/Right Motor/motor voltage", rightElevatorMotorVoltage.getValueAsDouble());
    DogLog.log(
        "Elevator/Right Motor/supply voltage", rightElevatorMotorSupplyVoltage.getValueAsDouble());
    DogLog.log("Elevator/Right Motor/device temp", rightElevatorMotorDeviceTemp.getValueAsDouble());
    DogLog.log(
        "Elevator/Right Motor/stator current", rightElevatorMotorStatorCurrent.getValueAsDouble());
    DogLog.log("Elevator/Right Motor/position", rightElevatorMotorPosition.getValueAsDouble());
  }
}
