package frc.robot.subsystems.elevator;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.HardwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DifferentialMotionMagicVoltage;
import com.ctre.phoenix6.controls.DifferentialVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.mechanisms.SimpleDifferentialMechanism;
import com.ctre.phoenix6.signals.ForwardLimitSourceValue;
import com.ctre.phoenix6.signals.ForwardLimitTypeValue;
import com.ctre.phoenix6.signals.ForwardLimitValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.ReverseLimitSourceValue;
import com.ctre.phoenix6.signals.ReverseLimitTypeValue;
import com.ctre.phoenix6.signals.ReverseLimitValue;
import dev.doglog.DogLog;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DigitalInput;

public class ElevatorIOReal implements ElevatorIO {

  private TalonFX m_frontElevatorMotor =
      new TalonFX(ElevatorConstants.FRONT_ELEVATOR_MOTOR_ID, "rio");
  public TalonFX m_backElevatorMotor = new TalonFX(ElevatorConstants.BACK_ELEVATOR_MOTOR_ID, "rio");
  public DigitalInput limitSwitch = new DigitalInput(ElevatorConstants.LIMIT_SWITCH_CHANNEL);

  private final DifferentialMotionMagicVoltage m_request =
      new DifferentialMotionMagicVoltage(0, 0).withEnableFOC(true);
  private final SimpleDifferentialMechanism differentialMechanism =
      new SimpleDifferentialMechanism(m_frontElevatorMotor, m_backElevatorMotor, false);

  private final DifferentialVoltage m_requestVoltage = new DifferentialVoltage(0, 0);

  private final StatusSignal<Double> frontElevatorMotorPIDGoal =
      m_frontElevatorMotor.getClosedLoopReference();
  private final StatusSignal<Voltage> frontElevatorMotorVoltage =
      m_frontElevatorMotor.getMotorVoltage();
  private final StatusSignal<Current> frontElevatorMotorStatorCurrent =
      m_frontElevatorMotor.getStatorCurrent();
  private final StatusSignal<Angle> frontElevatorMotorPosition = m_frontElevatorMotor.getPosition();

  private final StatusSignal<ForwardLimitValue> forwardLimit =
      m_frontElevatorMotor.getForwardLimit();
  private final StatusSignal<ReverseLimitValue> reverseLimit =
      m_frontElevatorMotor.getReverseLimit();

  private final StatusSignal<Double> backElevatorMotorPIDGoal =
      m_backElevatorMotor.getClosedLoopReference();
  private final StatusSignal<Voltage> backElevatorMotorVoltage =
      m_backElevatorMotor.getMotorVoltage();
  private final StatusSignal<Current> backElevatorMotorStatorCurrent =
      m_backElevatorMotor.getStatorCurrent();
  private final StatusSignal<Angle> backElevatorMotorPosition = m_backElevatorMotor.getPosition();

  private final Alert frontElevatorMotorConnectedAlert =
      new Alert("Front Elevator Motor Not Connected", AlertType.kError);
  private final Alert backElevatorMotorConnectedAlert =
      new Alert("Back Elevator Motor Not Connected", AlertType.kError);

  private boolean m_emergencyMode;

  public ElevatorIOReal() {
    TalonFXConfiguration talonFXConfigs = new TalonFXConfiguration();
    MotorOutputConfigs motorOutput = talonFXConfigs.MotorOutput;
    CurrentLimitsConfigs currentConfig = talonFXConfigs.CurrentLimits;
    Slot0Configs slot0Configs = talonFXConfigs.Slot0;
    MotionMagicConfigs motionMagicConfigs = talonFXConfigs.MotionMagic;
    HardwareLimitSwitchConfigs hardwareLimitSwitchConfigs = talonFXConfigs.HardwareLimitSwitch;
    SoftwareLimitSwitchConfigs softwareLimitSwitchConfigs = talonFXConfigs.SoftwareLimitSwitch;

    slot0Configs.kS = 0.049358; // Add 0.25 V output to overcome static friction
    slot0Configs.kG = 0.049961; // Add 0 voltage to overcome gravity
    slot0Configs.kV = 0.10924; // A velocity target of 1 rps results in 0.12 V output
    slot0Configs.kA = 0.0013678; // An acceleration of 1 rps/s requires 0.01 V output
    slot0Configs.kP = 4.8; // A position error of 2.5 rotations results in 12 V output
    slot0Configs.kI = 0; // no output for integrated error
    slot0Configs.kD = 0.1; // A velocity error of 1 rps results in 0.1 V output
    slot0Configs.withGravityType(GravityTypeValue.Elevator_Static);

    motionMagicConfigs.MotionMagicCruiseVelocity = ElevatorConstants.MAX_VELOCITY;
    motionMagicConfigs.MotionMagicAcceleration = ElevatorConstants.MAX_ACCELERATION;
    motionMagicConfigs.MotionMagicJerk = 0;

    currentConfig.withStatorCurrentLimitEnable(true);
    currentConfig.withStatorCurrentLimit(30);
    motorOutput.NeutralMode = NeutralModeValue.Coast;
    motorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    softwareLimitSwitchConfigs.ForwardSoftLimitEnable = true;
    softwareLimitSwitchConfigs.ReverseSoftLimitEnable = true;
    softwareLimitSwitchConfigs.ForwardSoftLimitThreshold =
        ElevatorSubsystem.metersToRotations(ElevatorConstants.TOP_METER);
    softwareLimitSwitchConfigs.ReverseSoftLimitThreshold = ElevatorSubsystem.metersToRotations(0);

    hardwareLimitSwitchConfigs.ForwardLimitSource = ForwardLimitSourceValue.LimitSwitchPin;
    hardwareLimitSwitchConfigs.ReverseLimitSource = ReverseLimitSourceValue.LimitSwitchPin;
    hardwareLimitSwitchConfigs.ForwardLimitEnable = true;
    hardwareLimitSwitchConfigs.ReverseLimitEnable = true;
    hardwareLimitSwitchConfigs.ForwardLimitType = ForwardLimitTypeValue.NormallyOpen;
    hardwareLimitSwitchConfigs.ReverseLimitType = ReverseLimitTypeValue.NormallyOpen;
    hardwareLimitSwitchConfigs.ReverseLimitAutosetPositionEnable = false;
    hardwareLimitSwitchConfigs.ReverseLimitAutosetPositionValue = 0;

    StatusCode backStatus = StatusCode.StatusCodeNotInitialized;
    for (int i = 0; i < 5; i++) {
      backStatus = m_backElevatorMotor.getConfigurator().apply(talonFXConfigs);
      if (backStatus.isOK()) break;
    }
    if (!backStatus.isOK()) {
      System.out.println("Could not configure device. Error: " + backStatus.toString());
    }

    motorOutput.Inverted = InvertedValue.Clockwise_Positive;

    StatusCode frontStatus = StatusCode.StatusCodeNotInitialized;
    for (int i = 0; i < 5; i++) {
      frontStatus = m_frontElevatorMotor.getConfigurator().apply(talonFXConfigs);
      if (frontStatus.isOK()) break;
    }
    if (!frontStatus.isOK()) {
      System.out.println("Could not configure device. Error: " + frontStatus.toString());
    }

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0,
        frontElevatorMotorPIDGoal,
        backElevatorMotorPIDGoal,
        frontElevatorMotorStatorCurrent,
        backElevatorMotorStatorCurrent);
  }

  public void setRotation(double rotation) {
    differentialMechanism.setControl(m_request.withTargetPosition(rotation));
  }

  public double getRotation() {
    return frontElevatorMotorPosition.getValueAsDouble();
  }

  public boolean getForwardLimit() {
    return forwardLimit.getValue().value == 0;
  }

  public boolean getReverseLimit() {
    return reverseLimit.getValue().value == 0;
  }

  public void setVoltage(double voltage) {
    if (m_emergencyMode == true) {
      differentialMechanism.setControl(m_requestVoltage.withTargetOutput(0));
    } else {
      differentialMechanism.setControl(m_requestVoltage.withTargetOutput(voltage));
    }
  }

  public void setNeutralMode(NeutralModeValue mode) {
    m_frontElevatorMotor.setNeutralMode(mode);
    m_backElevatorMotor.setNeutralMode(mode);
  }

  public void setPosition(double newValue) {
    if (m_emergencyMode == false) {
      m_frontElevatorMotor.setPosition(newValue);
      m_backElevatorMotor.setPosition(newValue);
    }
  }

  public void setEmergencyMode(boolean emergency) {
    m_emergencyMode = emergency;
  }

  @Override
  public void update() {
    BaseStatusSignal.refreshAll(
        frontElevatorMotorPIDGoal,
        frontElevatorMotorVoltage,
        frontElevatorMotorStatorCurrent,
        frontElevatorMotorPosition,
        forwardLimit,
        reverseLimit,
        backElevatorMotorPIDGoal,
        backElevatorMotorVoltage,
        backElevatorMotorStatorCurrent,
        backElevatorMotorPosition);
    DogLog.log("Elevator/Limit Switch/enabled", limitSwitch.get());
    DogLog.log("Elevator/Front Motor/pid goal", frontElevatorMotorPIDGoal.getValueAsDouble());
    DogLog.log("Elevator/Front Motor/motor voltage", frontElevatorMotorVoltage.getValueAsDouble());
    DogLog.log(
        "Elevator/Front Motor/stator current", frontElevatorMotorStatorCurrent.getValueAsDouble());
    DogLog.log("Elevator/Front Motor/position", frontElevatorMotorPosition.getValueAsDouble());

    DogLog.log("Elevator/Front Motor/Connected", m_frontElevatorMotor.isConnected());
    DogLog.log("Elevator/Back Motor/Connected", m_backElevatorMotor.isConnected());

    DogLog.log("Elevator/Back Motor/pid goal", backElevatorMotorPIDGoal.getValueAsDouble());
    DogLog.log("Elevator/Back Motor/motor voltage", backElevatorMotorVoltage.getValueAsDouble());
    DogLog.log(
        "Elevator/Back Motor/stator current", backElevatorMotorStatorCurrent.getValueAsDouble());
    DogLog.log("Elevator/Back Motor/position", backElevatorMotorPosition.getValueAsDouble());

    frontElevatorMotorConnectedAlert.set(!m_frontElevatorMotor.isConnected());
    backElevatorMotorConnectedAlert.set(!m_backElevatorMotor.isConnected());

    if (m_emergencyMode == true) {
      differentialMechanism.setStaticBrake();
    }
  }
}
