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
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
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
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;

public class ElevatorIOReal implements ElevatorIO {
  private TalonFX m_frontElevatorMotor =
      new TalonFX(ElevatorConstants.FRONT_ELEVATOR_MOTOR_ID, "rio");
  public TalonFX m_backElevatorMotor = new TalonFX(ElevatorConstants.BACK_ELEVATOR_MOTOR_ID, "rio");

  private final MotionMagicVoltage m_requestFront = new MotionMagicVoltage(0);
  private final Follower m_requestBack =
      new Follower(ElevatorConstants.FRONT_ELEVATOR_MOTOR_ID, true);

  private final VoltageOut m_requestFrontVoltage = new VoltageOut(0);

  private final StatusSignal<Double> frontElevatorMotorPIDGoal =
      m_frontElevatorMotor.getClosedLoopReference();
  private final StatusSignal<Voltage> frontElevatorMotorVoltage =
      m_frontElevatorMotor.getMotorVoltage();
  private final StatusSignal<Voltage> frontElevatorMotorSupplyVoltage =
      m_frontElevatorMotor.getSupplyVoltage();
  private final StatusSignal<Temperature> frontElevatorMotorDeviceTemp =
      m_frontElevatorMotor.getDeviceTemp();
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
  private final StatusSignal<Voltage> backElevatorMotorSupplyVoltage =
      m_backElevatorMotor.getSupplyVoltage();
  private final StatusSignal<Temperature> backElevatorMotorDeviceTemp =
      m_backElevatorMotor.getDeviceTemp();
  private final StatusSignal<Current> backElevatorMotorStatorCurrent =
      m_backElevatorMotor.getStatorCurrent();
  private final StatusSignal<Angle> backElevatorMotorPosition = m_backElevatorMotor.getPosition();

  private final Alert frontMotorConnectedAlert = new Alert ("Front motor Not Connected", AlertType.kError);
  private final Alert backMotorConnectedAlert = new Alert ("Back Motor Not Connected", AlertType.kError);

  public ElevatorIOReal() {
    TalonFXConfiguration talonFXConfigs = new TalonFXConfiguration();
    MotorOutputConfigs motorOutput = talonFXConfigs.MotorOutput;
    CurrentLimitsConfigs currentConfig = talonFXConfigs.CurrentLimits;
    Slot0Configs slot0Configs = talonFXConfigs.Slot0;
    MotionMagicConfigs motionMagicConfigs = talonFXConfigs.MotionMagic;
    HardwareLimitSwitchConfigs hardwareLimitSwitchConfigs = talonFXConfigs.HardwareLimitSwitch;
    SoftwareLimitSwitchConfigs softwareLimitSwitchConfigs = talonFXConfigs.SoftwareLimitSwitch;
    m_requestFront.EnableFOC = true; // this might also do it for back because it is a follower
    slot0Configs.kS = 0.25; // Add 0.25 V output to overcome static friction
    slot0Configs.kG = 0; // Add 0 voltage to overcome gravity
    slot0Configs.kV = 0.12; // A velocity target of 1 rps results in 0.12 V output
    slot0Configs.kA = 0.01; // An acceleration of 1 rps/s requires 0.01 V output
    slot0Configs.kP = 4.8; // A position error of 2.5 rotations results in 12 V output
    slot0Configs.kI = 0; // no output for integrated error
    slot0Configs.kD = 0.1; // A velocity error of 1 rps results in 0.1 V output
    slot0Configs.withGravityType(GravityTypeValue.Elevator_Static);

    motionMagicConfigs.MotionMagicCruiseVelocity = ElevatorConstants.MAX_VELOCITY;
    motionMagicConfigs.MotionMagicAcceleration = ElevatorConstants.MAX_ACCELERATION;
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

    hardwareLimitSwitchConfigs.ForwardLimitSource = ForwardLimitSourceValue.LimitSwitchPin;
    hardwareLimitSwitchConfigs.ReverseLimitSource = ReverseLimitSourceValue.LimitSwitchPin;
    hardwareLimitSwitchConfigs.ForwardLimitEnable = false;
    hardwareLimitSwitchConfigs.ReverseLimitEnable = true;
    hardwareLimitSwitchConfigs.ForwardLimitType = ForwardLimitTypeValue.NormallyOpen;
    hardwareLimitSwitchConfigs.ReverseLimitType = ReverseLimitTypeValue.NormallyOpen;
    hardwareLimitSwitchConfigs.ReverseLimitAutosetPositionEnable = true;
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

    // Set back motor to follow front motor
    m_backElevatorMotor.setControl(m_requestBack);

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0,
        frontElevatorMotorPIDGoal,
        backElevatorMotorPIDGoal,
        frontElevatorMotorStatorCurrent,
        backElevatorMotorStatorCurrent);
  }

  public void setRotation(double rotation) {
    m_frontElevatorMotor.setControl(m_requestFront.withPosition(rotation));
    m_backElevatorMotor.setControl(m_requestBack);
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
    m_frontElevatorMotor.setControl(m_requestFrontVoltage.withOutput(voltage));
    m_backElevatorMotor.setControl(m_requestBack);
  }

  public void setNeutralMode(NeutralModeValue mode) {
    m_frontElevatorMotor.setNeutralMode(mode);
    m_backElevatorMotor.setNeutralMode(mode);
  }

  @Override
  public void update() {
    boolean frontElevatorConnected =
        (BaseStatusSignal.refreshAll(
                frontElevatorMotorPIDGoal,
                frontElevatorMotorVoltage,
                frontElevatorMotorSupplyVoltage,
                frontElevatorMotorDeviceTemp,
                frontElevatorMotorStatorCurrent,
                frontElevatorMotorPosition,
                forwardLimit,
                reverseLimit)
            .isOK());

    boolean backElevatorConnected =
        (BaseStatusSignal.refreshAll(
                backElevatorMotorPIDGoal,
                backElevatorMotorVoltage,
                backElevatorMotorSupplyVoltage,
                backElevatorMotorDeviceTemp,
                backElevatorMotorStatorCurrent,
                backElevatorMotorPosition)
            .isOK());

    DogLog.log("Elevator/Front Motor/pid goal", frontElevatorMotorPIDGoal.getValueAsDouble());
    DogLog.log("Elevator/Front Motor/motor voltage", frontElevatorMotorVoltage.getValueAsDouble());
    DogLog.log(
        "Elevator/Front Motor/supply voltage", frontElevatorMotorSupplyVoltage.getValueAsDouble());
    DogLog.log("Elevator/Front Motor/device temp", frontElevatorMotorDeviceTemp.getValueAsDouble());
    DogLog.log(
        "Elevator/Front Motor/stator current", frontElevatorMotorStatorCurrent.getValueAsDouble());
    DogLog.log("Elevator/Front Motor/position", frontElevatorMotorPosition.getValueAsDouble());

    DogLog.log("Elevator/Front Motor/Connected", frontElevatorConnected);
    DogLog.log("Elevator/Back Motor/Connected", backElevatorConnected);

    DogLog.log("Elevator/Back Motor/pid goal", backElevatorMotorPIDGoal.getValueAsDouble());
    DogLog.log("Elevator/Back Motor/motor voltage", backElevatorMotorVoltage.getValueAsDouble());
    DogLog.log(
        "Elevator/Back Motor/supply voltage", backElevatorMotorSupplyVoltage.getValueAsDouble());
    DogLog.log("Elevator/Back Motor/device temp", backElevatorMotorDeviceTemp.getValueAsDouble());
    DogLog.log(
        "Elevator/Back Motor/stator current", backElevatorMotorStatorCurrent.getValueAsDouble());
    DogLog.log("Elevator/Back Motor/position", backElevatorMotorPosition.getValueAsDouble());

    frontMotorConnectedAlert.set(!frontElevatorConnected);
    backMotorConnectedAlert.set(!backElevatorConnected);
    
  }
}
