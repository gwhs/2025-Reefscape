package frc.robot.subsystems.Elevator;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class ElevatorIOReal implements ElevatorIO {
  private TalonFX m_leftElevatorMotor =
      new TalonFX(ElevatorConstants.LEFT_ELEVATOR_MOTOR_ID, "rio");
  private TalonFX m_rightElevatorMotor =
      new TalonFX(ElevatorConstants.RIGHT_ELEVATOR_MOTOR_ID, "rio");

  private final MotionMagicVoltage m_requestLeft = new MotionMagicVoltage(0);
  private final Follower m_requestRight =
      new Follower(ElevatorConstants.LEFT_ELEVATOR_MOTOR_ID, true);

  public ElevatorIOReal() {
    TalonFXConfiguration talonFXConfigs = new TalonFXConfiguration();
    MotorOutputConfigs motorOutput = talonFXConfigs.MotorOutput;
    CurrentLimitsConfigs currentConfig = talonFXConfigs.CurrentLimits;
    Slot0Configs slot0Configs = talonFXConfigs.Slot0;
    MotionMagicConfigs motionMagicConfigs = talonFXConfigs.MotionMagic;

    slot0Configs.kS = 0.25; // Add 0.25 V output to overcome static friction
    slot0Configs.kV = 0.12; // A velocity target of 1 rps results in 0.12 V output
    slot0Configs.kA = 0.01; // An acceleration of 1 rps/s requires 0.01 V output
    slot0Configs.kP = 4.8; // A position error of 2.5 rotations results in 12 V output
    slot0Configs.kI = 0; // no output for integrated error
    slot0Configs.kD = 0.1; // A velocity error of 1 rps results in 0.1 V output
    motionMagicConfigs.MotionMagicCruiseVelocity = 80; // Target cruise velocity of 80 rps
    motionMagicConfigs.MotionMagicAcceleration =
        160; // Target acceleration of 160 rps/s (0.5 seconds)
    motionMagicConfigs.MotionMagicJerk = 1600; // Target jerk of 1600 rps/s/s (0.1 seconds)

    currentConfig.withStatorCurrentLimitEnable(true);
    currentConfig.withStatorCurrentLimit(60);
    motorOutput.NeutralMode = NeutralModeValue.Brake;
    motorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    TalonFXConfigurator leftElevatorConfigurator = m_leftElevatorMotor.getConfigurator();
    leftElevatorConfigurator.apply(talonFXConfigs);

    // TODO:
    // motorOutput.Inverted = InvertedValue.Clockwise_Positive;

    TalonFXConfigurator rightElevatorConfigurator = m_rightElevatorMotor.getConfigurator();
    rightElevatorConfigurator.apply(talonFXConfigs);

    // TODO:
    // encoder settings
    // FeedbackConfigs
    // FeedbackConfigs feedbackConfigs = talonFXConfigs.Feedback;
    // https://api.ctr-electronics.com/phoenix6/release/java/com/ctre/phoenix6/configs/FeedbackConfigs.html

    // TODO:
    // limit switch settings
    // HardwareLimitSwitchConfigs
    // HardwareLimitSwitchConfigs hardwareLimitSwitchConfigs = talonFXConfigs.HardwareLimitSwitch;
    // https://api.ctr-electronics.com/phoenix6/release/java/com/ctre/phoenix6/configs/HardwareLimitSwitchConfigs.html

  }

  public void setPosition(double position) {
    m_leftElevatorMotor.setControl(m_requestLeft.withPosition(position));
    m_rightElevatorMotor.setControl(m_requestRight);
  }

  public double getPosition() {
    return m_leftElevatorMotor.getPosition().getValueAsDouble();
  }

  @Override
  public void update() {}
}
