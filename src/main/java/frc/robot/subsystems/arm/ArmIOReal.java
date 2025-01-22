package frc.robot.subsystems.Arm;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import dev.doglog.DogLog;

public class ArmIOReal implements ArmIO {
  private TalonFX armMotor = new TalonFX(ArmConstants.ARM_MOTOR_ID, "rio");
  private final MotionMagicVoltage m_request = new MotionMagicVoltage(0);

  public ArmIOReal() {
    TalonFXConfiguration talonFXConfigs = new TalonFXConfiguration();
    MotorOutputConfigs motorOutput = talonFXConfigs.MotorOutput;
    MotionMagicConfigs motionMagicConfigs = talonFXConfigs.MotionMagic;
    Slot0Configs slot0Configs = talonFXConfigs.Slot0;
    SoftwareLimitSwitchConfigs softwareLimitSwitch = talonFXConfigs.SoftwareLimitSwitch;

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

    motorOutput.NeutralMode = NeutralModeValue.Coast;
    motorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    softwareLimitSwitch.ForwardSoftLimitEnable = true;
    softwareLimitSwitch.ForwardSoftLimitThreshold = 270 * ArmConstants.ARM_GEAR_RATIO;
    softwareLimitSwitch.ReverseSoftLimitEnable = true;
    softwareLimitSwitch.ReverseSoftLimitThreshold = 0;

    TalonFXConfigurator leftElevatorConfigurator = armMotor.getConfigurator();
    leftElevatorConfigurator.apply(talonFXConfigs);
  }

  @Override
  public void setAngle(double angle) {
    armMotor.setControl(m_request.withPosition(angle * ArmConstants.ARM_GEAR_RATIO));
  }

  @Override
  public double getPosition() {
    return armMotor.getPosition(true).getValueAsDouble() / ArmConstants.ARM_GEAR_RATIO;
  }

  @Override
  public void update() {
    DogLog.log("Arm/pid goal", armMotor.getClosedLoopReference(true).getValueAsDouble());
    DogLog.log("Arm/motor voltage", armMotor.getMotorVoltage(true).getValueAsDouble());
    DogLog.log("Arm/ supply voltage", armMotor.getSupplyVoltage(true).getValueAsDouble());
    DogLog.log("Arm/device temp", armMotor.getDeviceTemp(true).getValueAsDouble());
    DogLog.log("Arm/stator current", armMotor.getStatorCurrent(true).getValueAsDouble());
  }
}
