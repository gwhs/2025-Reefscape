package frc.robot.subsystems.arm;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import dev.doglog.DogLog;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;

public class ArmIOReal implements ArmIO {
  private TalonFX armMotor = new TalonFX(ArmConstants.ARM_MOTOR_ID, "rio");
  private final MotionMagicVoltage m_request = new MotionMagicVoltage(0);

  public ArmIOReal() {
    TalonFXConfiguration talonFXConfigs = new TalonFXConfiguration();
    MotorOutputConfigs motorOutput = talonFXConfigs.MotorOutput;
    MotionMagicConfigs motionMagicConfigs = talonFXConfigs.MotionMagic;
    Slot0Configs slot0Configs = talonFXConfigs.Slot0;
    SoftwareLimitSwitchConfigs softwareLimitSwitch = talonFXConfigs.SoftwareLimitSwitch;
    CurrentLimitsConfigs currentConfig = talonFXConfigs.CurrentLimits;
    FeedbackConfigs feedbackConfigs = new FeedbackConfigs();


    slot0Configs.kS = 0.25; // Add 0.25 V output to overcome static friction
    slot0Configs.kG = 0; // Add 0 V to overcome gravity
    slot0Configs.kV = 0.12; // A velocity target of 1 rps results in 0.12 V output
    slot0Configs.kA = 0.01; // An acceleration of 1 rps/s requires 0.01 V output
    slot0Configs.kP = 4.8; // A position error of 2.5 rotations results in 12 V output
    slot0Configs.kI = 0; // no output for integrated error
    slot0Configs.kD = 0.1; // A velocity error of 1 rps results in 0.1 V output
    
    // feedbackConfigs.FeedbackRemoteSensorID = 0;
    // feedbackConfigs.FeedbackRotorOffset = 0;
    feedbackConfigs.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
    feedbackConfigs.RotorToSensorRatio = ArmConstants.ARM_GEAR_RATIO;
    feedbackConfigs.SensorToMechanismRatio = 1;

    motionMagicConfigs.MotionMagicCruiseVelocity = ArmConstants.MAX_VELOCITY;
    motionMagicConfigs.MotionMagicAcceleration = ArmConstants.MAX_ACCELERATION;
    motionMagicConfigs.MotionMagicJerk = 1600; // Target jerk of 1600 rps/s/s (0.1 seconds)

    motorOutput.NeutralMode = NeutralModeValue.Coast;
    motorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    softwareLimitSwitch.ForwardSoftLimitEnable = true;
    softwareLimitSwitch.ForwardSoftLimitThreshold =
        Units.degreesToRotations(330) * ArmConstants.ARM_GEAR_RATIO;
    softwareLimitSwitch.ReverseSoftLimitEnable = true;
    softwareLimitSwitch.ReverseSoftLimitThreshold =
        Units.degreesToRotations(20) * ArmConstants.ARM_GEAR_RATIO;

    currentConfig.withStatorCurrentLimitEnable(true);
    currentConfig.withStatorCurrentLimit(15);

    TalonFXConfigurator leftElevatorConfigurator = armMotor.getConfigurator();
    leftElevatorConfigurator.apply(talonFXConfigs);

    SmartDashboard.putData(
        "Arm Command/reset to 90",
        Commands.runOnce(
                () ->
                    armMotor.setPosition(
                        Units.degreesToRotations(90) * ArmConstants.ARM_GEAR_RATIO))
            .ignoringDisable(true));
  }

  // set arm angle in degrees
  @Override
  public void setAngle(double angle) {
    armMotor.setControl(
        m_request.withPosition(Units.degreesToRotations(angle) * ArmConstants.ARM_GEAR_RATIO));
  }

  // geta arm position in degrees
  @Override
  public double getPosition() {
    return Units.rotationsToDegrees(armMotor.getRotorPosition(true).getValueAsDouble())
        / ArmConstants.ARM_GEAR_RATIO;
  }

  @Override
  public void update() {
    DogLog.log("Arm/pid goal", armMotor.getClosedLoopReference(true).getValueAsDouble());
    DogLog.log("Arm/motor voltage", armMotor.getMotorVoltage(true).getValueAsDouble());
    DogLog.log("Arm/supply voltage", armMotor.getSupplyVoltage(true).getValueAsDouble());
    DogLog.log("Arm/device temp", armMotor.getDeviceTemp(true).getValueAsDouble());
    DogLog.log("Arm/stator current", armMotor.getStatorCurrent(true).getValueAsDouble());
  }
}
