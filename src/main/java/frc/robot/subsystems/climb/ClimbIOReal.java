package frc.robot.subsystems.climb;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import dev.doglog.DogLog;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;

public class ClimbIOReal implements ClimbIO {
  private final TalonFX climbMotor = new TalonFX(ClimbConstants.CLIMB_ID, "rio");

  private final MotionMagicVoltage m_request = new MotionMagicVoltage(0).withEnableFOC(true);

  private final StatusSignal<Double> climbPIDGoal = climbMotor.getClosedLoopReference();
  private final StatusSignal<Voltage> climbMotorVoltage = climbMotor.getMotorVoltage();
  private final StatusSignal<Current> climbStatorCurrent = climbMotor.getStatorCurrent();
  private final StatusSignal<Angle> climbPosition = climbMotor.getPosition();

  private final Alert climbMotorConnectedAlert =
      new Alert("Climb motor not connected", AlertType.kError);

  public ClimbIOReal() {
    TalonFXConfiguration talonFXConfigs = new TalonFXConfiguration();
    MotorOutputConfigs motorOutput = talonFXConfigs.MotorOutput;
    MotionMagicConfigs motionMagicConfigs = talonFXConfigs.MotionMagic;
    Slot0Configs slot0Configs = talonFXConfigs.Slot0;
    SoftwareLimitSwitchConfigs softwareLimitSwitch = talonFXConfigs.SoftwareLimitSwitch;
    CurrentLimitsConfigs currentConfig = talonFXConfigs.CurrentLimits;
    FeedbackConfigs feedbackConfigs = talonFXConfigs.Feedback;

    slot0Configs.kS = 0.18205; // Add 0.25 V output to overcome static friction
    slot0Configs.kG = 0.09885; // Add 0 V to overcome gravity
    slot0Configs.kV = 7.2427; // A velocity target of 1 rps results in 0.12 V output
    slot0Configs.kA = 0.086264; // An acceleration of 1 rps/s requires 0.01 V output
    slot0Configs.kP = 57.759; // A position error of 2.5 rotations results in 12 V output
    slot0Configs.kI = 0; // no output for integrated error
    slot0Configs.kD = 8.4867; // A velocity error of 1 rps results in 0.1 V output
    slot0Configs.withGravityType(GravityTypeValue.Arm_Cosine);

    feedbackConfigs.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
    feedbackConfigs.RotorToSensorRatio = 1;
    feedbackConfigs.SensorToMechanismRatio = ClimbConstants.CLIMB_GEAR_RATIO;

    motionMagicConfigs.MotionMagicCruiseVelocity = ClimbConstants.MAX_VELOCITY;
    motionMagicConfigs.MotionMagicAcceleration = ClimbConstants.MAX_ACCELERATION;
    motionMagicConfigs.MotionMagicJerk = 0;

    motorOutput.NeutralMode = NeutralModeValue.Brake;
    motorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    softwareLimitSwitch.ForwardSoftLimitEnable = true;
    softwareLimitSwitch.ForwardSoftLimitThreshold = ClimbConstants.CLIMB_CLIMB_POSITION;
    softwareLimitSwitch.ReverseSoftLimitEnable = true;
    softwareLimitSwitch.ReverseSoftLimitThreshold =
        Units.degreesToRotations(ClimbConstants.STOW_CLIMB_POSITION);

    currentConfig.withStatorCurrentLimitEnable(true);
    currentConfig.withStatorCurrentLimit(20);

    StatusCode status = StatusCode.StatusCodeNotInitialized;
    for (int i = 0; i < 5; i++) {
      status = climbMotor.getConfigurator().apply(talonFXConfigs);
      if (status.isOK()) break;
    }
    if (!status.isOK()) {
      System.out.println("Could not configure device. Error: " + status.toString());
    }

    BaseStatusSignal.setUpdateFrequencyForAll(50.0, climbPIDGoal, climbStatorCurrent);
  }

  @Override
  public void setPosition(double angle) {
    climbMotor.setControl(m_request.withPosition(angle));
  }

  @Override
  public double getPosition() {
    return Units.rotationsToDegrees(climbPosition.getValueAsDouble());
  }

  @Override
  public void update() {
    boolean armConnected =
        (BaseStatusSignal.refreshAll(
                climbPIDGoal, climbMotorVoltage, climbStatorCurrent, climbPosition)
            .isOK());

    DogLog.log("Climb/Motor/pid goal", climbPIDGoal.getValueAsDouble());
    DogLog.log("Climb/Motor/motor voltage", climbMotorVoltage.getValueAsDouble());
    DogLog.log("Climb/Motor/stator current", climbStatorCurrent.getValueAsDouble());
    DogLog.log("Climb/Motor/Connected", armConnected);

    climbMotorConnectedAlert.set(!climbMotor.isConnected());
  }
}
