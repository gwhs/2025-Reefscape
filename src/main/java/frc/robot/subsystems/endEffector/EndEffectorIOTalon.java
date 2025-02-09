package frc.robot.subsystems.endEffector;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorSensorV3;
import dev.doglog.DogLog;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;

class EndEffectorIOTalon implements EndEffectorIO {

  /**
   * This is a simple example to show how the REV Color Sensor V3 can be used to detect
   * pre-configured colors.
   */

  /** Change the I2C port below to match the connection of your color sensor */
  private final I2C.Port i2cPort = I2C.Port.kOnboard;

  /**
   * A Rev Color Sensor V3 object is constructed with an I2C port as a parameter. The device will be
   * automatically initialized with default parameters.
   */
  private final ColorSensorV3 m_colorSensor = new ColorSensorV3(i2cPort);

  /**
   * A Rev Color Match object is used to register and detect known colors. This can be calibrated
   * ahead of time or during operation.
   *
   * <p>This object uses a simple euclidian distance to estimate the closest match with given
   * confidence range.
   */
  private final ColorMatch m_colorMatcher = new ColorMatch();

  /**
   * Note: Any example colors should be calibrated as the user needs, these are here as a basic
   * example.
   */
  private final Color kBlueTarget = new Color(0.143, 0.427, 0.429);

  private final Color kGreenTarget = new Color(0.197, 0.561, 0.240);
  private final Color kRedTarget = new Color(0.561, 0.232, 0.114);
  private final Color kYellowTarget = new Color(0.361, 0.524, 0.113);
  private final Color kWhiteTarget = new Color(0.6, 0.6, 0.6);
  private final Color kCoralTarget = new Color(0.259, 0.478, 0.263);

  public void robotInit() {
    m_colorMatcher.addColorMatch(kBlueTarget);
    m_colorMatcher.addColorMatch(kGreenTarget);
    m_colorMatcher.addColorMatch(kRedTarget);
    m_colorMatcher.addColorMatch(kYellowTarget);
    m_colorMatcher.addColorMatch(kWhiteTarget);
    m_colorMatcher.addColorMatch(kCoralTarget);

    // m_colorSensor.configureColorSensor(ColorSensorResolution.kColorSensorRes13bit,
    // ColorSensorMeasurementRate.kColorRate2000ms, GainFactor.kGain18x);
  }

  public void robotPeriodic() {
    /**
     * The method GetColor() returns a normalized color value from the sensor and can be useful if
     * outputting the color to an RGB LED or similar. To read the raw color, use GetRawColor().
     *
     * <p>The color sensor works best when within a few inches from an object in well lit conditions
     * (the built in LED is a big help here!). The farther an object is the more light from the
     * surroundings will bleed into the measurements and make it difficult to accurately determine
     * its color.
     */
    Color detectedColor = m_colorSensor.getColor();
    double distance = m_colorSensor.getProximity();

    /** Run the color match algorithm on our detected color */
    String colorString;
    ColorMatchResult match = m_colorMatcher.matchClosestColor(detectedColor);

    String position;

    if (match.color == kBlueTarget) {
      colorString = "Blue";
    } else if (match.color == kRedTarget) {
      colorString = "Red";
    } else if (match.color == kGreenTarget) {
      colorString = "Green";
    } else if (match.color == kYellowTarget) {
      colorString = "Yellow";
    } else if (match.color == kWhiteTarget) {
      colorString = "White";
    } else if (match.color == kCoralTarget) {
      colorString = "CoralWhite";
    } else {
      colorString = "Unknown";
    }

    if (distance > 1500) {
      position = "inside";
    } else if (distance < 1500) {
      position = "outside";
    } else {
      position = "unknown";
    }

    /** Open Smart Dashboard or Shuffleboard to see the color detected by the sensor. */
    SmartDashboard.putNumber("Red", detectedColor.red);
    SmartDashboard.putNumber("Green", detectedColor.green);
    SmartDashboard.putNumber("Blue", detectedColor.blue);
    SmartDashboard.putNumber("Confidence", match.confidence);
    SmartDashboard.putString("Detected Color", colorString);
    SmartDashboard.putNumber("Sensor Distance", distance);
    SmartDashboard.putString("Coral Position", position);
  }

  private TalonFX motor = new TalonFX(EndEffectorConstants.deviceID, "rio");
  private final StatusSignal<Voltage> volts = motor.getMotorVoltage();
  private final StatusSignal<AngularVelocity> velocity = motor.getVelocity();
  private final StatusSignal<Temperature> temperature = motor.getDeviceTemp();

  public EndEffectorIOTalon() {
    TalonFXConfiguration talonConfig = new TalonFXConfiguration();
    CurrentLimitsConfigs limitsConfigs = talonConfig.CurrentLimits;

    limitsConfigs.withStatorCurrentLimitEnable(true);
    limitsConfigs.withStatorCurrentLimit(15);

    StatusCode status = StatusCode.StatusCodeNotInitialized;

    for (int i = 0; i < 5; i++) {
      status = motor.getConfigurator().apply(talonConfig);
      if (status.isOK()) break;
    }
    if (!status.isOK()) {
      System.out.println("Could not configure device. Error: " + status.toString());
    }
  }

  @Override
  public void setVoltage(double voltage) {
    motor.setVoltage(voltage);
  }

  @Override
  public void stopMotor() {
    motor.setVoltage(0);
  }

  @Override
  public double getVelocity() {
    return velocity.getValueAsDouble();
  }

  @Override
  public double getVoltage() {

    return volts.getValueAsDouble();
  }

  @Override
  public void update() {
    boolean endEffectorConnected =
        (BaseStatusSignal.refreshAll(volts, velocity, temperature)).isOK();
    DogLog.log("EndEffector/Temperature", temperature.getValueAsDouble());
    DogLog.log("endEffector/Connected", endEffectorConnected);
  }
}
