package frc.robot.subsystems.endEffector;

import com.playingwithfusion.TimeOfFlight;
import com.playingwithfusion.TimeOfFlight.RangingMode;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.lang.Math;


public class EndEffectorSensor {
    public double m_distance_EMA;
    public double m_dist_SDEV;


  private TimeOfFlight sensor = new TimeOfFlight(EndEffectorConstants.TOF_DEVICE_ID);

  public EndEffectorSensor() {
    sensor.setRangingMode(RangingMode.Short, 0.24); // Will need to test. must be between 24-1000ms
    m_distance_EMA = sensor.getRange();
    m_dist_SDEV = 0; 


  }

  public void periodic() {
    
    m_distance_EMA = (EndEffectorConstants.EXP_DECAY * sensor.getRange()) + ((1 - EndEffectorConstants.EXP_DECAY)*m_distance_EMA);
    m_dist_SDEV = (EndEffectorConstants.EXP_DECAY * Math.pow((m_distance_EMA - sensor.getRange()), 2)) + ((1 - EndEffectorConstants.EXP_DECAY)*m_dist_SDEV);
  }

  public double getRange() {
    return sensor.getRange();
  }

  public void robotPeriodic() {
    double distance = sensor.getRange();
    SmartDashboard.putNumber("Distance", distance);
    
  }
}
