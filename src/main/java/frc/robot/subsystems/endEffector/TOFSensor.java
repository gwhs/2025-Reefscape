package frc.robot.subsystems.endEffector;

import com.playingwithfusion.TimeOfFlight;
import com.playingwithfusion.TimeOfFlight.RangingMode;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class TOFSensor {
  public double m_distance_EMA;
  public double m_dist_SDEV_sq;
  // private String mode = "Short";

  int m_rangeX0;
  int m_rangeY0;
  int m_rangeX1;
  int m_rangeY1;

  private TimeOfFlight sensor;

  public TOFSensor(int CAN_ID) {
    sensor = new TimeOfFlight(CAN_ID);
    setRangeOfInterest(0, 0, 15, 15);
    sensor.setRangingMode(RangingMode.Short, 24); // Will need to test. must be between 24-1000ms
    m_distance_EMA = sensor.getRange();
    m_dist_SDEV_sq = 0;
  }

  public TimeOfFlight.Status getStatus() {
    return sensor.getStatus();
  }

  public void setRangeOfInterest(int rangeX0, int rangeY0, int rangeX1, int rangeY1) {
    m_rangeX0 = rangeX0;
    m_rangeY0 = rangeY0;
    m_rangeX1 = rangeX1;
    m_rangeY1 = rangeY1;

    sensor.setRangeOfInterest(m_rangeX0, m_rangeY0, m_rangeX1, m_rangeY1);
    SmartDashboard.putNumber("rangeX0", m_rangeX0);
    SmartDashboard.putNumber("rangeY0", m_rangeY0);
    SmartDashboard.putNumber("rangeX1", m_rangeX1);
    SmartDashboard.putNumber("rangeY1", m_rangeY1);
  }

  public double getRange() {
    return sensor.getRange();
  }

  public void robotPeriodic() {

    double distance = sensor.getRange();
    SmartDashboard.putNumber("Distance", distance);

    double diff_from_EMA = m_distance_EMA - sensor.getRange();
    m_dist_SDEV_sq =
        (EndEffectorConstants.SDEV_DECAY * diff_from_EMA * diff_from_EMA)
            + ((1 - EndEffectorConstants.SDEV_DECAY) * m_dist_SDEV_sq);
    m_distance_EMA =
        (EndEffectorConstants.EXP_DECAY * sensor.getRange())
            + ((1 - EndEffectorConstants.EXP_DECAY) * m_distance_EMA);

    SmartDashboard.putNumber("dist_EMA", m_distance_EMA);
    SmartDashboard.putNumber("dist_EDEV", Math.sqrt(m_dist_SDEV_sq));
    SmartDashboard.putString("sensor_Status", sensor.getStatus().toString());
    SmartDashboard.putNumber("getDist_Sigma", sensor.getRangeSigma());
  }
}
