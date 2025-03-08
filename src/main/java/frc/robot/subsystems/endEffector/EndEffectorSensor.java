package frc.robot.subsystems.endEffector;

import com.playingwithfusion.TimeOfFlight;
import com.playingwithfusion.TimeOfFlight.RangingMode;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class EndEffectorSensor {
  public double m_distance_EMA;
  public double m_dist_SDEV;
  public String mode = "Short";
  public int rangeX0 = -2;
  public int rangeY0 = -4;
  public int rangeX1 = 2;
  public int rangeY1 = -8;

  private TimeOfFlight sensor = new TimeOfFlight(EndEffectorConstants.TOF_DEVICE_ID);

  public EndEffectorSensor() {
    sensor.setRangingMode(RangingMode.Short, 0.24); // Will need to test. must be between 24-1000ms
    m_distance_EMA = sensor.getRange();
    m_dist_SDEV = 0;

    sensor.setRangeOfInterest(0, 11, 15, 15);
  }

  public double getRange() {
    return sensor.getRange();
  }

  public void robotPeriodic() {

    /*for (int topLeftRow = 0; topLeftRow < 12; topLeftRow++) {
      System.out.print("previous row");
      System.out.print(topLeftRow);
      for (int topLeftColumn = 0; topLeftColumn < 12; topLeftColumn++) {
        sensor.setRangeOfInterest(topLeftColumn, topLeftRow, topLeftColumn + 4, topLeftRow + 4);
        System.out.print(sensor.getRange() + ",");
        try {
          Thread.sleep(1000);
        } catch (InterruptedException e) {
          // TODO Auto-generated catch block
          e.printStackTrace();
        }
        System.out.print(sensor.getStatus());
      }
      System.out.println();
    }*/

    double distance = sensor.getRange();
    SmartDashboard.putNumber("Distance", distance);

    m_dist_SDEV =
        (EndEffectorConstants.SDEV_Decay * Math.pow((m_distance_EMA - sensor.getRange()), 2))
            + ((1 - EndEffectorConstants.SDEV_Decay) * m_dist_SDEV);
    m_distance_EMA =
        (EndEffectorConstants.EXP_DECAY * sensor.getRange())
            + ((1 - EndEffectorConstants.EXP_DECAY) * m_distance_EMA);

    SmartDashboard.putNumber("dist_EMA", m_distance_EMA);
    SmartDashboard.putNumber("dist_EDEV", m_dist_SDEV);
  }
}
