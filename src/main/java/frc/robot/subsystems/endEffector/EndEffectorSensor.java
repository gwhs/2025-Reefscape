package frc.robot.subsystems.endEffector;

import com.playingwithfusion.TimeOfFlight;
import com.playingwithfusion.TimeOfFlight.RangingMode;
public class EndEffectorSensor {
    private TimeOfFlight sensor = new TimeOfFlight(EndEffectorConstants.TOF_DEVICE_ID);
    
    public EndEffectorSensor() {
        sensor.setRangingMode(RangingMode.Short, 0.24); // Will need to test. must be between 24-1000ms
    }

    public double getRange() {
        return sensor.getRange();
    }

}
