package frc.robot.subsystems.climb;

import edu.wpi.first.math.system.LinearSystem;

public class ClimbConstants {
    public static final int CLIMB_ID = 56;
    public static final double CLIMB_GEAR_RATIO = 64.0;
    public static final double CLIMB_MEASUREMENT_STDEV = 0.1;
    public static final LinearSystem<N2,N1,N2> linearSystem = new LinearSystem<>(null, null, null, null);
    
}
