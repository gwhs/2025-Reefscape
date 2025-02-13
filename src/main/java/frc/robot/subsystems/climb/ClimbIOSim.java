package frc.robot.subsystems.climb;

import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class ClimbIOSim implements ClimbIO{
    public DCMotorSim climbSim = new DCMotorSim(LinearSystem<N2,N1,N2> plant, ClimbConstants.CLIMB_GEAR_RATIO, ClimbConstants.CLIMB_MEASUREMENT_STDEV);

    public ClimbIOSim() {

    }

    @Override
    public double getPosition() {
        return Units.radiansToDegrees(climbSim.getAngularPositionRad());
    }

    @Override
    public void setPosition(double desiredHeight) {
        climbSim.setInputVoltage(desiredHeight);
    }

    @Override
    public void update() {
        climbSim.update(0.20);
    }

}
