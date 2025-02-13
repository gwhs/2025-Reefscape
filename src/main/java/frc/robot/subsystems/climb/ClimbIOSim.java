package frc.robot.subsystems.climb;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class ClimbIOSim implements ClimbIO{

    private static final Matrix<N2, N2> A = new Matrix<>(Nat.N2(), Nat.N2());
    private static final Matrix<N2, N1> B = new Matrix<>(Nat.N2(), Nat.N1());
    private static final Matrix<N2, N2> C = new Matrix<>(Nat.N2(), Nat.N2());
    private static final Matrix<N2, N1> D = new Matrix<>(Nat.N2(), Nat.N1());

    public static final LinearSystem<N2, N1, N2> linearSystem = new LinearSystem<>(A, B, C, D);
    public DCMotorSim climbSim = new DCMotorSim(linearSystem, ClimbConstants.CLIMB_GEAR_RATIO, ClimbConstants.CLIMB_MEASUREMENT_STDEV);

    public ClimbIOSim() {

    }

    @Override
    public double getPosition() {
        return Units.radiansToDegrees(climbSim.getAngularPositionRad());
    }

    @Override
    public void setPosition(double voltage) {
        climbSim.setInputVoltage(voltage);
    }

    @Override
    public void update() {
        climbSim.update(0.20);
    }

}
