package frc.robot.subsystems.climb;

import com.ctre.phoenix6.hardware.TalonFX;

public class ClimbIOReal implements ClimbIO {
    private TalonFX climbMotor = new TalonFX(ClimbConstants.CLIMB_ID, "rio");

    public ClimbIOReal() {

    }
    


    @Override
    public double getPosition() {
        return climbMotor.get();
    }

    @Override
    public void setPosition(double desiredPos) {
        climbMotor.setVoltage(desiredPos);
    }

    @Override
    public void update() {
    }
    
}
