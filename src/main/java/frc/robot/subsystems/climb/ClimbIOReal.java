package frc.robot.subsystems.climb;

import com.ctre.phoenix6.hardware.TalonFX;

public class ClimbIOReal implements ClimbIO {
    private TalonFX climbMotor = new TalonFX(ClimbConstants.CLIMB_ID, "rio");
    


    @Override
    public double getHeight() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'getHeight'");
    }

    @Override
    public double setHeight(double desiredHeight) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'setHeight'");
    }

    @Override
    public void update() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'update'");
    }
    
}
