package frc.robot.subsystems.endEffector;

public class EndEffectorIODisabled implements EndEffectorIO {


@Override
public void setAmps(double current, double dutyCycle) {
	
}	


@Override
public void setVoltage(double voltage) {
		
}

@Override
public void update() {
	
}




@Override
public double getVoltage() {
	return 0;
}

@Override
public double getVelocity() {
	return 0;
}

@Override
public boolean coralLoaded() {
	return false;
}



@Override
public void stopMotor() {
	
}
}
