package frc.robot.subsystems.groundIntake;

import edu.wpi.first.wpilibj.RobotBase;

public class groundIntakeSubsystem {

	groundIntakeIO groundintakeIO;

	public groundIntakeSubsystem() {
		if (RobotBase.isSimulation()) {
			groundintakeIO = new groundIntakeIOSim();
		}
		else {
			groundintakeIO = new groundIntakeIOReal();
		}
	}


	
}
