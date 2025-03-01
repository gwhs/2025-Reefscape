package frc.robot.subsystems.groundIntake;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class groundIntakeSubsystem extends SubsystemBase {

  groundIntakeIO groundintakeIO;

  public groundIntakeSubsystem() {
    if (RobotBase.isSimulation()) {
      groundintakeIO = new groundIntakeIOSim();
    } else {
      groundintakeIO = new groun
	      dIntakeIOReal();
    }
  }
	/**
	 * @param voltage the voltage to set it to
	 * @return run the command
	 */
	public Command setPivotMot`orVoltage(double voltage) {
		return Commands.runOnce( () -> groundintakeIO.setPivotMotorVoltage(voltage));
	}

	/**
	 * @param voltage the voltage to set it to
	 * @return run the command
	 */ 
	public Command setSpinMotorVoltage(double voltage) {
		return Commands.runOnce( () -> groundintakeIO.setSpinMotorVoltage(voltage));
	}

	@Override
	public void periodic() {
		groundintakeIO.update();
	}
}
