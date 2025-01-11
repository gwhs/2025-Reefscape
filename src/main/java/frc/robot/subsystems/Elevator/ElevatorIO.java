// Copied from https://github.com/gwhs/2024v2/blob/2025-beta/src/main/java/frc/robot/subsystems/ClimbSubsystem/ClimbIO.java
package frc.robot.subsystems.Elevator;

public interface ElevatorIO {

    public double getRightMotorPosition();
  
    public double getLeftMotorPosition();
  
    public void setLeftMotorSpeed(double speed);
  
    public void setRightMotorSpeed(double speed);
  
    public void update();
  
}
