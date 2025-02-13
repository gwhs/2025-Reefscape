package frc.robot.subsystems.climb;

public interface ClimbIO {
    public double getPosition();

    public void setPosition(double desiredHeight);

    public void update();
}
