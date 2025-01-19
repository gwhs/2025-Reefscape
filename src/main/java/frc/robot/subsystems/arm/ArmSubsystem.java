package frc.robot.subsystems.arm;

import dev.doglog.DogLog;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmSubsystem extends SubsystemBase{
    private ArmIO armIO;

    public ArmSubsystem() {
        if (RobotBase.isSimulation()) {
            armIO = new ArmIOSim();
        } else {
            armIO = new ArmIOReal();
        }

        SmartDashboard.putData("turn to 60 degrees", setAngle(60));
        SmartDashboard.putData("turn to 120 degrees", setAngle(120));
        SmartDashboard.putData("turn to 270 degrees", setAngle(270));
        SmartDashboard.putData("reset to 0 degrees", setAngle(0));

        DogLog.log("Arm Subystem/arm angle", armIO.getPosition());
    }

    public Command setAngle(double angle) {
    return this.runOnce(
            () -> {
              armIO.setAngle(angle);
            })
        .andThen(
            Commands.waitUntil(
                () -> MathUtil.isNear(angle, armIO.getPosition(), 0.1)));        
    }
}
