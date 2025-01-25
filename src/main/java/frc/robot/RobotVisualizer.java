// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import dev.doglog.DogLog;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.subsystems.Elevator.ElevatorConstants;

/** Add your docs here. */
public class RobotVisualizer {
  //private final ArmSubsystem armSubsystem;
  //private final IntakeSubsystem intakeSubsystem;
  private final Mechanism2d panel = new Mechanism2d(ROBOT_LENGTH, ROBOT_LENGTH*3);

  // Robot Constants
  public static final double ROBOT_LENGTH = Units.inchesToMeters(28);

  public Color color1 = Color.kFirstBlue;
  public Color color2 = Color.kFirstRed;

  // Code for the stick figure of each subsystems

  //elevator base

  MechanismRoot2d base = panel.getRoot("base", -0.31, 0.0175);
  MechanismLigament2d m_base = base.append(new MechanismLigament2d("baseL", 1.3, 0, 10, new Color8Bit(color1)));

  // Code for elevator 
  MechanismRoot2d root = panel.getRoot("elevator", 0, 0); 
  MechanismLigament2d m_elevator = root.append(new MechanismLigament2d("elevatorL", 1.5, 90, 10, new Color8Bit(color1)));

  MechanismRoot2d root2 = panel.getRoot("elevator2", 0.7, 0); 
  MechanismLigament2d m_elevator2 = root2.append(new MechanismLigament2d("elevatorL2", 1.5, 90, 10, new Color8Bit(color1)));

// inner elevator
  MechanismRoot2d root3 = panel.getRoot("elevator3", 0.04, 0.078); 
  MechanismLigament2d m_elevator3 = root3.append(new MechanismLigament2d("elevatorL3", 1.5, 90, 10, new Color8Bit(color2)));

  MechanismRoot2d root4 = panel.getRoot("elevator4", 0.66, 0.078); 
  MechanismLigament2d m_elevator4 = root4.append(new MechanismLigament2d("elevatorL4", 1.5, 90, 10, new Color8Bit(color2)));

  MechanismLigament2d m_elevatorH2 = m_elevator3.append(new MechanismLigament2d("horizontal2", 0.62, -90, 10, new Color8Bit(color2)));
  MechanismLigament2d m_elevatorH3 = m_elevator3.append(new MechanismLigament2d("horizontal3", 0.31, -90, 10, new Color8Bit(color2)));

  MechanismLigament2d m_elevatorH1 = root3.append(new MechanismLigament2d("horizontal1", 0.62, 0, 10, new Color8Bit(color2)));

  //arm
  MechanismLigament2d arm = m_elevatorH3.append(new MechanismLigament2d("arm", 0.85, -90, 10, new Color8Bit(Color.kWhite)));

  //MechanismLigament2d arm = m_elevator2.append(new MechanismLigament2d("arm", 0.5, 0));

 // Overlapping arm height with superstructure to illustrate both superstructure and arm height
  //MechanismLigament2d m_armJoint = root.append(new MechanismLigament2d("armL", 0.78, 90));

  // Code for arm
  //MechanismLigament2d m_arm = m_armJoint.append(new MechanismLigament2d("arm", 0.38, 270));

  // code for pizzaBox
  // need to figure out how to append into center of arm
 // MechanismLigament2d m_pizzaBox1 = m_arm.append(new MechanismLigament2d("pizzaBox1", 0.205, 90));
  //MechanismLigament2d m_pizzaBox2 = m_arm.append(new MechanismLigament2d("pizzaBox2", 0.205, -90));

  // TO DO: use MechanismRoot2d and MechanismLigament2d to form stick figures that represent the intake
  // Example of attaching to roots/ligaments: MechanismLigament2d m_spinner = m_arm.append(new MechanismLigament2d("spinner", 0.2, 270, 12, new Color8Bit(color2)));
  
  // code for intakes
  // needs fixing
  /**MechanismRoot2d root1 = panel.getRoot("intakeJoint", 0.57, 0.15);
  MechanismLigament2d m_intakeArm1 = root1.append(new MechanismLigament2d("intakeArm", 0.34 , 80));
  MechanismLigament2d m_intakeArm2 = m_intakeArm1.append(new MechanismLigament2d("intake arm 2", 0.23, -135));
  */

 /* public Mechanism2D(ArmSubsystem armSubsystem, IntakeSubsystem intakeSubsystem) {
    this.armSubsystem = armSubsystem;
    this.intakeSubsystem = intakeSubsystem; 
  }
*/
 public void update() {
    SmartDashboard.putData("RobotVisualizer", panel);
    
  } 
}
  