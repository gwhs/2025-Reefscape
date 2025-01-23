// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
//import frc.robot.subsystems.Arm.ArmSubsystem;
//import frc.robot.subsystems.Intake.IntakeSubsystem;

/** Add your docs here. */
public class Mechanism2D {
  //private final ArmSubsystem armSubsystem;
  //private final IntakeSubsystem intakeSubsystem;
  private final Mechanism2d panel = new Mechanism2d(ROBOT_LENGTH, ROBOT_LENGTH*3);
 // private final MechanismLigament2d elevator;

  // Robot Constants
  public static final double ROBOT_LENGTH = Units.inchesToMeters(28);

  // Code for the stick figure of each subsystems
  // Code for elevator 
  Mechanism2d mech = new Mechanism2d( 0.44, 0); 
  MechanismRoot2d root = mech.getRoot("elevator", ROBOT_LENGTH, ROBOT_LENGTH);

  elevator = root.append(new MechanismLigament2d("elevator", kElevatorMinimumLength, 90));
 // Overlapping arm height with superstructure to illustrate both superstructure and arm height
  //MechanismLigament2d m_armJoint = root.append(new MechanismLigament2d("armL", 0.78, 90));

  // Code for arm
  //MechanismLigament2d m_arm = m_armJoint.append(new MechanismLigament2d("arm", 0.38, 270));

  // code for pizzaBox
  // need to figure out how to append into center of arm
 // MechanismLigament2d m_pizzaBox1 = m_arm.append(new MechanismLigament2d("pizzaBox1", 0.205, 90));
  //MechanismLigament2d m_pizzaBox2 = m_arm.append(new MechanismLigament2d("pizzaBox2", 0.205, -90));

  // TO DO: use MechanismRoot2d and MechanismLigament2d to form stick figures that represent the intake
  // Example of attaching to roots/ligaments: MechanismLigament2d m_spinner = m_arm.append(new MechanismLigament2d("spinner", 0.2, 270, 12, new Color8Bit(Color.kPurple)));
  
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
    /* 
    double intakeArmAngle = intakeSubsystem.getArmAngle();
    double armAngle = armSubsystem.getArmAngle();
  
    //TO DO: Update arm angles in stick figures; 90 degrees is straight down. 180 degrees is perpendicular to floor and above intake
    m_arm.setAngle(armAngle+90);

    //TO DO: Update intake arm angles in stick figures; 0 degree is deployed n, 92 degrees is retracted position
    m_intakeArm1.setAngle(intakeArmAngle+8);
    m_intakeArm2.setAngle(-intakeArmAngle-50); 
    */
    SmartDashboard.putData("Mech2d", mech);
  }
}
  