/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;


/**
 * Add your docs here.
 */
public class PneumaticTestSubsystem extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  //private DoubleSolenoid m_FrontrightLargePiston;
  private DoubleSolenoid m_FrontleftLargePiston;
  private DoubleSolenoid m_FrontrightLargePiston;
  
  /*
  private DoubleSolenoid m_BackrightLargePiston;
  private DoubleSolenoid m_BackleftLargePiston;


  private DoubleSolenoid m_FrontleftSmallPiston;
  private DoubleSolenoid m_FrontrightSmallPiston;
  private DoubleSolenoid m_BackrightSmallPiston;
  private DoubleSolenoid m_BackleftSmallPiston;


  private TalonSRX m_MiniDrive;

*/
  public PneumaticTestSubsystem() {

    m_FrontleftLargePiston = new DoubleSolenoid(RobotMap.PcmCanID,RobotMap.SolenoidPort1,RobotMap.SolenoidPort2);
    m_FrontrightLargePiston = new DoubleSolenoid(RobotMap.PcmCanID,RobotMap.SolenoidPort3,RobotMap.SolenoidPort4);
    
    /*
    m_BackleftLargePiston = new DoubleSolenoid(1,1,1);
    m_BackrightLargePiston = new DoubleSolenoid(1,1,1);
    

    m_FrontleftSmallPiston = new DoubleSolenoid(1,1,1);
    m_FrontrightSmallPiston = new DoubleSolenoid(1,1,1);
    m_BackleftSmallPiston = new DoubleSolenoid(1,1,1);
    m_BackrightSmallPiston = new DoubleSolenoid(1,1,1);
   
    m_MiniDrive = new TalonSRX(1);
    */
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
    
  }

  /*
  public void ExtendAllLarge() {

    m_FrontleftLargePiston.set(DoubleSolenoid.Value.kForward);
    m_FrontrightLargePiston.set(DoubleSolenoid.Value.kForward);
    m_BackleftLargePiston.set(DoubleSolenoid.Value.kForward);
    m_BackrightLargePiston.set(DoubleSolenoid.Value.kForward);



  }

  public void RetractAllLarge() {

    m_FrontleftLargePiston.set(DoubleSolenoid.Value.kReverse);
    m_FrontrightLargePiston.set(DoubleSolenoid.Value.kReverse);
    m_BackleftLargePiston.set(DoubleSolenoid.Value.kReverse);
    m_BackrightLargePiston.set(DoubleSolenoid.Value.kReverse);

  }

  */

  public void ExtendFrontLarge() {

    m_FrontleftLargePiston.set(DoubleSolenoid.Value.kForward);
    m_FrontrightLargePiston.set(DoubleSolenoid.Value.kForward);

  }

  public void RetractFrontLarge() {

    m_FrontleftLargePiston.set(DoubleSolenoid.Value.kReverse);
    m_FrontrightLargePiston.set(DoubleSolenoid.Value.kReverse);


  }

  /*

  public void ExtendBackLarge() {

    m_BackleftLargePiston.set(DoubleSolenoid.Value.kForward);
    m_BackleftLargePiston.set(DoubleSolenoid.Value.kForward);


  }

  public void RetractBackLarge() {

    m_BackleftLargePiston.set(DoubleSolenoid.Value.kReverse);
    m_BackrightLargePiston.set(DoubleSolenoid.Value.kReverse);


  }

  public void ExtendFrontSmall() {

    m_FrontleftSmallPiston.set(DoubleSolenoid.Value.kForward);
    m_FrontrightSmallPiston.set(DoubleSolenoid.Value.kForward);


  }

  public void RetractFrontSmall() {

    m_FrontleftSmallPiston.set(DoubleSolenoid.Value.kReverse);
    m_FrontrightSmallPiston.set(DoubleSolenoid.Value.kReverse);


  }

  public void ExtendBackSmall() {

    m_BackleftSmallPiston.set(DoubleSolenoid.Value.kForward);
    m_BackrightSmallPiston.set(DoubleSolenoid.Value.kForward);


  }

  public void RetractBackSmall() {

    m_BackleftSmallPiston.set(DoubleSolenoid.Value.kReverse);
    m_BackrightSmallPiston.set(DoubleSolenoid.Value.kReverse);


  }

  */
}
