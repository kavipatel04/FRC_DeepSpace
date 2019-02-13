/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import frc.robot.Robot;
import frc.robot.RobotMap.MapKeys;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.command.Subsystem;

/**
 * Add your docs here.
 */
public class GrabberHandSubsystem extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
   private DoubleSolenoid m_grabberHand;
   public GrabberHandSubsystem() {
   m_grabberHand = new DoubleSolenoid(Robot.m_map.getId(MapKeys.PCM_ARMCANID), Robot.m_map.getId(MapKeys.SOLENOID_PUSHERPUSH), Robot.m_map.getId(MapKeys.SOLENOID_PUSHERRETRACT));
   m_grabberHand.set(DoubleSolenoid.Value.kOff);
  }

   public void handTurnOff() {
    m_grabberHand.set(DoubleSolenoid.Value.kOff);
  }
  public void handExtend(){
    m_grabberHand.set(DoubleSolenoid.Value.kForward);
  } 
  public void handRetract() {
    m_grabberHand.set(DoubleSolenoid.Value.kReverse);
  }

  @Override
  protected void initDefaultCommand() {

  }
} 
