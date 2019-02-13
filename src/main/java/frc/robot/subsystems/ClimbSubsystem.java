/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.Robot;
import frc.robot.RobotMap.MapKeys;

import java.util.EnumMap;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;



/**
 * Add your docs here.
 */
public class ClimbSubsystem extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  enum ClimbState {
    IDLE,
    DESCEND_S1, DESCEND_S2, DESCEND_S3, DESCEND_S4,
    CLIMB_L2_S1, CLIMB_L2_S2, CLIMB_L2_S3, CLIMB_L2_S4, CLIMB_L2_S5, CLIMB_L2_S6,
    CLIMB_L3_S1, CLIMB_L3_S2, CLIMB_L3_S3, CLIMB_L3_S4,
  }
  private ClimbState m_climbState = ClimbState.IDLE;

  private static final EnumMap<ClimbState, ClimbState> nextStageMap = new EnumMap<>(ClimbState.class);
  private static final EnumMap<ClimbState, ClimbState> prevStageMap = new EnumMap<>(ClimbState.class);

  private static final double DESCEND_SLOW = 0.5;

  private static final double L2_ASCEND_SPEED = 0.5;

  private static final double L2_ASCEND_SLOW = 0.5;

  private static final double L3_ASCEND_SPEED = 0.5;

  private static final double L3_ASCEND_SLOW = 0.5;

  private boolean m_autoDescend;

  private boolean m_autoL2Ascend;

  private boolean m_autoL3Ascend;

  private boolean m_autoNextStage;
  
  private boolean m_autoPrevStage;

  private double m_timeLeft_sec;

  private DoubleSolenoid DummySolenoid;
  private DoubleSolenoid Solenoid_1;
  private DoubleSolenoid Solenoid_2;
  private DoubleSolenoid Solenoid_3;
  private DoubleSolenoid Solenoid_4;
  private DoubleSolenoid Solenoid_5;
  private DoubleSolenoid Solenoid_6;
  private DoubleSolenoid Solenoid_7;
  private DoubleSolenoid Solenoid_8; 
  WPI_TalonSRX backleftwheel;
  WPI_TalonSRX backrightwheel;
  public SpeedControllerGroup climbwheels;

  private double m_mainDrive;

  private double m_auxDrive;

  private int m_LEDRedValue;

  private int m_LEDBlueValue;

  private int m_LEDGreenValue;
  


  public ClimbSubsystem() {

    nextStageMap.put(ClimbState.IDLE, ClimbState.IDLE);

    nextStageMap.put(ClimbState.DESCEND_S1, ClimbState.DESCEND_S2);
    nextStageMap.put(ClimbState.DESCEND_S2, ClimbState.DESCEND_S3);
    nextStageMap.put(ClimbState.DESCEND_S3, ClimbState.DESCEND_S4);
    nextStageMap.put(ClimbState.DESCEND_S4, ClimbState.IDLE);

    nextStageMap.put(ClimbState.CLIMB_L2_S1, ClimbState.CLIMB_L2_S2);
    nextStageMap.put(ClimbState.CLIMB_L2_S2, ClimbState.CLIMB_L2_S3);
    nextStageMap.put(ClimbState.CLIMB_L2_S3, ClimbState.CLIMB_L2_S4);
    nextStageMap.put(ClimbState.CLIMB_L2_S4, ClimbState.CLIMB_L2_S5);
    nextStageMap.put(ClimbState.CLIMB_L2_S5, ClimbState.CLIMB_L2_S6);
    nextStageMap.put(ClimbState.CLIMB_L2_S6, ClimbState.IDLE);

    nextStageMap.put(ClimbState.CLIMB_L3_S1, ClimbState.CLIMB_L3_S2);
    nextStageMap.put(ClimbState.CLIMB_L3_S2, ClimbState.CLIMB_L3_S3);
    nextStageMap.put(ClimbState.CLIMB_L3_S3, ClimbState.CLIMB_L3_S4);
    nextStageMap.put(ClimbState.CLIMB_L3_S4, ClimbState.IDLE);

    prevStageMap.put(ClimbState.IDLE, ClimbState.IDLE);

    prevStageMap.put(ClimbState.DESCEND_S1, ClimbState.IDLE);
    prevStageMap.put(ClimbState.DESCEND_S2, ClimbState.DESCEND_S1);
    prevStageMap.put(ClimbState.DESCEND_S3, ClimbState.DESCEND_S2);
    prevStageMap.put(ClimbState.DESCEND_S4, ClimbState.DESCEND_S3);

    prevStageMap.put(ClimbState.CLIMB_L2_S1, ClimbState.IDLE);
    prevStageMap.put(ClimbState.CLIMB_L2_S2, ClimbState.CLIMB_L2_S1);
    prevStageMap.put(ClimbState.CLIMB_L2_S3, ClimbState.CLIMB_L2_S2);
    prevStageMap.put(ClimbState.CLIMB_L2_S4, ClimbState.CLIMB_L2_S3);
    prevStageMap.put(ClimbState.CLIMB_L2_S5, ClimbState.CLIMB_L2_S4);
    prevStageMap.put(ClimbState.CLIMB_L2_S6, ClimbState.CLIMB_L2_S5);

    prevStageMap.put(ClimbState.CLIMB_L3_S1, ClimbState.IDLE);
    prevStageMap.put(ClimbState.CLIMB_L3_S2, ClimbState.CLIMB_L3_S1);
    prevStageMap.put(ClimbState.CLIMB_L3_S3, ClimbState.CLIMB_L3_S2);
    prevStageMap.put(ClimbState.CLIMB_L3_S4, ClimbState.CLIMB_L3_S3);

    backleftwheel = new WPI_TalonSRX(Robot.m_map.getId(MapKeys.BACKLEFTCLIMBWHEEL));  
    backleftwheel.setInverted(true); 
    backrightwheel = new WPI_TalonSRX(Robot.m_map.getId(MapKeys.BACKRIGHTCLIMBWHEEL));
    backrightwheel.setInverted(true);
    climbwheels = new SpeedControllerGroup(backleftwheel, backrightwheel);
    final int PCM_1_CAN_ID = Robot.m_map.getId(MapKeys.PCM_CLIMBCANID);
    final int PCM_2_CAN_ID = Robot.m_map.getId(MapKeys.PCM_CLIMBCANID2);

    /*DummySolenoid = new DoubleSolenoid(PCM_1_CAN_ID, 6,7);
    DummySolenoid.set(DoubleSolenoid.Value.kOff);
    */
    Solenoid_1 = new DoubleSolenoid(
      PCM_1_CAN_ID,
      Robot.m_map.getId(MapKeys.SOLENOID_FRONTLEFTEXTEND),
      Robot.m_map.getId(MapKeys.SOLENOID_FRONTLEFTRETRACT)
    );
    Solenoid_1.set(DoubleSolenoid.Value.kOff);

    Solenoid_2 = new DoubleSolenoid(
      PCM_1_CAN_ID,
      Robot.m_map.getId(MapKeys.SOLENOID_FRONTRIGHTEXTEND),
      Robot.m_map.getId(MapKeys.SOLENOID_FRONTRIGHTRETRACT)
    );
    Solenoid_2.set(DoubleSolenoid.Value.kOff);
  

    
    Solenoid_3 = new DoubleSolenoid(
      PCM_1_CAN_ID,
      Robot.m_map.getId(MapKeys.SOLENOID_BACKLEFTEXTEND),
      Robot.m_map.getId(MapKeys.SOLENOID_BACKLEFTRETRACT)
      );
    Solenoid_3.set(DoubleSolenoid.Value.kOff);

    Solenoid_4 = new DoubleSolenoid(
      PCM_1_CAN_ID,
    Robot.m_map.getId(MapKeys.SOLENOID_BACKRIGHTEXTEND),
    Robot.m_map.getId(MapKeys.SOLENOID_BACKRIGHTRETRACT)
    );
    Solenoid_4.set(DoubleSolenoid.Value.kOff);
    
    // solenoids below do not work on albert

    Solenoid_5 = new DoubleSolenoid(
      PCM_2_CAN_ID,
      Robot.m_map.getId(MapKeys.SOLENOID_LOWERFRONTEXTEND),
      Robot.m_map.getId(MapKeys.SOLENOID_LOWERFRONTRETRACT)
    );
    Solenoid_5.set(DoubleSolenoid.Value.kOff);

    Solenoid_6 = new DoubleSolenoid(
      PCM_2_CAN_ID,
      Robot.m_map.getId(MapKeys.SOLENOID_LOWERBACKEXTEND),
      Robot.m_map.getId(MapKeys.SOLENOID_LOWERBACKRETRACT)
    );
    Solenoid_6.set(DoubleSolenoid.Value.kOff); 
    
    Solenoid_7 = new DoubleSolenoid(
      PCM_2_CAN_ID,
      Robot.m_map.getId(MapKeys.SOLENOID_ASCENDASSISTBACKLEFTEXTEND),
      Robot.m_map.getId(MapKeys.SOLENOID_ASCENDASSISTBACKLEFTRETRACT)
    );
    Solenoid_7.set(DoubleSolenoid.Value.kOff);

    Solenoid_8 = new DoubleSolenoid(
      PCM_2_CAN_ID,
      Robot.m_map.getId(MapKeys.SOLENOID_ASCENDASSISTBACKRIGHTEXTEND),
      Robot.m_map.getId(MapKeys.SOLENOID_ASCENDASSISTBACKRIGHTRETRACT)
    );
    Solenoid_8.set(DoubleSolenoid.Value.kOff);
    
    
    
    
  

    m_mainDrive = 0.0;
    
    m_auxDrive = 0.0;

    m_LEDRedValue = 0;
    m_LEDBlueValue = 0;
    m_LEDGreenValue = 0;







    }

  public void initialize() {
    setActuators();
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }

  public boolean stateDescend(){
    return !m_autoDescend;
  } 

  public void startDescend() {

    m_climbState = ClimbState.DESCEND_S1;
    setActuators();
    m_autoDescend = true;
  }

  public boolean stateL2Ascend(){
    return !m_autoL2Ascend;
  }

  public void startL2Ascend() {

    m_climbState = ClimbState.CLIMB_L2_S1;
    setActuators();
    m_autoL2Ascend = true;
  }

  public boolean stateL3Ascend(){
    return !m_autoL3Ascend;
  }

  public void startL3Ascend() {

    m_climbState = ClimbState.CLIMB_L3_S1;
    setActuators();
    m_autoL3Ascend = true;

  }

  public boolean stateNextStage(){
    return !m_autoNextStage;
  }

  public void nextStage() {

    m_climbState = nextStageMap.get(m_climbState);
    setActuators();
    m_autoNextStage = true;

  }

  public boolean statePrevStage(){
    return !m_autoPrevStage;
  }

  public void prevStage() {

    m_climbState = prevStageMap.get(m_climbState);
    setActuators();
    m_autoPrevStage = true;
  }
  private void setActuators() {

    switch(m_climbState) {

      case IDLE:
        ascendFront(false);
        ascendBack(false);
        descendAssistBack(false);
        descendAssistFront(false);
        ascendAssistBack(false);
        m_auxDrive = 0.0;
        m_mainDrive = 0.0;
        m_timeLeft_sec = 0.0; 
        m_LEDRedValue = 255;
        m_LEDBlueValue = 0;
        m_LEDGreenValue = 0;
        m_autoDescend = false;
        m_autoL2Ascend = false;
        m_autoL3Ascend = false;
        m_autoNextStage = false;
        m_autoPrevStage = false;

        System.out.print("Robot is in stage IDLE\n");
        break;

      case DESCEND_S1:
        ascendFront(false);
        ascendBack(false);
        descendAssistBack(false);
        descendAssistFront(true);
        ascendAssistBack(false);
        m_auxDrive = 0.0;
        m_mainDrive = DESCEND_SLOW;
        m_timeLeft_sec = 2.0;
        m_LEDRedValue = 0;
        m_LEDBlueValue = 255;
        m_LEDGreenValue = 0;
        System.out.print("Descend Stage 1\n");
        break;

      case DESCEND_S2:
        ascendFront(false);
        ascendBack(false);
        descendAssistBack(true);
        descendAssistFront(true);
        ascendAssistBack(false);
        m_auxDrive = 0.0;
        m_mainDrive = DESCEND_SLOW;
        m_timeLeft_sec = 2.0;
        m_LEDRedValue = 0;
        m_LEDBlueValue = 255;
        m_LEDGreenValue = 0;
        System.out.print("Descend Stage 2\n");
        break;

      case DESCEND_S3:
        ascendFront(false);
        ascendBack(false);
        descendAssistBack(true);
        descendAssistFront(false);
        ascendAssistBack(false);
        m_auxDrive = 0.0;
        m_mainDrive = DESCEND_SLOW;
        m_timeLeft_sec = 2.0;
        m_LEDRedValue = 0;
        m_LEDBlueValue = 255;
        m_LEDGreenValue = 0;
        System.out.print("Descend Stage 3\n");
        break;

      case DESCEND_S4:
        ascendFront(false);
        ascendBack(false);
        descendAssistBack(false);
        descendAssistFront(false);
        ascendAssistBack(false);
        m_auxDrive = 0.0;
        m_mainDrive = DESCEND_SLOW;
        m_timeLeft_sec = 2.0;
        m_LEDRedValue = 0;
        m_LEDBlueValue = 255;
        m_LEDGreenValue = 0;
        System.out.print("Descend Stage 4\n");
        break;

      case CLIMB_L2_S1:
        ascendFront(true);
        ascendBack(true);
        descendAssistBack(false);
        descendAssistFront(false);
        ascendAssistBack(false);
        m_auxDrive = 0.0;
        m_mainDrive = 0.0;
        m_timeLeft_sec = 2.0;
        m_LEDRedValue = 0;
        m_LEDBlueValue = 0;
        m_LEDGreenValue = 0;
        System.out.print("Climb Level 2 Stage 1\n");
        break;

      case CLIMB_L2_S2:
        ascendFront(true);
        ascendBack(true);
        descendAssistBack(false);
        descendAssistFront(false);
        ascendAssistBack(false);
        m_auxDrive = L2_ASCEND_SPEED;
        m_mainDrive = L2_ASCEND_SLOW;
        m_timeLeft_sec = 2.0;
        m_LEDRedValue = 0;
        m_LEDBlueValue = 0;
        m_LEDGreenValue = 0;
        System.out.print("Climb Level 2 Stage 2\n");
        break;

      case CLIMB_L2_S3:
        ascendFront(true);
        ascendBack(true);
        descendAssistBack(false);
        descendAssistFront(false);
        ascendAssistBack(true);
        m_auxDrive = L2_ASCEND_SPEED;
        m_mainDrive = L2_ASCEND_SLOW;
        m_timeLeft_sec = 2.0;
        m_LEDRedValue = 0;
        m_LEDBlueValue = 0;
        m_LEDGreenValue = 0;
        System.out.print("Climb Level 2 Stage 3\n");
        break;

      case CLIMB_L2_S4:
        ascendFront(false);
        ascendBack(false);
        descendAssistBack(false);
        descendAssistFront(false);
        ascendAssistBack(true);
        m_auxDrive = 0.0;
        m_mainDrive = L2_ASCEND_SLOW;
        m_timeLeft_sec = 2.0;
        m_LEDRedValue = 0;
        m_LEDBlueValue = 0;
        m_LEDGreenValue = 0;
        System.out.print("Climb Level 2 Stage 4\n");
        break;

      case CLIMB_L2_S5:
        ascendFront(false);
        ascendBack(false);
        descendAssistBack(false);
        descendAssistFront(false);
        ascendAssistBack(false);
        m_auxDrive = 0.0;
        m_mainDrive = L2_ASCEND_SLOW;
        m_timeLeft_sec =2.0;
        m_LEDRedValue = 0;
        m_LEDBlueValue = 0;
        m_LEDGreenValue = 0;
        System.out.print("Climb Level 2 Stage 5\n");
        break;

      case CLIMB_L2_S6:
        ascendFront(false);
        ascendBack(false);
        descendAssistBack(false);
        descendAssistFront(false);
        ascendAssistBack(false);
        m_auxDrive = 0.0;
        m_mainDrive = 0.0;
        m_timeLeft_sec = 2.0;
        m_LEDRedValue = 0;
        m_LEDBlueValue = 0;
        m_LEDGreenValue = 0;
        System.out.print("Climb Level 2 Stage 6\n");
        break;

      case CLIMB_L3_S1:
        ascendFront(true);
        ascendBack(true);
        descendAssistBack(false);
        descendAssistFront(false);
        ascendAssistBack(false);
        m_auxDrive = 0.0;
        m_mainDrive = 0.0;
        m_timeLeft_sec = 2.0;
        m_LEDRedValue = 0;
        m_LEDBlueValue = 0;
        m_LEDGreenValue = 0;
        System.out.print("Climb Level 3 Stage 1\n");
        break;

      case CLIMB_L3_S2:
        ascendFront(true);
        ascendBack(true);
        descendAssistBack(false);
        descendAssistFront(false);
        ascendAssistBack(false);
        m_auxDrive = L3_ASCEND_SPEED;
        m_mainDrive = L3_ASCEND_SLOW;
        m_timeLeft_sec = 2.0;
        m_LEDRedValue = 0;
        m_LEDBlueValue = 0;
        m_LEDGreenValue = 0;
        System.out.print("Climb Level 3 Stage 2\n");
        break;

      case CLIMB_L3_S3:
        ascendFront(false);
        ascendBack(true);
        descendAssistBack(false);
        descendAssistFront(false);
        ascendAssistBack(false);
        m_auxDrive = L3_ASCEND_SPEED;
        m_mainDrive = L3_ASCEND_SLOW;
        m_timeLeft_sec = 2.0;
        m_LEDRedValue = 0;
        m_LEDBlueValue = 0;
        m_LEDGreenValue = 0;
        System.out.print("Climb Level 3 Stage 3\n");
        break;

      case CLIMB_L3_S4:
        ascendFront(false);
        ascendBack(false);
        descendAssistBack(false);
        descendAssistFront(false);
        ascendAssistBack(false);
        m_auxDrive = 0.0;
        m_mainDrive = L3_ASCEND_SLOW;
        m_timeLeft_sec = 2.0;
        m_LEDRedValue = 0;
        m_LEDBlueValue = 0;
        m_LEDGreenValue = 0;
        System.out.print("Climb Level 3 Stage 4\n");
        break;

    }

  }

  public double getMainDrive() {

    return m_mainDrive;

  }
  public int getRedValue(){
    return m_LEDRedValue;
  }

  public int getBlueValue(){
    return m_LEDBlueValue;
  }

  public int getGreenValue(){
    return m_LEDGreenValue;
  }

  public double getTimeLeft() {

    return m_timeLeft_sec;

  }

  public double getauxDrive() {
    return m_auxDrive;
  }

  public void setauxDrive() {
    climbwheels.set(m_auxDrive);
  }
  




  public void ascendFront(boolean state) {
    if (state) {
      System.out.print("Acscend Front Activated\n");
    Solenoid_1.set(DoubleSolenoid.Value.kForward);
    Solenoid_2.set(DoubleSolenoid.Value.kForward);
  }
  else {
    Solenoid_1.set(DoubleSolenoid.Value.kReverse);
    Solenoid_2.set(DoubleSolenoid.Value.kReverse);
  }
}

public void ascendBack(boolean state) {
  if (state) {
    System.out.print("Acscend Back Activated\n");
    Solenoid_3.set(DoubleSolenoid.Value.kForward);
    Solenoid_4.set(DoubleSolenoid.Value.kForward);
  }
  else{
    Solenoid_3.set(DoubleSolenoid.Value.kReverse);
    Solenoid_4.set(DoubleSolenoid.Value.kReverse);
  }
}

public void descendAssistFront(boolean state) {
  if (state) {
    System.out.print("Descend Front Activated\n");
    Solenoid_5.set(DoubleSolenoid.Value.kForward);
  }
  else {
    Solenoid_5.set(DoubleSolenoid.Value.kReverse);
  }

}

public void descendAssistBack(boolean state) {
  if (state) {
    System.out.print("Descend Back Activated\n");
    Solenoid_6.set(DoubleSolenoid.Value.kForward);
  }
  else {
    Solenoid_6.set(DoubleSolenoid.Value.kReverse);
  }
}

public void ascendAssistBack(boolean state) {
  if (state) {
    System.out.print("Acscend Assist Back Activated\n");
    Solenoid_7.set(DoubleSolenoid.Value.kForward);
    Solenoid_8.set(DoubleSolenoid.Value.kForward);
  }
  else {
    Solenoid_7.set(DoubleSolenoid.Value.kReverse);
    Solenoid_8.set(DoubleSolenoid.Value.kReverse);
  }
}
 
}

