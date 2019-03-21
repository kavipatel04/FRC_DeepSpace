/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.Robot;
import frc.robot.RobotMap.MapKeys;

import java.util.EnumMap;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.kauailabs.navx.frc.AHRS;



//Add your docs here.
 
public class ClimbSubsystem extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  enum ClimbState {
    IDLE,
    DESCEND_S1, DESCEND_S2, DESCEND_S3, DESCEND_S4, DESCEND_S5,
    CLIMB_L2_S1, CLIMB_L2_S2, CLIMB_L2_S3, CLIMB_L2_S4, CLIMB_L2_S5, CLIMB_L2_S6,
    CLIMB_L3_S1A, CLIMB_L3_S1B,CLIMB_L3_S2, CLIMB_L3_S3, CLIMB_L3_S4, CLIMB_L3_S5,
  }
  private ClimbState m_climbState = ClimbState.IDLE;

  private static final EnumMap<ClimbState, ClimbState> nextStageMap = new EnumMap<>(ClimbState.class);
  private static final EnumMap<ClimbState, ClimbState> prevStageMap = new EnumMap<>(ClimbState.class);

  private double m_timeLeft_sec;

  
  private DoubleSolenoid Solenoid_1;
  private DoubleSolenoid Solenoid_2;
  private DoubleSolenoid Solenoid_3;
  private DoubleSolenoid Solenoid_4;
  private DoubleSolenoid Solenoid_5;
  private DoubleSolenoid Solenoid_6;
  private DoubleSolenoid Solenoid_7;
  private DoubleSolenoid Solenoid_8;

  WPI_TalonSRX frontleftwheel;
  WPI_TalonSRX frontrightwheel;
  public SpeedControllerGroup climbwheels;

  public double m_mainDrive;

  public double m_auxDrive;

  public int m_LEDRedValue;

  public int m_LEDBlueValue;

  public int m_LEDGreenValue;
  
  public boolean m_autoDescend;
  public boolean m_autoL2Ascend;
  public boolean m_autoL3Ascend;

  private boolean m_configured = false;

  private double m_velocity;

  private double m_DriveTime;

  private int m_tickcounter;

  private float m_roll;
  private float m_pitch;
  private boolean m_rp_initialized = false;

  private float m_pitch0;
  private float m_roll0;

   
  


  public ClimbSubsystem() {
  
    nextStageMap.put(ClimbState.IDLE, ClimbState.IDLE);

    nextStageMap.put(ClimbState.DESCEND_S1, ClimbState.DESCEND_S2);
    nextStageMap.put(ClimbState.DESCEND_S2, ClimbState.DESCEND_S3);
    nextStageMap.put(ClimbState.DESCEND_S3, ClimbState.DESCEND_S4);
    nextStageMap.put(ClimbState.DESCEND_S4, ClimbState.DESCEND_S5);
    nextStageMap.put(ClimbState.DESCEND_S5, ClimbState.IDLE);

    nextStageMap.put(ClimbState.CLIMB_L2_S1, ClimbState.CLIMB_L2_S2);
    nextStageMap.put(ClimbState.CLIMB_L2_S2, ClimbState.CLIMB_L2_S3);
    nextStageMap.put(ClimbState.CLIMB_L2_S3, ClimbState.CLIMB_L2_S4);
    nextStageMap.put(ClimbState.CLIMB_L2_S4, ClimbState.CLIMB_L2_S5);
    nextStageMap.put(ClimbState.CLIMB_L2_S5, ClimbState.CLIMB_L2_S6);
    nextStageMap.put(ClimbState.CLIMB_L2_S6, ClimbState.IDLE);

    nextStageMap.put(ClimbState.CLIMB_L3_S1A, ClimbState.CLIMB_L3_S1B);
    nextStageMap.put(ClimbState.CLIMB_L3_S1B, ClimbState.CLIMB_L3_S2);
    nextStageMap.put(ClimbState.CLIMB_L3_S2, ClimbState.CLIMB_L3_S3);
    nextStageMap.put(ClimbState.CLIMB_L3_S3, ClimbState.CLIMB_L3_S4);
    nextStageMap.put(ClimbState.CLIMB_L3_S4, ClimbState.CLIMB_L3_S5);
    nextStageMap.put(ClimbState.CLIMB_L3_S5, ClimbState.IDLE);

    prevStageMap.put(ClimbState.IDLE, ClimbState.IDLE);

    prevStageMap.put(ClimbState.DESCEND_S1, ClimbState.IDLE);
    prevStageMap.put(ClimbState.DESCEND_S2, ClimbState.DESCEND_S1);
    prevStageMap.put(ClimbState.DESCEND_S3, ClimbState.DESCEND_S2);
    prevStageMap.put(ClimbState.DESCEND_S4, ClimbState.DESCEND_S3);
    prevStageMap.put(ClimbState.DESCEND_S5, ClimbState.DESCEND_S4);

    prevStageMap.put(ClimbState.CLIMB_L2_S1, ClimbState.IDLE);
    prevStageMap.put(ClimbState.CLIMB_L2_S2, ClimbState.CLIMB_L2_S1);
    prevStageMap.put(ClimbState.CLIMB_L2_S3, ClimbState.CLIMB_L2_S2);
    prevStageMap.put(ClimbState.CLIMB_L2_S4, ClimbState.CLIMB_L2_S3);
    prevStageMap.put(ClimbState.CLIMB_L2_S5, ClimbState.CLIMB_L2_S4);
    prevStageMap.put(ClimbState.CLIMB_L2_S6, ClimbState.CLIMB_L2_S5);

    prevStageMap.put(ClimbState.CLIMB_L3_S1A, ClimbState.IDLE);
    prevStageMap.put(ClimbState.CLIMB_L3_S1B, ClimbState.CLIMB_L3_S1A);
    prevStageMap.put(ClimbState.CLIMB_L3_S2, ClimbState.CLIMB_L3_S1B);
    prevStageMap.put(ClimbState.CLIMB_L3_S3, ClimbState.CLIMB_L3_S2);
    prevStageMap.put(ClimbState.CLIMB_L3_S4, ClimbState.CLIMB_L3_S3);
    prevStageMap.put(ClimbState.CLIMB_L3_S5, ClimbState.CLIMB_L3_S4);

    m_mainDrive = 0.0;
    
    m_auxDrive = 0.0;

    m_LEDRedValue = 0;
    m_LEDBlueValue = 0;
    m_LEDGreenValue = 0;

    
    }

  public void initialize() {

    //TODO-CALCULATE VELOCITY
    m_velocity = 28.24; //Inches per Second
    readNavX();
    initActuators();
    setActuators();
  }

  public void zeroNavX(){
    m_pitch0 = m_pitch;
    m_roll0 = m_roll;
  }

  private void readNavX() {
    m_pitch = -Robot.m_ahrs.getPitch();
    m_roll = -Robot.m_ahrs.getRoll();
    if (!m_rp_initialized){
      m_rp_initialized = true;
      zeroNavX();
    }
    m_pitch -= m_pitch0;
    m_roll -= m_roll0;
    //System.out.printf("Roll,Pitch = %f, %f\n", m_roll, m_pitch);
  }

  public void initActuators() {
    int frontLeftClimbCanID = Robot.m_map.getId(MapKeys.FRONTLEFTCLIMBWHEEL);
    int frontRightClimbCanID = Robot.m_map.getId(MapKeys.FRONTRIGHTCLIMBWHEEL);
    if ((frontLeftClimbCanID != 0) && (frontRightClimbCanID != 0)){
      frontleftwheel = new WPI_TalonSRX(frontLeftClimbCanID);  
      frontleftwheel.setInverted(false); 
      frontrightwheel = new WPI_TalonSRX(frontRightClimbCanID);
      frontrightwheel.setInverted(false);
      climbwheels = new SpeedControllerGroup(frontleftwheel, frontrightwheel);
    }


    final int PCM_1_CAN_ID = Robot.m_map.getId(MapKeys.PCM_CLIMBCANID);
    final int PCM_2_CAN_ID = Robot.m_map.getId(MapKeys.PCM_CLIMBCANID2);
    if ((PCM_1_CAN_ID != 0) && (PCM_2_CAN_ID != 0)){
      Solenoid_1 = new DoubleSolenoid(
        PCM_1_CAN_ID,
        Robot.m_map.getId(MapKeys.SOLENOID_FRONTEXTEND),
        Robot.m_map.getId(MapKeys.SOLENOID_FRONTRETRACT)
      );
      Solenoid_1.set(DoubleSolenoid.Value.kOff);

      Solenoid_2 = new DoubleSolenoid(
        PCM_1_CAN_ID,
        Robot.m_map.getId(MapKeys.SOLENOID_BACKEXTEND),
        Robot.m_map.getId(MapKeys.SOLENOID_BACKRETRACT)
      );
      Solenoid_2.set(DoubleSolenoid.Value.kOff);
    

      
    /*  Solenoid_3 = new DoubleSolenoid(
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
      */
    

      Solenoid_5 = new DoubleSolenoid(
        PCM_1_CAN_ID,
        Robot.m_map.getId(MapKeys.SOLENOID_LOWERFRONTEXTEND),
        Robot.m_map.getId(MapKeys.SOLENOID_LOWERFRONTRETRACT)
      );
      Solenoid_5.set(DoubleSolenoid.Value.kOff);

      Solenoid_6 = new DoubleSolenoid(
        PCM_1_CAN_ID,
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
    }
    if ((frontLeftClimbCanID != 0) && (frontRightClimbCanID != 0) && 
        (PCM_1_CAN_ID != 0) && (PCM_2_CAN_ID != 0)){
      m_configured = true;
    }
  }
  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }

  public double getDescecndTimeleft(double m_Distance){

    m_DriveTime = (m_Distance)/(m_velocity);
    
    return m_DriveTime;
  }

  public boolean stateDescend(){
    return !m_autoDescend;
  } 
  public boolean stateL2Ascend(){
    return !m_autoL2Ascend;
  }
  public boolean stateL3Ascend(){
    return !m_autoL3Ascend;
  }
  public void startDescend() {

    m_climbState = ClimbState.DESCEND_S1;
    setActuators();
  }

  public void startL2Ascend() {

    m_climbState = ClimbState.CLIMB_L2_S1;
    setActuators();
  }

  public void startL3Ascend() {

    m_climbState = ClimbState.CLIMB_L3_S1A;
    setActuators();

  }

  public void nextStage() {

    m_climbState = nextStageMap.get(m_climbState);
    if (m_climbState == ClimbState.CLIMB_L3_S1B) {
      m_tickcounter = 0;
    }
    setActuators();

  }

  public void prevStage() {

    m_climbState = prevStageMap.get(m_climbState);
    if (m_climbState == ClimbState.CLIMB_L3_S1B) {
      m_tickcounter = 0;
    }
    setActuators();
  }

  @Override 
  public void periodic() {
    readNavX();

    if (m_climbState == ClimbState.CLIMB_L3_S1B) {
      Preferences prefs = Preferences.getInstance();
      
      double pitchThresh = prefs.getDouble("Climb_L3_S1B_Pitch", 5.0);

      boolean frontExtend = true;
      boolean rearExtend = true;

      if (m_pitch > pitchThresh){
        //Pitched too far forward
        rearExtend = false; //Pitched too far forward
        System.out.println("Stopped Rear");
      } 
      if (m_pitch < -pitchThresh){
        //Pitched too far backward
        frontExtend = false;
        System.out.println("Stopped Front");
      } 

      ascendBack(rearExtend);
      ascendFront(frontExtend);
      /* int onTime = prefs.getInt("Climb_L3_S1B_onTime", 10);
      int offTime = prefs.getInt("Climb_L3_S1B_offTime", 0);

      if ((m_tickcounter % (onTime + offTime)) < offTime) {
        //Drive Front Pistons Down for Off Time
        ascendFront(false);
      }
      else{
        //Drive Front Pistons Up for On Time
        ascendFront(true);
      }

      m_tickcounter += 1; */

    }
    
    
  }

  private void setActuators() {
    Preferences prefs = Preferences.getInstance();
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
        System.out.print("Robot is in stage IDLE\n");
        m_autoDescend = false;
        m_autoL2Ascend = false;
        m_autoL3Ascend = false;
        break;

        case DESCEND_S1:
        ascendFront(false);
        ascendBack(false);
        descendAssistBack(false);
        descendAssistFront(true);
        ascendAssistBack(false);
        m_auxDrive = prefs.getDouble("Descend_S1_AuxDrive", 0.0);
        m_mainDrive = prefs.getDouble("Descend_S1_MainDrive", 0.4);
        m_timeLeft_sec = getDescecndTimeleft(38.0);
        //m_timeLeft_sec = prefs.getDouble("Descend_S1_TimeLeft", 0.5);
        m_LEDRedValue = 0;
        m_LEDBlueValue = 255;
        m_LEDGreenValue = 0;
        System.out.print("Descend Stage 1\n");
        m_autoDescend = true;
        m_autoL2Ascend = false;
        m_autoL3Ascend = false;
        break;

      case DESCEND_S2:
        ascendFront(false);
        ascendBack(false);
        descendAssistBack(false);
        descendAssistFront(false);
        ascendAssistBack(false);
        m_auxDrive = prefs.getDouble("Descend_S2_AuxDrive", 0.0);
        m_mainDrive = prefs.getDouble("Descend_S2_MainDrive", 0.4);
        m_timeLeft_sec = getDescecndTimeleft(8.0);
        //m_timeLeft_sec = prefs.getDouble("Descend_S2_TimeLeft", 2);
        m_LEDRedValue = 0;
        m_LEDBlueValue = 255;
        m_LEDGreenValue = 0;
        System.out.print("Descend Stage 2\n");
        m_autoDescend = true;
        m_autoL2Ascend = false;
        m_autoL3Ascend = false;
        break;

      case DESCEND_S3:
        ascendFront(false);
        ascendBack(false);
        descendAssistBack(false);
        descendAssistFront(false);
        ascendAssistBack(true);
        m_auxDrive = prefs.getDouble("Descend_S3_AuxDrive", 0.0);
        m_mainDrive = prefs.getDouble("Descend_S3_MainDrive", 0.4);
        m_timeLeft_sec = getDescecndTimeleft(8.0);
        //m_timeLeft_sec = prefs.getDouble("Descend_S3_TimeLeft", 0.75);
        m_LEDRedValue = 0;
        m_LEDBlueValue = 255;
        m_LEDGreenValue = 0;
        System.out.print("Descend Stage 3\n");
        m_autoDescend = true;
        m_autoL2Ascend = false;
        m_autoL3Ascend = false;
        break;

      case DESCEND_S4:
        ascendFront(false);
        ascendBack(false);
        descendAssistBack(false);
        descendAssistFront(false);
        ascendAssistBack(false);
        m_auxDrive = prefs.getDouble("Descend_S4_AuxDrive", 0.0);
        m_mainDrive = prefs.getDouble("Descend_S4_MainDrive", 0.4);
        m_timeLeft_sec = getDescecndTimeleft(0.0);
        //m_timeLeft_sec = prefs.getDouble("Descend_S4_TimeLeft", 0.75);
        m_LEDRedValue = 0;
        m_LEDBlueValue = 255;
        m_LEDGreenValue = 0;
        System.out.print("Descend Stage 4\n");
        m_autoDescend = true;
        m_autoL2Ascend = false;
        m_autoL3Ascend = false;
        break;

      case DESCEND_S5:
        ascendFront(false);
        ascendBack(false);
        descendAssistBack(false);
        descendAssistFront(false);
        ascendAssistBack(false);
        m_auxDrive = prefs.getDouble("Descend_S5_AuxDrive", 0.0);
        m_mainDrive = prefs.getDouble("Descend_S5_MainDrive", 0.0);
        m_timeLeft_sec = getDescecndTimeleft(0.0);
        //m_timeLeft_sec = prefs.getDouble("Descend_S5_TimeLeft", 0.0);
        m_LEDRedValue = 0;
        m_LEDBlueValue = 255;
        m_LEDGreenValue = 0;
        System.out.print("Descend Stage 5\n");
        m_autoDescend = true;
        m_autoL2Ascend = false;
        m_autoL3Ascend = false;
        break;

      case CLIMB_L2_S1:
        ascendFront(true);
        ascendBack(false);
        descendAssistBack(false);
        descendAssistFront(false);
        ascendAssistBack(false);
        m_auxDrive = prefs.getDouble("Climb_L2_S1_AuxDrive", 0.0);
        m_mainDrive = prefs.getDouble("Climb_L2_S1_MainDrive", 0.0);
        m_timeLeft_sec = prefs.getDouble("Climb_L2_S1_TimeLeft", 1.25);
        m_LEDRedValue = 0;
        m_LEDBlueValue = 0;
        m_LEDGreenValue = 0;
        System.out.print("Climb Level 2 Stage 1\n");
        m_autoDescend = false;
        m_autoL2Ascend = true;
        m_autoL3Ascend = false;
        break;

      case CLIMB_L2_S2:
        ascendFront(true);
        ascendBack(false);
        descendAssistBack(false);
        descendAssistFront(false);
        ascendAssistBack(false);
        m_auxDrive = prefs.getDouble("Climb_L2_S2_AuxDrive", 0.5);
        m_mainDrive = prefs.getDouble("Climb_L2_S2_MainDrive", 0.5);
        m_timeLeft_sec = prefs.getDouble("Climb_L2_S2_TimeLeft", 1.0);
        m_LEDRedValue = 0;
        m_LEDBlueValue = 0;
        m_LEDGreenValue = 0;
        System.out.print("Climb Level 2 Stage 2\n");
        m_autoDescend = false;
        m_autoL2Ascend = true;
        m_autoL3Ascend = false;
        break;

      case CLIMB_L2_S3:
        ascendFront(false);
        ascendBack(false);
        descendAssistBack(false);
        descendAssistFront(false);
        ascendAssistBack(false);
        m_auxDrive = prefs.getDouble("Climb_L2_S3_AuxDrive", 0.0);
        m_mainDrive = prefs.getDouble("Climb_L2_S3_MainDrive", 0.0);
        m_timeLeft_sec = prefs.getDouble("Climb_L2_S3_TimeLeft", 2.0);
        m_LEDRedValue = 0;
        m_LEDBlueValue = 0;
        m_LEDGreenValue = 0;
        System.out.print("Climb Level 2 Stage 3\n");
        m_autoDescend = false;
        m_autoL2Ascend = true;
        m_autoL3Ascend = false;
        break;

      case CLIMB_L2_S4:
        ascendFront(false);
        ascendBack(true);
        descendAssistBack(false);
        descendAssistFront(false);
        ascendAssistBack(true);
        m_auxDrive = prefs.getDouble("Climb_L2_S4_AuxDrive", 0.0);
        m_mainDrive = prefs.getDouble("Climb_L2_S4_MainDrive", 0.0);
        m_timeLeft_sec = prefs.getDouble("Climb_L2_S4_TimeLeft", 2.0);
        m_LEDRedValue = 0;
        m_LEDBlueValue = 0;
        m_LEDGreenValue = 0;
        System.out.print("Climb Level 2 Stage 4\n");
        m_autoDescend = false;
        m_autoL2Ascend = true;
        m_autoL3Ascend = false;
        break;

      case CLIMB_L2_S5:
        ascendFront(false);
        ascendBack(false);
        descendAssistBack(false);
        descendAssistFront(false);
        ascendAssistBack(true);
        m_auxDrive = prefs.getDouble("Climb_L2_S5_AuxDrive", 0.0);
        m_mainDrive = prefs.getDouble("Climb_L2_S5_MainDrive", 0.3);
        m_timeLeft_sec = prefs.getDouble("Climb_L2_S5_TimeLeft", 2.0);
        m_LEDRedValue = 0;
        m_LEDBlueValue = 0;
        m_LEDGreenValue = 0;
        System.out.print("Climb Level 2 Stage 5\n");
        m_autoDescend = false;
        m_autoL2Ascend = true;
        m_autoL3Ascend = false;
        break;

      case CLIMB_L2_S6:
        ascendFront(false);
        ascendBack(false);
        descendAssistBack(false);
        descendAssistFront(false);
        ascendAssistBack(false);
        m_auxDrive = prefs.getDouble("Climb_L2_S6_AuxDrive", 0.0);
        m_mainDrive = prefs.getDouble("Climb_L2_S6_MainDrive", 0.5);
        m_timeLeft_sec = prefs.getDouble("Climb_L2_S6_TimeLeft", 4.0);
        m_LEDRedValue = 0;
        m_LEDBlueValue = 0;
        m_LEDGreenValue = 0;
        System.out.print("Climb Level 2 Stage 6\n");
        m_autoDescend = false;
        m_autoL2Ascend = true;
        m_autoL3Ascend = false;
        break;

      case CLIMB_L3_S1A:
        ascendFront(true); 
        ascendBack(true);
        descendAssistBack(false);
        descendAssistFront(false);
        ascendAssistBack(false);
        m_auxDrive = prefs.getDouble("Climb_L3_S1_AuxDrive", 0.15);
        m_mainDrive = prefs.getDouble("Climb_L3_S1_MainDrive", 0.0);
        m_timeLeft_sec = prefs.getDouble("Climb_L3_S1_TimeLeft", 2.0);
        m_LEDRedValue = 0;
        m_LEDBlueValue = 0;
        m_LEDGreenValue = 0;
        System.out.print("Climb Level 3 Stage 1A\n");
        m_autoDescend = false;
        m_autoL2Ascend = false;
        m_autoL3Ascend = true;
        break;

        case CLIMB_L3_S1B:
        ascendFront(true); 
        ascendBack(true);
        descendAssistBack(false);
        descendAssistFront(false);
        ascendAssistBack(false);
        m_auxDrive = prefs.getDouble("Climb_L3_S1_AuxDrive", 0.15);
        m_mainDrive = prefs.getDouble("Climb_L3_S1_MainDrive", 0.0);
        m_timeLeft_sec = prefs.getDouble("Climb_L3_S1B_TimeLeft", 0.0);
        m_LEDRedValue = 0;
        m_LEDBlueValue = 0;
        m_LEDGreenValue = 0;
        System.out.print("Climb Level 3 Stage 1B\n");
        m_autoDescend = false;
        m_autoL2Ascend = false;
        m_autoL3Ascend = true;
        break;

      case CLIMB_L3_S2:
        ascendFront(true);
        ascendBack(true);
        descendAssistBack(false);
        descendAssistFront(false);
        ascendAssistBack(false);
        m_auxDrive = prefs.getDouble("Climb_L3_S2_AuxDrive", 0.5);
        m_mainDrive = prefs.getDouble("Climb_L3_S2_MainDrive", 0.5);
        m_timeLeft_sec = prefs.getDouble("Climb_L3_S2_TimeLeft", 2.0);
        m_LEDRedValue = 0;
        m_LEDBlueValue = 0;
        m_LEDGreenValue = 0;
        System.out.print("Climb Level 3 Stage 2\n");
        m_autoDescend = false;
        m_autoL2Ascend = false;
        m_autoL3Ascend = true;
        break;

      case CLIMB_L3_S3:
        ascendFront(false);
        ascendBack(true);
        descendAssistBack(false);
        descendAssistFront(false);
        ascendAssistBack(false);
        m_auxDrive = prefs.getDouble("Climb_L3_S3_AuxDrive", 0.5);
        m_mainDrive = prefs.getDouble("Climb_L3_S3_MainDrive", 0.5);
        m_timeLeft_sec = prefs.getDouble("Climb_L3_S3_TimeLeft", 2.0);
        m_LEDRedValue = 0;
        m_LEDBlueValue = 0;
        m_LEDGreenValue = 0;
        System.out.print("Climb Level 3 Stage 3\n");
        m_autoDescend = false;
        m_autoL2Ascend = false;
        m_autoL3Ascend = true;
        break;

      case CLIMB_L3_S4:
        ascendFront(false);
        ascendBack(true);
        descendAssistBack(false);
        descendAssistFront(false);
        ascendAssistBack(false);
        m_auxDrive = prefs.getDouble("Climb_L3_S4_AuxDrive", 0.0);
        m_mainDrive = prefs.getDouble("Climb_L3_S4_MainDrive", 0.5);
        m_timeLeft_sec = prefs.getDouble("Climb_L3_S4_TimeLeft", 2.0);
        m_LEDRedValue = 0;
        m_LEDBlueValue = 0;
        m_LEDGreenValue = 0;
        System.out.print("Climb Level 3 Stage 4\n");
        m_autoDescend = false;
        m_autoL2Ascend = false;
        m_autoL3Ascend = true;
        break;

        case CLIMB_L3_S5:
        ascendFront(false);
        ascendBack(false);
        descendAssistBack(false);
        descendAssistFront(false);
        ascendAssistBack(false);
        m_auxDrive = prefs.getDouble("Climb_L3_S5_AuxDrive", 0.0);
        m_mainDrive = prefs.getDouble("Climb_L3_S5_MainDrive", 0.5);
        m_timeLeft_sec = prefs.getDouble("Climb_L3_S5_TimeLeft", 2.0);
        m_LEDRedValue = 0;
        m_LEDBlueValue = 0;
        m_LEDGreenValue = 0;
        System.out.print("Climb Level 3 Stage 5\n");
        m_autoDescend = false;
        m_autoL2Ascend = false;
        m_autoL3Ascend = true;
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
    if (!m_configured) {
      return;
    }
    climbwheels.set(m_auxDrive);
  }
  




  public void ascendFront(boolean state) {
    if (!m_configured) {
      return;
    }
    if (state) {
      System.out.print("Acscend Front Activated\n");
    Solenoid_1.set(DoubleSolenoid.Value.kForward);
  
  }
  else {
    System.out.print("Acscend Front Retracted\n");
    Solenoid_1.set(DoubleSolenoid.Value.kReverse);
    
  }
}

public void ascendBack(boolean state) {
  if (!m_configured) {
    return;
  }
  if (state) {
    System.out.print("Acscend Back Activated\n");
    Solenoid_2.set(DoubleSolenoid.Value.kForward);
  }
  else{
    Solenoid_2.set(DoubleSolenoid.Value.kReverse);
  }
}

public void descendAssistFront(boolean state) {
  if (!m_configured) {
    return;
  }
  if (state) {
    System.out.print("Descend Front Activated\n");
    Solenoid_5.set(DoubleSolenoid.Value.kForward);
  }
  else {
    Solenoid_5.set(DoubleSolenoid.Value.kReverse);
  }

}

public void descendAssistBack(boolean state) {
  if (!m_configured) {
    return;
  }
  if (state) {
    System.out.print("Descend Back Activated\n");
    Solenoid_6.set(DoubleSolenoid.Value.kForward);
  }
  else {
    Solenoid_6.set(DoubleSolenoid.Value.kReverse);
  }
}

public void ascendAssistBack(boolean state) {
  if (!m_configured) {
    return;
  }
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