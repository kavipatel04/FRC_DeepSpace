package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.Robot;
import frc.robot.RobotMap.MapKeys;
import frc.robot.commands.LiftCommand;
import java.lang.Math;


/**
 * Add your docs here.
 */
public class LiftSubsystem extends Subsystem {
  
  public TalonSRX m_motorup1;
  public TalonSRX m_motorup2;

  private double m_speed_ips; //inches per second
  private double m_position_in; //desired position in inches
  private double m_position_counts; 
  private static final double MAXSPEED = 40.0; //inches per second
  private static final double AUTOLIFTDOWN = 45.0; //inches per second
  private static final double AUTOLIFTSPEED = 45.0; //inches per second
  private static final double SECONDS_PER_TICK = 0.02; // seconds per encoder tic
  private static final double COUNTS_PER_INCH = -1366; // encoder counts per inch (formerly 150)
  private static final int TALON_TIMEOUT_MS = 1000; 
  private static final double DISTANCE_PER_TICK = AUTOLIFTSPEED * SECONDS_PER_TICK; // inches travelled per encoder tick
  private static final double DISTANCE_PER_TICK_DOWN = AUTOLIFTDOWN *SECONDS_PER_TICK; //inches travelled 

  private double m_p = 0.5;
  private double m_i = 0.0;
  private double m_d = 10.0;
  private double m_maxIntegral = 1.0;
  private int m_maxAmps = 10;

  private boolean m_autoActive; //is auto active?
  private double m_autoDistance; //distance to travel autonomously
  private double m_autoSpeed_ips = 0;
  
  //Acceleration
  private static final double AUTO_ACCEL_IPS2 = 200.0;
  private static final double MAX_VEL_IPS = 100.0;
  private static final double AUTO_MIN_SPEED = 1.0; //inches per second
  private static final double AUTO_MIN_DIST = 1.0; //inches
  



  public double HIGH_POS_IN;
  public double MID_POS_IN;  
  public double LOW_POS_IN;

  //private double m_current; //current draw of the talon from the PDP

  public LiftSubsystem(){
    m_position_in = 0.0;
    m_speed_ips = 0.0;
  }
  public void initialize() {
    Preferences prefs = Preferences.getInstance();

    HIGH_POS_IN = prefs.getDouble("Lift_High_Pos", 57.0);
    MID_POS_IN = prefs.getDouble("Lift_Mid_Pos", 32.0); // was 29
    LOW_POS_IN = prefs.getDouble("Lift_Low_Pos", 0.0);

    m_p = prefs.getDouble("Lift_P_Value", 0.5);
    m_i = prefs.getDouble("Lift_I_Value", 0.0);
    m_d = prefs.getDouble("Lift_D_Value", 10.0);

    

    int leftCanID = Robot.m_map.getId(MapKeys.LIFT_LEFT);
    int rightCanID = Robot.m_map.getId(MapKeys.LIFT_RIGHT);
    if ((leftCanID != 0) && (rightCanID !=0)){
      m_motorup1 = new TalonSRX(leftCanID);
      m_motorup2 = new TalonSRX(rightCanID);
      setOutput(0.0);
      configTalons();
    }
  }


  /*
  public void Initialize() {
    Preferences prefs = Preferences.getInstance();

    m_p = prefs.getDouble("Lift_P_Value", 0.5);
    m_i = prefs.getDouble("Lift_I_Value", 0.0);
    m_d = prefs.getDouble("Lift_D_Value", 10.0);

    HIGH_POS_IN = prefs.getDouble("Lift_High_Pos", 57.0);
    MID_POS_IN = prefs.getDouble("Lift_Mid_Pos", 26.0);
    LOW_POS_IN = prefs.getDouble("Lift_Low_Pos", 0.0);
    
    m_motorup1 = new TalonSRX(Robot.m_map.getId(MapKeys.LIFT_LEFT));
    m_motorup2 = new TalonSRX(Robot.m_map.getId(MapKeys.LIFT_RIGHT));
    setOutput(0.0);
    configTalons();

  }
*/
  public void setOutput(double output) {
    if ((m_motorup1 != null) && (m_motorup2 != null)) {
      m_motorup1.set(ControlMode.PercentOutput, output);
      m_motorup2.set(ControlMode.PercentOutput, output);
    }
  }

  public void configTalons() {

   

    if ((m_motorup1 == null) && (m_motorup2 == null)) {
      return;
    }

    m_motorup1.selectProfileSlot(0, 0);
    m_motorup1.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, TALON_TIMEOUT_MS);
    m_motorup1.setSensorPhase(false); 

    m_motorup1.config_kP(0, m_p, TALON_TIMEOUT_MS);
    m_motorup1.config_kI(0, m_i, TALON_TIMEOUT_MS);
    m_motorup1.config_kD(0, m_d, TALON_TIMEOUT_MS);

    m_motorup1.configMaxIntegralAccumulator(0, m_maxIntegral, TALON_TIMEOUT_MS);
    m_motorup1.configContinuousCurrentLimit(m_maxAmps, TALON_TIMEOUT_MS);
    m_motorup1.configPeakCurrentLimit(m_maxAmps, TALON_TIMEOUT_MS);

    m_motorup1.setSelectedSensorPosition(0, 0 , TALON_TIMEOUT_MS); //sets current pos to be 0
    m_position_counts = 0;
    m_motorup1.setIntegralAccumulator(0);
    m_motorup1.set(ControlMode.Position, m_position_counts); // moves motor to 0

    m_motorup2.set(ControlMode.Follower, Robot.m_map.getId(MapKeys.LIFT_LEFT));
  }

  /** public void liftToBottom() {
    current = Robot.m_pdp.getCurrent(14);
    while(current)
    setOutput(-0.5);
  } */

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    setDefaultCommand(new LiftCommand());
     
  }
  

  public void update(double x) {
    m_speed_ips = x * MAXSPEED;
    //in the commands, converts joystick to motor speed
  }

  @Override
  public void periodic() {
    

    //if auto is active, do second if loop, if not, revert to manual position
    //if(m_autoActive) {
      /*
      if (Math.abs(m_autoDistance) < Math.abs(DISTANCE_PER_TICK)) { //if there is only a small distance left to travel, finishes auto move
        m_position_in += m_autoDistance;
        m_autoDistance = 0;
        m_autoActive = false;
      }else{
        if (m_autoDistance > 0) {  //going up
          m_position_in += DISTANCE_PER_TICK;
          m_autoDistance -= DISTANCE_PER_TICK;
          ///NEW STUFF
        distance = m_autoDistance;
        velocity = 
          up = true;
        } else if (m_autoDistance < 0) {  //going down
          m_position_in -= DISTANCE_PER_TICK_DOWN;
          m_autoDistance += DISTANCE_PER_TICK_DOWN;
          up = false;
        }
        
      }*/
      ///NEW STUFF
    if(m_autoActive) {
      double d = m_autoDistance;
      double v = m_autoSpeed_ips;
      double a = AUTO_ACCEL_IPS2;
      boolean up = true;
      
      if (d < 0) {
        //moving backwards (down)
        d = -d;
        v = -v;
        up = false;
      }
      
      double t = Math.sqrt((2.0 * d) / a);
      double speed_limit = a*(t- SECONDS_PER_TICK);
      
      if ((v >= speed_limit) || (v > AUTOLIFTSPEED)){
        v -= a*SECONDS_PER_TICK;
      } else if (((v + a*SECONDS_PER_TICK) < speed_limit) && 
                 ((v + a*SECONDS_PER_TICK) < AUTOLIFTSPEED))
      {
        v += a * SECONDS_PER_TICK;
      } else {
        v = v;
      }

      double dx = v * SECONDS_PER_TICK;

      if (up){
        m_autoDistance -= dx;
        m_position_in += dx;
        m_autoSpeed_ips = v;
      } else{
        m_autoDistance += dx;
        m_position_in -= dx;
        m_autoSpeed_ips = -v;
      }

      System.out.printf("POS: %5.2f Speed:%5.2f\n", m_position_in, m_autoSpeed_ips);


      //check for end condition
      if((Math.abs(m_autoSpeed_ips) < AUTO_MIN_SPEED) && 
      (Math.abs(m_autoDistance) < AUTO_MIN_DIST)){
        m_autoDistance = 0.0;
        m_autoActive = false;
      } 

    } else {
      m_position_in += m_speed_ips * SECONDS_PER_TICK;
    }
    
    m_position_counts = m_position_in * COUNTS_PER_INCH; //converts desired position to counts
    if (m_motorup1 != null) { //moves tha motor to desired position
      m_motorup1.set(ControlMode.Position, m_position_counts);
      //System.out.println(m_position_counts);
    }

    

  }
public void startAutoMove(double position_in) {
  if (m_autoActive) {
    //cancels auto move if previous one was still active when this method is called again
    m_autoActive = false;
    m_autoDistance = 0;
  } else {
    //activate auto lift
    m_autoActive = true;
    m_autoDistance = position_in - m_position_in; //sets auto distance to travel to the desired - current desired distance
    m_autoSpeed_ips = m_speed_ips;
    System.out.printf("Moving to %f\n", position_in);
  }
}
public boolean autoMoveFinished() {
	return !m_autoActive;
}

public void teleopInit(){
  /*m_motorup1.setSelectedSensorPosition(0, 0 , TALON_TIMEOUT_MS); //sets current pos to be 0
  m_position_counts = 0;
  m_motorup1.setIntegralAccumulator(0);
  m_motorup1.set(ControlMode.Position, m_position_counts);*/
  m_autoDistance = 0;
  m_autoActive = false;
  configTalons();
}
public void autonomousInit(){
  /*m_motorup1.setSelectedSensorPosition(0, 0 , TALON_TIMEOUT_MS); //sets current pos to be 0
  m_position_counts = 0;
  m_motorup1.setIntegralAccumulator(0);
  m_motorup1.set(ControlMode.Position, m_position_counts);*/
  m_autoDistance = 0;
  m_autoActive = false;
  configTalons();
}

} 
