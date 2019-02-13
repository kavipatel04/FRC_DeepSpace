package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.Robot;
import frc.robot.RobotMap.MapKeys;
import frc.robot.commands.LiftCommand;
import java.math.*;


/**
 * Add your docs here.
 */
public class LiftSubsystem extends Subsystem {
  
  public TalonSRX m_motorup1;
  private TalonSRX m_motorup2;

  private double m_speed_ips; //inches per second
  private double m_position_in; //desired position in inches
  private double m_position_counts; 
  private static final double MAXSPEED = 20.0; //inches per second
  private static final double AUTOLIFTSPEED = 10.0; //inches per second
  private static final double SECONDS_PER_TICK = .02; // seconds per encoder tic
  private static final double COUNTS_PER_INCH = 150; // encoder counts per inch
  private static final int TALON_TIMEOUT_MS = 1000; 
  private static final double DISTANCE_PER_TICK = AUTOLIFTSPEED * SECONDS_PER_TICK; // inches travelled per encoder tick

  private double m_p = Robot.m_map.pidLiftMotor("p");
  private double m_i = Robot.m_map.pidLiftMotor("i");
  private double m_d = Robot.m_map.pidLiftMotor("d");
  private double m_maxIntegral = 1.0;
  private int m_maxAmps = 2;

  private boolean m_autoActive; //is auto active?
  private double m_autoDistance; //distance to travel autonomously

  //private double m_current; //current draw of the talon from the PDP

  public LiftSubsystem(){
    m_motorup1 = new TalonSRX(Robot.m_map.getId(MapKeys.LIFT_LEFT));
    m_motorup2 = new TalonSRX(Robot.m_map.getId(MapKeys.LIFT_RIGHT));
    setOutput(0.0);
    configTalons();
    m_position_in = 0.0;
    m_speed_ips = 0.0;


    
  }
  public void setOutput(double output) {
    if ((m_motorup1 != null) && (m_motorup2 != null)) {
      m_motorup1.set(ControlMode.PercentOutput, output);
      m_motorup2.set(ControlMode.PercentOutput, output);
    }
  }

  public void configTalons() {
    if ((m_motorup1 == null) || (m_motorup2 == null)) {
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
    if(m_autoActive) {

      if (Math.abs(m_autoDistance) < Math.abs(DISTANCE_PER_TICK)) { //if there is only a small distance left to travel, finishes auto move
        m_position_in += m_autoDistance;
        m_autoDistance = 0;
        m_autoActive = false;
      }else{
        if (m_autoDistance > 0) {
          m_position_in += DISTANCE_PER_TICK;
          m_autoDistance -= DISTANCE_PER_TICK;
        } else if (m_autoDistance < 0) {
          m_position_in -= DISTANCE_PER_TICK;
          m_autoDistance += DISTANCE_PER_TICK;
        }
        
      }
    } else {
      m_position_in += m_speed_ips * SECONDS_PER_TICK;
    }
    
    m_position_counts = m_position_in * COUNTS_PER_INCH; //converts desired position to counts
    if (m_motorup1 != null) { //moves tha motor to desired position
      m_motorup1.set(ControlMode.Position, m_position_counts);
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
  }
}
public boolean autoMoveFinished() {
	return !m_autoActive;
}

  
} 
