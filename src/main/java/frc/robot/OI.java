package frc.robot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.JoystickButton;

import frc.robot.commands.*;
import java.lang.Math.*;





public class OI{


  Joystick joy = new Joystick(0);

  private final double FORWARD_DEADBAND_LIMIT = .1;
  private final double TURN_DEADBAND_LIMIT = .1;

  int CLIMB_BUTTON = 1;
  public JoystickButton xButton = new JoystickButton(joy, CLIMB_BUTTON);
  JoystickButton a = new JoystickButton(joy, 1);
  JoystickButton b = new JoystickButton(joy, 2);
  JoystickButton x = new JoystickButton(joy, 3);
  JoystickButton y = new JoystickButton(joy, 4);
  private static final int POS_HOLD_TOGGLE_BUTTON = 2;  // B button
  private static final int POS_HOLD_UPDATE_BUTTON = 4;  // Y button

   private static final double MAX_POS_DEG = 90.0;       // how far to rotate shaft at full joystick deflection.
   private static final int POS_HOLD_AXIS = 0;           // left-right on left joystick

   private static final Joystick m_controller = new Joystick(1);
  private static final JoystickButton m_b = new JoystickButton(m_controller, POS_HOLD_TOGGLE_BUTTON);
  private static final JoystickButton m_y = new JoystickButton(m_controller, POS_HOLD_UPDATE_BUTTON);

  private final int CAMERA_BUTTON = 2;
  private final int Lvl2DESC_BUTTON = 7;
  private final int Lvl2CLIMB_BUTTON = 8;
  private final int Lvl3CLIMB_BUTTON = 3;//cant find button
  private final int CLIMBNEXT_BUTTON = 6;
  private final int CLIMBPREV_BUTTON = 5;
  public JoystickButton bButton1 = new JoystickButton(joy, CAMERA_BUTTON);
	public JoystickButton backButton1 = new JoystickButton(joy, Lvl2DESC_BUTTON);
	public JoystickButton startButton1 = new JoystickButton(joy, Lvl2CLIMB_BUTTON);
  public JoystickButton leftBumper1 = new JoystickButton(joy, CLIMBPREV_BUTTON);
  public JoystickButton rightBumper1 = new JoystickButton(joy, CLIMBNEXT_BUTTON);
  public JoystickButton xButton1 = new JoystickButton(joy, Lvl3CLIMB_BUTTON);
    
  Joystick joy2 = new Joystick(1);


  private final int HIGHHATCH_BUTTON = 4;
  private final int MIDHATCH_BUTTON = 3;
  private final int LOWHATCH_BUTTON = 1;
  private final int CLAMP_BUTTON = 2;

  public JoystickButton yButton2 = new JoystickButton(joy2, HIGHHATCH_BUTTON);
  public JoystickButton xButton2 = new JoystickButton(joy2, MIDHATCH_BUTTON);
  public JoystickButton aButton2 = new JoystickButton(joy2, LOWHATCH_BUTTON);
  public JoystickButton bButton2 = new JoystickButton(joy2, CLAMP_BUTTON);

  public double turnModifier = .4;
  public double driveModifier = .9;


  public OI () {

    //bButton1.toggleWhenPressed(new CameraToggle);

    backButton1.whenPressed(new DescendCommandGroup()); //initiate lvl 2 descent
    startButton1.whenPressed(new L2AscendCommandGroup()); //initiate lvl 2 climb
    xButton1.whenPressed(new L3AscendCommandGroup()); // intiate lvl 3 climb

    leftBumper1.whenPressed(new ClimbPrevCommand());
    rightBumper1.whenPressed(new ClimbNextCommand());

    yButton2.whenPressed(new LiftHighCommand()); //move lift to high hatch position
    xButton2.whenPressed(new LiftMidCommand()); //move lift to mid hatch position
    aButton2.whenPressed(new LiftLowCommand()); //move lift to low hatch position

    bButton2.whenActive(new GrabberExtend()); //toggle for clamp
    bButton2.whenInactive(new GrabberRetract()); //toggle for clamp
    

    
  }
  public double readPositionDeg() {
    double input = m_controller.getRawAxis(POS_HOLD_AXIS);  // get joystick position, in range -1.0 to 1.0
    double holdPos = input * MAX_POS_DEG;     // convert to degrees

    return holdPos;
  } 
 



  public double getForwardValue() {
    if (Math.abs(joy.getRawAxis(1)) < FORWARD_DEADBAND_LIMIT) { //creates deadband
      return 0;
    } else {
      joy.getRawAxis(1);
    }
    return joy.getRawAxis(1) * driveModifier;
  }
  public double getTurnValue() {
    if (Math.abs(joy.getRawAxis(4)) < TURN_DEADBAND_LIMIT) {
      return 0;
    } else {
      return joy.getRawAxis(4) * turnModifier;
    }
    
  }

  public double getLiftValue() {
    return joy2.getRawAxis(1);
  }

  public double getSlideValue() {
    return joy2.getRawAxis(3) - joy2.getRawAxis(2);
  }


  


}
