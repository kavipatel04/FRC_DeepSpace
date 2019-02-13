/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.util.EnumMap;

import edu.wpi.first.wpilibj.Preferences;

/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */
public class RobotMap {
  // For example to map the left and right motors, you could define the
  // following variables to use with your drivetrain subsystem.
  // public static int leftMotor = 1;
  // public static int rightMotor = 2;

  // If you are using multiple modules, make sure to define both the port
  // number and the module. For example you with a rangefinder:
  // public static int rangefinderPort = 1;
  // public static int rangefinderModule = 1;
  Preferences prefs;
  public enum MapKeys {
    DRIVE_FRONTLEFT, DRIVE_FRONTRIGHT, DRIVE_BACKLEFT, DRIVE_BACKRIGHT,
    SLIDE, LIFT_LEFT, LIFT_RIGHT, 
    PCM_CLIMBCANID, 
    SOLENOID_FRONTLEFTEXTEND, SOLENOID_FRONTLEFTRETRACT, SOLENOID_BACKLEFTEXTEND, SOLENOID_BACKLEFTRETRACT, SOLENOID_FRONTRIGHTEXTEND, SOLENOID_FRONTRIGHTRETRACT, SOLENOID_BACKRIGHTEXTEND, SOLENOID_BACKRIGHTRETRACT,
    PCM_CLIMBCANID2,
    SOLENOID_LOWERFRONTEXTEND, SOLENOID_LOWERFRONTRETRACT, SOLENOID_LOWERBACKEXTEND, SOLENOID_LOWERBACKRETRACT, SOLENOID_ASCENDASSISTBACKLEFTEXTEND, SOLENOID_ASCENDASSISTBACKLEFTRETRACT, SOLENOID_ASCENDASSISTBACKRIGHTEXTEND, SOLENOID_ASCENDASSISTBACKRIGHTRETRACT,
    PCM_ARMCANID,
    SOLENOID_PUSHERPUSH, SOLENOID_PUSHERRETRACT, SOLENOID_ARMFORWARD, SOLENOID_ARMREVERSE, BACKLEFTCLIMBWHEEL, BACKRIGHTCLIMBWHEEL

  }

  public enum RobotTypes {
    ALBERT, BRIEFCASE, DEEPSPACE_ROBOT, DEADPIXEL
  }

  public RobotMap() {

    // BRIEFCASE IDs BEGIN HERE #########################################

    briefcase.put(MapKeys.DRIVE_FRONTLEFT, 1);
    briefcase.put(MapKeys.DRIVE_FRONTRIGHT, 3);
    briefcase.put(MapKeys.DRIVE_BACKLEFT, 0);
    briefcase.put(MapKeys.DRIVE_BACKRIGHT, 0);

    briefcase.put(MapKeys.SLIDE, 0);
    briefcase.put(MapKeys.LIFT_LEFT, 0);
    briefcase.put(MapKeys.LIFT_RIGHT, 0);

    briefcase.put(MapKeys.PCM_CLIMBCANID, 0);
    briefcase.put(MapKeys.SOLENOID_FRONTLEFTEXTEND, 0);
    briefcase.put(MapKeys.SOLENOID_FRONTLEFTRETRACT, 0);
    briefcase.put(MapKeys.SOLENOID_BACKLEFTEXTEND, 0);
    briefcase.put(MapKeys.SOLENOID_BACKLEFTRETRACT, 0);
    briefcase.put(MapKeys.SOLENOID_FRONTRIGHTEXTEND, 0);
    briefcase.put(MapKeys.SOLENOID_FRONTRIGHTRETRACT, 0);
    briefcase.put(MapKeys.SOLENOID_BACKRIGHTEXTEND, 0);
    briefcase.put(MapKeys.SOLENOID_BACKRIGHTRETRACT, 0);

    briefcase.put(MapKeys.PCM_CLIMBCANID2, 0);
    briefcase.put(MapKeys.SOLENOID_LOWERFRONTEXTEND, 0);
    briefcase.put(MapKeys.SOLENOID_LOWERFRONTRETRACT, 0);
    briefcase.put(MapKeys.SOLENOID_LOWERBACKEXTEND, 0);
    briefcase.put(MapKeys.SOLENOID_LOWERBACKRETRACT, 0);
    briefcase.put(MapKeys.SOLENOID_ASCENDASSISTBACKLEFTEXTEND, 0);
    briefcase.put(MapKeys.SOLENOID_ASCENDASSISTBACKLEFTRETRACT, 0);
    briefcase.put(MapKeys.SOLENOID_ASCENDASSISTBACKRIGHTEXTEND, 0);
    briefcase.put(MapKeys.SOLENOID_ASCENDASSISTBACKRIGHTRETRACT, 0);

    briefcase.put(MapKeys.PCM_ARMCANID, 0);
    briefcase.put(MapKeys.SOLENOID_PUSHERPUSH, 0);
    briefcase.put(MapKeys.SOLENOID_PUSHERRETRACT, 0);
    briefcase.put(MapKeys.SOLENOID_ARMFORWARD, 0);
    briefcase.put(MapKeys.SOLENOID_ARMREVERSE, 0);

    // ALBERT IDs BEGIN HERE #########################################

    albert.put(MapKeys.DRIVE_FRONTLEFT, 20);
    albert.put(MapKeys.DRIVE_FRONTRIGHT, 11);
    albert.put(MapKeys.DRIVE_BACKLEFT, 21);
    albert.put(MapKeys.DRIVE_BACKRIGHT, 10);

    albert.put(MapKeys.BACKLEFTCLIMBWHEEL, 2);
    albert.put(MapKeys.BACKRIGHTCLIMBWHEEL, 3);

    albert.put(MapKeys.SLIDE, 3);
    
    albert.put(MapKeys.LIFT_LEFT, 0);
    albert.put(MapKeys.LIFT_RIGHT, 0);

    albert.put(MapKeys.PCM_CLIMBCANID, 0);
    albert.put(MapKeys.SOLENOID_FRONTLEFTEXTEND, 0);
    albert.put(MapKeys.SOLENOID_FRONTLEFTRETRACT, 1);
    albert.put(MapKeys.SOLENOID_BACKLEFTEXTEND, 2);
    albert.put(MapKeys.SOLENOID_BACKLEFTRETRACT, 3);
    albert.put(MapKeys.SOLENOID_FRONTRIGHTEXTEND,6);
    albert.put(MapKeys.SOLENOID_FRONTRIGHTRETRACT, 7);
    albert.put(MapKeys.SOLENOID_BACKRIGHTEXTEND, 4);
    albert.put(MapKeys.SOLENOID_BACKRIGHTRETRACT, 5);

    albert.put(MapKeys.PCM_CLIMBCANID2, 6);
    albert.put(MapKeys.SOLENOID_LOWERFRONTEXTEND, 0);
    albert.put(MapKeys.SOLENOID_LOWERFRONTRETRACT, 1);
    albert.put(MapKeys.SOLENOID_LOWERBACKEXTEND, 2);
    albert.put(MapKeys.SOLENOID_LOWERBACKRETRACT, 3);
    albert.put(MapKeys.SOLENOID_ASCENDASSISTBACKLEFTEXTEND, 4);
    albert.put(MapKeys.SOLENOID_ASCENDASSISTBACKLEFTRETRACT, 5);
    albert.put(MapKeys.SOLENOID_ASCENDASSISTBACKRIGHTEXTEND, 6);
    albert.put(MapKeys.SOLENOID_ASCENDASSISTBACKRIGHTRETRACT, 7);

    albert.put(MapKeys.PCM_ARMCANID, 7);
    albert.put(MapKeys.SOLENOID_PUSHERPUSH, 0);
    albert.put(MapKeys.SOLENOID_PUSHERRETRACT, 1);
    albert.put(MapKeys.SOLENOID_ARMFORWARD, 2);
    albert.put(MapKeys.SOLENOID_ARMREVERSE, 3);

    // DEADPIXEL IDs BEGIN HERE #########################################

    deadpixel.put(MapKeys.DRIVE_FRONTLEFT, 10);
    deadpixel.put(MapKeys.DRIVE_FRONTRIGHT, 11);
    deadpixel.put(MapKeys.DRIVE_BACKLEFT, 7);
    deadpixel.put(MapKeys.DRIVE_BACKRIGHT, 13);

    deadpixel.put(MapKeys.SLIDE, 0);
    deadpixel.put(MapKeys.LIFT_LEFT, 0);
    deadpixel.put(MapKeys.LIFT_RIGHT, 0);

    deadpixel.put(MapKeys.PCM_CLIMBCANID, 5);
    deadpixel.put(MapKeys.SOLENOID_FRONTLEFTEXTEND, 0);
    deadpixel.put(MapKeys.SOLENOID_FRONTLEFTRETRACT, 1);
    deadpixel.put(MapKeys.SOLENOID_BACKLEFTEXTEND, 2);
    deadpixel.put(MapKeys.SOLENOID_BACKLEFTRETRACT, 3);
    deadpixel.put(MapKeys.SOLENOID_FRONTRIGHTEXTEND, 4);
    deadpixel.put(MapKeys.SOLENOID_FRONTRIGHTRETRACT, 5);
    deadpixel.put(MapKeys.SOLENOID_BACKRIGHTEXTEND, 6);
    deadpixel.put(MapKeys.SOLENOID_BACKRIGHTRETRACT, 7);

    deadpixel.put(MapKeys.PCM_CLIMBCANID2, 0);
    deadpixel.put(MapKeys.SOLENOID_LOWERFRONTEXTEND, 0);
    deadpixel.put(MapKeys.SOLENOID_LOWERFRONTRETRACT, 0);
    deadpixel.put(MapKeys.SOLENOID_LOWERBACKEXTEND, 0);
    deadpixel.put(MapKeys.SOLENOID_LOWERBACKRETRACT, 0);
    deadpixel.put(MapKeys.SOLENOID_ASCENDASSISTBACKLEFTEXTEND, 0);
    deadpixel.put(MapKeys.SOLENOID_ASCENDASSISTBACKLEFTRETRACT, 0);
    deadpixel.put(MapKeys.SOLENOID_ASCENDASSISTBACKRIGHTEXTEND, 0);
    deadpixel.put(MapKeys.SOLENOID_ASCENDASSISTBACKRIGHTRETRACT, 0);

    deadpixel.put(MapKeys.PCM_ARMCANID, 0);
    deadpixel.put(MapKeys.SOLENOID_PUSHERPUSH, 0);
    deadpixel.put(MapKeys.SOLENOID_PUSHERRETRACT, 0);
    deadpixel.put(MapKeys.SOLENOID_ARMFORWARD, 0);
    deadpixel.put(MapKeys.SOLENOID_ARMREVERSE, 0);

  }


  public RobotTypes activeRobot = RobotTypes.ALBERT;

  public int getId(MapKeys key) {
    if(activeRobot == RobotTypes.BRIEFCASE) {
      return briefcase.get(key);
    }
    else if(activeRobot == RobotTypes.DEADPIXEL) {
      return deadpixel.get(key);
    }
    else {
      return albert.get(key);
    }
  }
  
  public double pidSlideMotor(String x) {
    if (x.toLowerCase() == "p") {
      return prefs.getDouble("P Slide Motor", 0.0);
    } else if (x.toLowerCase() == "i") {
      return prefs.getDouble("I Slide Motor", 0.0);
    } else if (x.toLowerCase() == "d") {
      return prefs.getDouble("D Slide Motor", 0.0);
    } else {
      System.out.println("Valid PID letter not entered");
      return 0.0;
    }
  }

  public double pidLiftMotor(String x) {
    if (x.toLowerCase() == "p") {
      return prefs.getDouble("P Lift Motor", 0.0);
    } else if (x.toLowerCase() == "i") {
      return prefs.getDouble("I Lift Motor", 0.0);
    } else if (x.toLowerCase() == "d") {
      return prefs.getDouble("D Lift Motor", 0.0);
    } else {
      System.out.println("Valid PID letter not entered");
      return 0.0;
    }
  }

  public double pidDriveLeft(String x) {
    if (x.toLowerCase() == "p") {
      return prefs.getDouble("P Drive Left", 0.0);
    } else if (x.toLowerCase() == "i") {
      return prefs.getDouble("I Drive Left", 0.0);
    } else if (x.toLowerCase() == "d") {
      return prefs.getDouble("D Drive Left", 0.0);
    } else if (x.toLowerCase() == "f") {
      return prefs.getDouble("F Drive Left", 0.0);
    } else {
      System.out.println("Valid PIDF letter not entered");
      return 0.0;
    }
  }

  public double pidDriveRight(String x) {
    if (x.toLowerCase() == "p") {
      return prefs.getDouble("P Drive Right", 0.0);
    } else if (x.toLowerCase() == "i") {
      return prefs.getDouble("I Drive Right", 0.0);
    } else if (x.toLowerCase() == "d") {
      return prefs.getDouble("D Drive Right", 0.0);
    } else if (x.toLowerCase() == "f") {
      return prefs.getDouble("F Drive Right", 0.0);
    } else {
      System.out.println("Valid PIDF letter not entered");
      return 0.0;
    }
  }

  /** public void pidPrefMethod() {
    prefs.putDouble("P Slide Motor", pidSlideMotor("p"));
    prefs.putDouble("I Slide Motor", pidSlideMotor("i"));
    prefs.putDouble("D Slide Motor", pidSlideMotor("d"))
    prefs.getDouble("P Lift Motor", pidLiftMotor("p"));
    prefs.getDouble("I Lift Motor", pidLiftMotor("i"));
    prefs.getDouble("D Lift Motor", pidLiftMotor("d"));

    prefs.getDouble("P Left Drive Motor", pidDriveLeft("p"));
    prefs.getDouble("I Left Drive Motor", pidDriveLeft("i"));
    prefs.getDouble("D Left Drive Motor", pidDriveLeft("d"));
    prefs.getDouble("F Left Drive Motor", pidDriveLeft("f"));

    prefs.getDouble("P Right Drive Motor", pidDriveLeft("p"));
    prefs.getDouble("I Right Drive Motor", pidDriveLeft("i"));
    prefs.getDouble("D Right Drive Motor", pidDriveLeft("d"));
    prefs.getDouble("F Right Drive Motor", pidDriveLeft("f"));
  } */

  public EnumMap<MapKeys, Integer> briefcase = new EnumMap<MapKeys, Integer>(MapKeys.class);

  public EnumMap<MapKeys, Integer> albert = new EnumMap<MapKeys, Integer>(MapKeys.class);

  public EnumMap<MapKeys, Integer> deadpixel = new EnumMap<MapKeys, Integer>(MapKeys.class);

  


}
