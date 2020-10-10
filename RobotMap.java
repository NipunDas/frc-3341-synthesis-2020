/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

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

  //Motor ports
  public static int leftDrivePort = 1;
  public static int rightDrivePort = 0;
  public static int extenderPort = 2;
  public static int liftPort = 3;

  //Joystick buttons/axes
  public static int xboxPort = 0;
  public static int leftJoystick = 1;
  public static int rightJoystick = 4;
  public static int leftBumper = 5;
  public static int rightBumper = 6;
  public static int triggers = 2;
  public static int aButton = 2;
  public static int bButton = 3;

}