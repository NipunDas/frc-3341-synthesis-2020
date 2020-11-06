/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.PWMTalonSRX;
import edu.wpi.first.wpilibj.Encoder;
import frc.robot.RobotMap;
import frc.robot.OI;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;

/**
 * Add your docs here.
 */
public class DriveTrain extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  private PWMTalonSRX left = new PWMTalonSRX(RobotMap.leftDrivePort), right = new PWMTalonSRX(RobotMap.rightDrivePort);
  public static DriveTrain drive;
  private Encoder leftEncoder = new Encoder(5, 6), rightEncoder = new Encoder(3, 4);
  private ADXRS450_Gyro gyro = new ADXRS450_Gyro();
  private double ticksToInches = (7 * Math.PI)/360;

  public DriveTrain() {
    right.setInverted(true);
    leftEncoder.reset();
    leftEncoder.reset();
    gyro.reset();
  }

  public static DriveTrain getInstance() {
    if (drive == null) {
      drive = new DriveTrain();
    }
    return drive;
  }

  public void tankDrive(double leftPow, double rightPow) {

    //Minimizing error from small inputs
    if (leftPow < 0.05 && leftPow > -0.05) {
      leftPow = 0;
    }
    if (rightPow < 0.05 && rightPow > -0.05) {
      rightPow = 0;
    }

    left.set(leftPow);
    right.set(rightPow);
  }
  
  public void resetEncoders() {
    leftEncoder.reset();
    leftEncoder.reset();
  }

  public double returnDistance() {
    return ((leftEncoder.getRaw() + rightEncoder.getRaw())/2) * ticksToInches;
  }

  public void resetGyro() {
    gyro.reset();
  }

  public double getAngle() {
    return gyro.getAngle();
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }

  @Override
  public void periodic() {
    tankDrive(OI.returnController().getRawAxis(RobotMap.leftJoystick) * -0.3, OI.returnController().getRawAxis(RobotMap.rightJoystick) * -0.3);
  }
}
