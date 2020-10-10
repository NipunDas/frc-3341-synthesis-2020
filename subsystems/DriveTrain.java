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
  private Encoder leftEncoder = new Encoder(0, 1), rightEncoder = new Encoder(2, 3);
  private ADXRS450_Gyro gyro = new ADXRS450_Gyro();

  public DriveTrain() {
    left.setInverted(true);
    leftEncoder.reset();
    rightEncoder.reset();
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
    rightEncoder.reset();
  }

  public double returnDistance() {
    double leftDistance = (leftEncoder.getRaw()/4096) * 3.5;
    double rightDistance = (rightEncoder.getRaw()/4096) * 3.5;
    return (leftDistance + rightDistance)/2;
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
    tankDrive(OI.returnController().getRawAxis(RobotMap.leftJoystick) * 0.3, OI.returnController().getRawAxis(RobotMap.rightJoystick) * 0.3);
  }
}
