/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.OI;
import edu.wpi.first.wpilibj.Timer;

public class EncoderDriveForward extends Command {

  private final Timer m_timer = new Timer();
  private double kP = 100; //kI = 1, kD = 1;
  private double targetDistance, error, distanceTravelled;
  private double power;

  public EncoderDriveForward(double distance) {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(OI.m_drive);
    targetDistance = distance;
    error = distance;
    power = kP * error;
    distanceTravelled = 0;
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    OI.m_drive.resetEncoders();
    m_timer.reset();
    m_timer.start();
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    distanceTravelled = OI.m_drive.returnDistance();
    error = targetDistance - distanceTravelled;
    power = kP * error;
    OI.m_drive.tankDrive(power, power);
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return Math.abs(error) < 2;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    OI.m_drive.tankDrive(0, 0);
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}
