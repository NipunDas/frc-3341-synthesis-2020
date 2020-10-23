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

public class PIDDriveForward extends Command {

  private final Timer m_timer = new Timer();
  private double kP = 100;// kI = 1, kD = 1;
  private double targetDistance, error;
  private double previousTime = 0, distanceSum = 0, power, velocity, dt;
  private double powerToVelocityConstant = 20;
  private boolean isNegative;

  public PIDDriveForward(double distance) {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(OI.m_drive);
    targetDistance = Math.abs(distance);
    isNegative = (distance < 0);
    power = kP/distance;
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    m_timer.reset();
    m_timer.start();
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    velocity = power * powerToVelocityConstant;
    dt = m_timer.get() - previousTime;
    distanceSum += velocity * dt;
    previousTime = m_timer.get();

    error = targetDistance - distanceSum;
    power = kP/error;
    if (isNegative) {
      OI.m_drive.tankDrive(power, power);
    } else {
      OI.m_drive.tankDrive(-power, -power);
    }
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return error <= 0;
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
