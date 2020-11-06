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

public class ArmPID extends Command {

  private final Timer m_timer = new Timer();
  private double kP = 1;
  private double targetDistance = 20, error = 20;
  private double previousTime = 0, distanceSum = 0, power, velocity, dt;
  private double basePower = 0.05;
  private double powerToVelocityConstant = 20;

  public ArmPID() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(OI.m_grabber);
    power = (kP * targetDistance) + basePower;
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
    power = (kP * power) + basePower;
    OI.m_grabber.setArmPower(power);
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return error < 0;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    OI.m_grabber.setArmPower(power);
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}
