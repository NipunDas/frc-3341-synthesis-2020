/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.OI;

public class TurnRobotTimer extends Command {

  private final Timer m_timer = new Timer();
  public double targetTurnTime;

  public TurnRobotTimer(double time) {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(OI.m_drive);
    targetTurnTime = time;
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
    if (targetTurnTime > 0) {
      OI.m_drive.tankDrive(-0.3, 0.3);
    } else {
      OI.m_drive.tankDrive(0.3, -0.3);
    }
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    if (m_timer.get() >= Math.abs(targetTurnTime)) {
      return true;
    } else {
      return false;
    }
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
