/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.OI;

public class TurnRobot extends Command {

  double targetAngle;

  public TurnRobot(double angle) {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(OI.m_drive);
    targetAngle = angle;
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    OI.m_drive.resetGyro();
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    if (OI.m_drive.getAngle() < targetAngle) {
      OI.m_drive.tankDrive(-0.8, 0.8);
    }

    if (OI.m_drive.getAngle() > targetAngle) {
      OI.m_drive.tankDrive(0.8, -0.8);
    }
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    if (Math.abs(OI.m_drive.getAngle() - targetAngle) <= 2) {
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
