/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.PWMTalonSRX;
import frc.robot.RobotMap;
import frc.robot.OI;

/**
 * Add your docs here.
 */
public class Grabber extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  private PWMTalonSRX extender = new PWMTalonSRX(RobotMap.extenderPort);
  private PWMTalonSRX lift = new PWMTalonSRX(RobotMap.liftPort);
  
  public static Grabber grabberInstance;

  public Grabber() {
    lift.setInverted(true);
  }

  public static Grabber getInstance() {
    if (grabberInstance == null) {
      grabberInstance = new Grabber();
    }
    return grabberInstance;
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }

  public void setArmPower(double power) {
    lift.set(power);
  }

  @Override
  public void periodic() {

    //Arm extension
    if (OI.returnController().getRawButton(RobotMap.leftBumper)) {
      extender.set(0.1);
    } else if (OI.returnController().getRawButton(RobotMap.rightBumper)) {
      extender.set(-0.1);
    } else {
      extender.set(0);
    }

    //Lift
    lift.set(Math.abs(OI.returnController().getRawAxis(RobotMap.triggers) * 0.3));
  }
}
