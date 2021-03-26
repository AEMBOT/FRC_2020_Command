// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.hardware.pneumatics;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DoublePiston {
  
  // Double solenoid to control the piston
  private DoubleSolenoid solenoidPiston;

  /***
   * Constructs the solenoid / piston using 2 ports
   * @param forwardPort forward point on the solenoid
   * @param reversePort reverse point on the solenoid
   */
  public DoublePiston(int forwardPort, int reversePort){
      solenoidPiston = new DoubleSolenoid(forwardPort, reversePort);

      // Keep the current piston retracted at start so it doesn't fall outside the frame parameter
      retract();
  }

  /**
   * Toggle the status of the solenoid
   */
  public void toggle(){
    solenoidPiston.toggle();
  }

  /**
   * Retract the piston
   */
  public void retract(){
    solenoidPiston.set(Value.kForward);
  }

  /**
   * Starts a thread to retract the piston
   */
  public void actuate(){
    solenoidPiston.set(Value.kReverse);
  }

  /**
   * Sets the solenoid that controls the piston to closed and doesn't allow air to flow through
   */
  public void closeSolenoid(){
      solenoidPiston.set(Value.kOff);
  }

  /**
   * Gets the current value of the piston solenoid (kOff, kForward, kClose)
   */
  public Value getCurrentPistonState(){
      return solenoidPiston.get();
  }

}
