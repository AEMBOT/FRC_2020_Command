// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.ballSystem;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.RobotMap.*;

public class Hopper extends SubsystemBase {

  // Used to drive the transport belts
  private final CANSparkMax transportBeltMotor;

  /** Creates a new Hopper. */
  public Hopper() {
    // Create the transport belt motor with a 1/2 second ramp rate
    transportBeltMotor = new CANSparkMax(BeltMotor, MotorType.kBrushless);
    transportBeltMotor.setOpenLoopRampRate(0.5);
    transportBeltMotor.setInverted(true);
  }

  /**
   * Run the hopper belts at a set speed
   * 
   * @param power duty to cycle to run the belts at
   */
  public void runBelts(double power) {
    transportBeltMotor.set(power);
  }

  /***
   * Get the current belt current
   * @return motor current
   */
  public double getBeltCurrent() {
    return transportBeltMotor.getOutputCurrent();
  }

  /***
   * Get the current belt power
   * @return belt duty cycle
   */
  public double getBeltPower(){
    return transportBeltMotor.get();
  }
}
