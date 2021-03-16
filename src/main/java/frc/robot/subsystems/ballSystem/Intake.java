// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.ballSystem;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.hardware.pneumatics.DoublePiston;

import static frc.robot.RobotMap.*;

public class Intake extends SubsystemBase {

  // Used on the intake itself
  private final CANSparkMax intakeMotor;

  // Used to push in and out the intake
  private final DoublePiston intakePiston;

  /** Creates a new Intake. */
  public Intake() {
    // Create the intake motor and set an arbitrary ramp rate of half a second
    intakeMotor = new CANSparkMax(FrontIntakeMotor, MotorType.kBrushless);
    intakeMotor.setOpenLoopRampRate(0.5);

    // Invert the intake motor so forward is actually forward
    intakeMotor.setInverted(true);

    // Create the piston to drive the intake arm
    intakePiston = new DoublePiston(IntakePistonA, IntakePistonB);
  }

  /**
   * Run the intake motor at a set power
   * 
   * @param power duty cycle to give the motor
   */
  public void runIntake(double power) {
    intakeMotor.set(power);
  }

  /**
   * Extends the pistons to push the intake out
   */
  public void extendIntake() {
    intakePiston.actuate();
  }

  /**
   * Retract the intake pistons to pull the intake back in
   */
  public void retractIntake() {
    intakePiston.retract();
  }
}
