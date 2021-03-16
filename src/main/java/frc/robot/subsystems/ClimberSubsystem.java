// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.hardware.pneumatics.DoublePiston;
import static frc.robot.RobotMap.*;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

public class ClimberSubsystem extends SubsystemBase {

  // Climber pistons
  private DoublePiston climberPiston;

  // Create the climber winches
  private TalonFX rightWinch;
  private TalonFX leftWinch;

  /** Creates a new ClimberSubsystem. */
  public ClimberSubsystem() {
    climberPiston = new DoublePiston(ClimberPistonA, ClimberPistonB);

    // Create the winch motors
    rightWinch = new TalonFX(RightWinchMotor);
    leftWinch = new TalonFX(LefWinchMotor);

    // Invert both the winch motors so that a non altered duty cycle can be passed
    rightWinch.setInverted(InvertType.InvertMotorOutput);
    leftWinch.setInverted(InvertType.InvertMotorOutput);
  }

  /**
   * Run the winch manually down
   * 
   * @param power the power to apply winch
   */
  public void manualWinch(double leftPower, double rightPower) {
    rightWinch.set(ControlMode.PercentOutput, rightPower);
    leftWinch.set(ControlMode.PercentOutput, leftPower);
  }

  /**
   * Deploy the hook
   */
  public void deployClimber() {
    climberPiston.actuate();
  }

  /**
   * Retract the climber
   */
  public void retractClimber() {
    climberPiston.retract();
  }

  /**
   * Runs the left winch individually
   */
  public void runLeftWinch(double power) {
    leftWinch.set(ControlMode.PercentOutput, power);
  }

  /**
   * Run the right winch individually
   */
  public void runRightWinch(double power) {
    rightWinch.set(ControlMode.PercentOutput, power);
  }

}
