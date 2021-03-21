// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.hardware.pneumatics.AdvancedCompressor;

import static frc.robot.RobotMap.*;

public class ArcShooter extends SubsystemBase {

  // Flywheel motors
  private CANSparkMax flywheelMotor;
  private CANSparkMax flywheelMotor2;

  private AdvancedCompressor compressor;

  private boolean shooterToggled = false;

  /** Creates a new ArcShooter. */
  public ArcShooter() {

    // Construct both flywheel motor objects
    flywheelMotor = new CANSparkMax(ShooterFlyWheelMotor, MotorType.kBrushless);
    flywheelMotor2 = new CANSparkMax(ShooterFlyWheelMotor2, MotorType.kBrushless);

    // Invert the first motor and have the second motor follow also inverted
    flywheelMotor.setInverted(true);
    flywheelMotor2.follow(flywheelMotor, true);

    // Set the open loop ramp rate, really should be using closed loop but that is
    // currently not important
    flywheelMotor.setOpenLoopRampRate(2.5);
    AdvancedCompressor.get().stopCompressor();
  }

  @Override
  public void periodic() {
      updateDashboard();
  }

  private void updateDashboard(){

    // Wether or not the fly wheel has reached full speed
    SmartDashboard.putBoolean("Fly-Wheel-Speed-Status", isFullSpeed());

    // The current flywheel speed
    SmartDashboard.putNumber("Fly-Wheel-RPM", getFlywheelRPM());
  }

  /**
   * Check if the geared up motor is at max rpm or > than 9000 rpm
   * 
   * @return status of full motor speed
   */
  public boolean isFullSpeed() {
    return Math.abs(getFlywheelRPM()) > 9000;
  }

  /**
   * Get the current flywheel RPM
   * @return
   */
  public double getFlywheelRPM(){
    return flywheelMotor.getEncoder().getVelocity() * 2;
  }

  /**
   * Determines whether or not the shooter is running
   * 
   * @return shooter status
   */
  public boolean isRunning() {
    if (Math.abs(flywheelMotor.get()) > 0.05) {
      return true;
    }
    return false;
  }

  /**
   * Run the shooter motor given a manual power
   */
  public void runShooter(double motorPower) {
    if (motorPower > 0.1) {
      flywheelMotor.set(motorPower);
    } else
      flywheelMotor.set(0);

  }
}
