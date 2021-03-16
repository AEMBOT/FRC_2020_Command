// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.ballSystem;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import static frc.robot.RobotMap.*;

public class Indexer extends SubsystemBase {
  // Used on the indexer motors to index the ball
  private final CANSparkMax frontIndexerMotor;
  private final CANSparkMax backIndexerMotor;

  /** Creates a new Indexer. */
  public Indexer() {

    // Create front/back indexer motors
    frontIndexerMotor = new CANSparkMax(FrontIndexerMotor, MotorType.kBrushless);
    backIndexerMotor = new CANSparkMax(BackIndexerMotor, MotorType.kBrushless);

    // Set the front indexer motor to have a ramp rate of 1/2 a second and have the
    // backIndexer simply follow the front
    frontIndexerMotor.setOpenLoopRampRate(0.5);
    backIndexerMotor.follow(frontIndexerMotor, true);
  }

  /***
   * Run the indexer motors at a set speed
   * 
   * @param power the duty cycle to run the motors at
   */
  public void runIndexers(double power) {
    frontIndexerMotor.set(power);
  }

  public double getFrontIndexerCurrent() {
    return frontIndexerMotor.getOutputCurrent();
  }

  public double getBackIndexerCurrent() {
    return backIndexerMotor.getOutputCurrent();
  }
}
