// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotMap;

public class DriveTrainSystem extends SubsystemBase {

  // Drive train speed controller variables
  private SpeedControllerGroup leftSide;
  private SpeedControllerGroup rightSide;

  // Local instance of the Differential Drive class
  private DifferentialDrive diffDrive;

  // Create variables for the through bore encoders on either side of the drive
  // train
  private Encoder leftSideEncoder;
  private Encoder rightSideEncoder;

  // Individual Left Side Motors
  private CANSparkMax LeftFrontMotor;
  private CANSparkMax LeftMiddleMotor;
  private CANSparkMax LeftBackMotor;

  private CANSparkMax[] leftMotorsArray;

  // Individual Right Side Motors
  private CANSparkMax RightFrontMotor;
  private CANSparkMax RightMiddleMotor;
  private CANSparkMax RightBackMotor;

  private CANSparkMax[] rightMotorsArray;

  /** Creates a new DriveTrainSystem. */
  public DriveTrainSystem() {
    // Constructs the motors and adds them to speed controller groups
    createMotors();

    // Constructs the encoders
    createEncoders();
  }

  /**
   * Intermediate used to construct the motors
   */
  private void createMotors() {

    // Create the individual motors for the left side to add to the
    // SpeedControllerGroup
    LeftFrontMotor = new CANSparkMax(RobotMap.LeftFrontMotor, MotorType.kBrushless);
    LeftMiddleMotor = new CANSparkMax(RobotMap.LeftMiddleMotor, MotorType.kBrushless);
    LeftBackMotor = new CANSparkMax(RobotMap.LeftBackMotor, MotorType.kBrushless);

    // Create and add motors to the Left side motor container
    leftMotorsArray = new CANSparkMax[3];
    leftMotorsArray[0] = LeftFrontMotor;
    leftMotorsArray[1] = LeftMiddleMotor;
    leftMotorsArray[2] = LeftBackMotor;

    // Create the individual motors for the right side to add to the
    // SpeedControllerGroup
    RightFrontMotor = new CANSparkMax(RobotMap.RightFrontMotor, MotorType.kBrushless);
    RightMiddleMotor = new CANSparkMax(RobotMap.RightMiddleMotor, MotorType.kBrushless);
    RightBackMotor = new CANSparkMax(RobotMap.RightBackMotor, MotorType.kBrushless);

    // Create an array to hold the right side motors
    rightMotorsArray = new CANSparkMax[3];
    rightMotorsArray[0] = RightFrontMotor;
    rightMotorsArray[1] = RightMiddleMotor;
    rightMotorsArray[2] = RightBackMotor;

    // SpeedControllerGroups that hold all meaningful
    leftSide = new SpeedControllerGroup(LeftFrontMotor, LeftMiddleMotor, LeftBackMotor);
    rightSide = new SpeedControllerGroup(RightFrontMotor, RightMiddleMotor, RightBackMotor);

    // Flip the forward direction of the drive train
    leftSide.setInverted(true);

    // Create the differential robot control system
    // NOTE: Right and Left are flipped to account for weird inverted values that I
    // dont want to change because autonmous works
    diffDrive = new DifferentialDrive(leftSide, rightSide);

    diffDrive.setSafetyEnabled(false);
  }

  /**
   * Constructs the required drive train encoders
   */
  private void createEncoders() {

    // Create the encoders
    leftSideEncoder = new Encoder(RobotMap.LeftSideEncoderA, RobotMap.LeftSideEncoderB);
    rightSideEncoder = new Encoder(RobotMap.RightSideEncoderA, RobotMap.RightSideEncoderB);

    // Flip Encoder values
    leftSideEncoder.setReverseDirection(true);

    // Convert the pulses into usable distances
    leftSideEncoder.setDistancePerPulse(Constants.kEncoderDistancePerPulse);
    rightSideEncoder.setDistancePerPulse(Constants.kEncoderDistancePerPulse);
  }

  /**
   * Wrapper for the differential drive arcade drive
   */
  public void arcadeDrive(double drivePower, double turnPower) {
    diffDrive.arcadeDrive(-drivePower, -turnPower);
  }

  /**
   * Sets the ramp rate to zero seconds
   */
  public void disableOpenRampRate() {
    LeftFrontMotor.setOpenLoopRampRate(0);
    LeftMiddleMotor.setOpenLoopRampRate(0);
    LeftBackMotor.setOpenLoopRampRate(0);

    RightBackMotor.setOpenLoopRampRate(0);
    RightMiddleMotor.setOpenLoopRampRate(0);
    RightFrontMotor.setOpenLoopRampRate(0);
  }

  /**
   * Set the ramp rate on the drive train
   * 
   * @param accelTime the time in seconds it takes to go from 0-100
   */
  public void enableOpenRampRate(double accelTime) {

    // Left Ramp Rate
    LeftFrontMotor.setOpenLoopRampRate(accelTime);
    LeftMiddleMotor.setOpenLoopRampRate(accelTime);
    LeftBackMotor.setOpenLoopRampRate(accelTime);

    // Right Ramp Rate
    RightBackMotor.setOpenLoopRampRate(accelTime);
    RightMiddleMotor.setOpenLoopRampRate(accelTime);
    RightFrontMotor.setOpenLoopRampRate(accelTime);
  }

  /**
   * Wrapper for the tank drive method in the diff drive class
   */
  public void tankDrive(double leftPower, double rightPower) {

    diffDrive.tankDrive(leftPower, rightPower);
  }

  /**
   * Get the left-side's speed controller group
   * 
   * @return leftSide
   */
  public SpeedControllerGroup getLeftSide() {
    return leftSide;
  }

  /**
   * Get the right-side's speed controller group
   * 
   * @return rightSide
   */
  public SpeedControllerGroup getRightSide() {
    return rightSide;
  }

  /**
   * Get the value from the left side encoder
   */
  public int getLeftSideEncoderPosition() {
    return leftSideEncoder.get();
  }

  /**
   * Get a reference to the encoder on the left side
   * 
   * @return leftSideEncoder
   */
  public Encoder getLeftSideEncoder() {
    return leftSideEncoder;
  }

  /**
   * Get a reference to the encoder on the right side
   * 
   * @return rightSideEncoder
   */
  public Encoder getRightSideEncoder() {
    return rightSideEncoder;
  }

  /**
   * Get the value from the right side encoder
   */
  public int getRightSideEncoderPosition() {
    return rightSideEncoder.get();
  }

  /**
   * Gets the average position between the two sides
   * 
   * @return the averaged position
   */
  public int getAveragePosition() {
    return ((getLeftSideEncoderPosition() + getRightSideEncoderPosition()) / 2);
  }

  /**
   * The average distance traveled between both encoders
   * 
   * @return the distance
   */
  public double getAverageEncoderDistance() {
    return ((leftSideEncoder.getDistance() + rightSideEncoder.getDistance()) / 2.0);
  }

  /**
   * Reset both sides encoders
   */
  public void resetEncoders() {
    rightSideEncoder.reset();
    leftSideEncoder.reset();
  }
}
