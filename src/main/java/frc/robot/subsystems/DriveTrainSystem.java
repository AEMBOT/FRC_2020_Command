// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotMap;
import frc.robot.hardware.sensors.NavX;

public class DriveTrainSystem extends SubsystemBase {

  // Drive train speed controller variables
  private SpeedControllerGroup leftSide;
  private SpeedControllerGroup rightSide;

  // Local instance of the Differential Drive class
  private DifferentialDrive diffDrive;

  private final NavX navX = NavX.get();

  private final DifferentialDriveOdometry m_odometry;

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

    // Constructs the encoders and then resets them
    createEncoders();

    m_odometry = new DifferentialDriveOdometry(navX.getRotation2d());
  }

  @Override
  public void periodic() {
    //Update the odometry information each loop
    m_odometry.update(navX.getRotation2d(), getLeftEncoderDistance(), getRightEncoderDistance());
  }

  /**
   * Get the robots current field pose
   * @return pose information about the robot
   */
  public Pose2d getPose(){
    return m_odometry.getPoseMeters();
  }

  /**
   * Get the wheel speeds of the differential drive train
   * @return wheel speeds in m/s
   */
  public DifferentialDriveWheelSpeeds getWheelSpeeds(){
    return new DifferentialDriveWheelSpeeds(getLeftRate(), getRightRate());
  }

  public void tankDriveVolts(double leftVolts, double rightVolts){
    leftSide.setVoltage(leftVolts);
    rightSide.setVoltage(-rightVolts);
    diffDrive.feed();
  }
  
  /**
   * Get the average distance of the two encoders
   * @return the average of the two encoder readings
   */
  public double getAverageEncoderDistance(){
    return (getLeftEncoderDistance() + getRightEncoderDistance()) / 2.0;
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose){
    resetEncoders();
    m_odometry.resetPosition(pose, navX.getRotation2d());
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

    resetEncoders();
  }

  /**
   * Wrapper for the differential drive arcade drive
   */
  public void arcadeDrive(double drivePower, double turnPower) {
    diffDrive.arcadeDrive(-turnPower, drivePower);
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
   * Get the encoder values of the left side converted to meters
   * @return distance driven
   */
  public double getLeftEncoderDistance(){
    return leftSideEncoder.getDistance();
  }

  /**
   * Get the encoder values of the right side converted to meters
   * @return distance driven
   */
  public double getRightEncoderDistance(){
    return rightSideEncoder.getDistance();
  }

  /**
   * Get the rate of the left side encoders
   * @return rate in m/s
   */
  public double getLeftRate(){
    return leftSideEncoder.getRate();
  }

  /**
   * Get the rate of the right side encoders
   * @return rate in m/s
   */
  public double getRightRate(){
    return rightSideEncoder.getRate();
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
   * Reset both sides encoders
   */
  public void resetEncoders() {
    rightSideEncoder.reset();
    leftSideEncoder.reset();
  }

  /**
   * Get a reference to the left encoder object
   * @return the left encoder object
   */
  public Encoder getLeftEncoder(){
    return leftSideEncoder;
  }

  /**
   * Get a reference to the right encoder object
   * @return the right encoder object
   */
  public Encoder getRightEncoder(){
    return rightSideEncoder;
  }

  /**
   * Sets the max output of the drive.  Useful for scaling the drive to drive more slowly.
   *
   * @param maxOutput the maximum output to which the drive will be constrained
   */
  public void setMaxOutput(double maxOutput) {
    diffDrive.setMaxOutput(maxOutput);
  }

   /**
   * Zeroes the heading of the robot.
   */
  public void zeroHeading() {
    navX.reset();
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  public double getHeading() {
    return navX.getRotation2d().getDegrees();
  }

   /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    return -navX.getRate();
  }
}
