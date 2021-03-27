// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Map;
import java.util.Random;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.SlewRateLimiter;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotMap;
import frc.robot.hardware.sensors.NavX;
import frc.robot.shuffleboard.WaypointData;

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

  // Allows the driver to ramp the drivetrain up to 1.2 duty cycle in one second
  SlewRateLimiter driveRamp = new SlewRateLimiter(1.2);

  /** Creates a new DriveTrainSystem. */
  public DriveTrainSystem() {
    // Constructs the motors and adds them to speed controller groups
    createMotors();

    // Constructs the encoders and then resets them
    createEncoders();

    m_odometry = new DifferentialDriveOdometry(getHeading());
    resetOdometry(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(0)));
    
    
  }

  @Override
  public void periodic() {
    //Update the odometry information each loop
    m_odometry.update(getHeading(), getLeftEncoderDistance(), getRightEncoderDistance());

    // Update the information displayed on the user dashboard
    updateDashboard();
  }

  private void updateDashboard(){

    // Add both encoders to the dashboard to keep track of speed and position
    SmartDashboard.putData("Left-Side-Encoder", getLeftEncoder());
    SmartDashboard.putData("Right-Side-Encoder", getRightEncoder());

    // Gyro information
    SmartDashboard.putData("Gyro", navX.getAhrs());

    // Average drive train motor temperatures
    SmartDashboard.putNumber("Left-Side-Temperature", getAverageLeftSideTemperature());
    SmartDashboard.putNumber("Right-Side-Temperature", getAverageRightSideTemperature());

    // Average drive train current draw
    SmartDashboard.putNumber("Right-Side-Current-Draw", getAverageRightSideCurrents());
    SmartDashboard.putNumber("Left-Side-Current-Draw", getAverageLeftSideCurrents());
  
    SmartDashboard.putNumber("Odometry-X", getPose().getX());
    SmartDashboard.putNumber("Odometry-Y", getPose().getY());

    SmartDashboard.putNumber("NavX-Rotation", navX.getAngle());

    
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

  /**
   * Command the robots drive train using voltage as opposed to duty cycle
   * @param leftVolts voltage to supply to the left side of the drive train
   * @param rightVolts voltage to supply to the right side of the drive train
   */
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
    m_odometry.resetPosition(pose, getHeading());
  }

  /**
   * Set the brake mode of all the motors on the bot
   */
  public void setBrakeMode(IdleMode mode){
    for(int i=0; i < leftMotorsArray.length; i++){
      leftMotorsArray[i].setIdleMode(mode);
      rightMotorsArray[i].setIdleMode(mode);
    }
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

    // Diff drive 
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

  public void resetNavX(){
    navX.reset();
    navX.resetYaw();
  }

  /**
   * Wrapper for the differential drive arcade drive
   * @param drivePower power to supply to the drive train to move forward/backwards
   * @param turnPower power to supply to the drive train to rotate the robot
   * @param enableRateLimiters enables the rate limiter on the drive for smoother acceleration and less jerk
   */
  public void arcadeDrive(double drivePower, double turnPower, boolean enableRateLimiters) {
    if(enableRateLimiters)
      diffDrive.arcadeDrive(driveRamp.calculate(drivePower), turnPower);
    else
      diffDrive.arcadeDrive(drivePower, turnPower);
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
  public Rotation2d getHeading() {
    return Rotation2d.fromDegrees(Math.IEEEremainder(navX.getAngle(), 360) * (Constants.kGyroReversed ? -1.0 : 1.0));
  }

   /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    return -navX.getRate();
  }

  /**
   * Retrieve the average temperature of the left side of the drive train
   * @return current motor temp. average
   */
  public double getAverageLeftSideTemperature(){
    double sum  = 0;
    for (CANSparkMax motor : leftMotorsArray) {
      sum += motor.getMotorTemperature();
    }
    return (sum / leftMotorsArray.length);
  }

  /**
   * Retrieve the average temperature of the right side of the drive train
   * @return current motor temp. average
   */
  public double getAverageRightSideTemperature(){
    double sum  = 0;
    for (CANSparkMax motor : rightMotorsArray) {
      sum += motor.getMotorTemperature();
    }
    return (sum / rightMotorsArray.length);
  }

  /**
   * Retrieve the average amperage of the left side of the drive train
   * @return current motor amp draw
   */
  public double getAverageLeftSideCurrents(){
    double sum  = 0;
    for (CANSparkMax motor : leftMotorsArray) {
      sum += motor.getOutputCurrent();
    }
    return (sum / leftMotorsArray.length);
  }

  /**
   * Retrieve the average amperage of the right side of the drive train
   * @return current motor amp draw average
   */
  public double getAverageRightSideCurrents(){
    double sum  = 0;
    for (CANSparkMax motor : rightMotorsArray) {
      sum += motor.getOutputCurrent();
    }
    return (sum / rightMotorsArray.length);
  }
}
