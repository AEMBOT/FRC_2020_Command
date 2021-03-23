// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.hardware.sensors;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.geometry.Rotation2d;

import static org.mockito.Mockito.mock;

import com.kauailabs.navx.frc.AHRS;

public class NavX{
    public static NavX instance;

    private AHRS ahrs;

    // Keeps track of the current running heading
    private double lastHeading;

    // Running heading accounting for sign
    private double runningHeading;

    /**
     * Creates a private constructor so that this class cannot be created more than
     * once
     */
    private NavX() {
        if(RobotBase.isReal()){
            try {ahrs = new AHRS(SPI.Port.kMXP);} 
            catch (RuntimeException ex) {}   
        }
        else{
            ahrs = mock(AHRS.class);
        }
       
        // Init the current heading to the current total
        lastHeading = getAngle();
    }

    /**
     * Mainly used in test cases and to reset the navX heading
     */
    public void resetLastHeading(){
        lastHeading = getAngle();
    }

    /**
     * Creates a global reference to the singular NavX
     */
    public static NavX get() {
        
            if (instance == null) {
                instance = new NavX();
            }
            return instance;
    
       
    }

    /**
     * Gets the yaw axis or for all intensive purposes the robot heading purely
     * based on the gyro scope inside (0-180, -180-0)
     * 
     * @return the robot heading
     */
    public float getYaw() {
        return ahrs.getYaw();
    }

    /**
     * Get the correct yaw to 0-360
     * 
     * @return the correct yaw
     */
    public double getCorrectedHeading() {
        double yaw = ahrs.getYaw();
        if (yaw >= 0) {
            return yaw;
        } else {
            return (360 - Math.abs(yaw));
        }
    }

    public double getEdgeCaseAngle(double setpoint) {
        return (((((setpoint - getCorrectedHeading()) + 180) + 360) % 360) - 180);
    }

    /**
     * This method can't truly be reset and will return a heading based of the gyro
     * and the compass (0-360)
     * 
     * @return
     */
    public float getFusedHeading() {
        return ahrs.getFusedHeading();
    }

    /**
     * Reset the yaw axis on the robot
     */
    public void resetYaw() {
        ahrs.zeroYaw();
    }

    /**
     * !Warning! The value may not be accurate due to EMI from the motors Gets the
     * value from the compass in the NavX
     * 
     * @return compass heading
     */
    public float getCompassHeading() {
        return ahrs.getCompassHeading();
    }

    /**
     * Used to reset the robot heading on the NavX
     */
    public void reset() {
        ahrs.reset();
    }

    /**
     * Determines whether or not the robot is moving
     * 
     * @return whether or not the robot is moving
     */
    public boolean isMoving() {
        return ahrs.isMoving();
    }

    /**
     * Determines whether or not the robot is rotating
     * 
     * @return whether or not the robot is rotating
     */
    public boolean isRotating() {
        return ahrs.isRotating();
    }

    /**
     * Return the current state of the NavX calibration
     * 
     * @return whether or not the robot is actively calibrating
     */
    public boolean isCalibrating() {
        return ahrs.isCalibrating();
    }

    /**
     * Get the current continuous angle such that 360 passes to 361 seamlessly 
     * 
     * 
     * @return
     */
    public double getAngleWeird(){

        // Get the current total cumulative angle
        double currentHeading = getAngle();
        double currentRate = getRate();

        // Find the difference between the last heading and now
        double headingDelta = Math.abs(currentHeading - getLastHeading());

        // Add the current heading difference with the sign copied from the rate of the gyro to the running heading and return that value
        runningHeading += Math.copySign(headingDelta, currentRate);
        lastHeading = currentHeading;
        return runningHeading;
    }

    /**
     * Set the last cumulative heading (used in unit tests mainly)
     * @return
     */
    public double getLastHeading(){
        return lastHeading;
    }

    /**
     * Return the rate at which the NavX is moving
     * 
     * @return rate of change in seconds
     */
    public double getRate() {
        return ahrs.getRate();
    }

    /**
     * Return a continuos angle greater than 360 degrees
     * 
     * @return the total angle
     */
    public double getAngle() {
        return ahrs.getAngle();
    }

    /**
     * Return a continuous rotation2D
     * @return Rotation2d object with the current angle
     */
    public Rotation2d getRotation2d(){
        new Rotation2d();
        return Rotation2d.fromDegrees(getAngle());
    }

    public AHRS getAhrs() {
        return ahrs;
    }
}
