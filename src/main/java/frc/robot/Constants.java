// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;

/**
 * General robot constants
 */
public final class Constants {
    // Number of ticks per one revolution of the wheel, 8192 is based on a REV
    // through bore encoders
    public static final int PULSES_PER_REV = 2048;


    // Calculate the circumference of an 8in pneumatic wheel
    public static final double WHEEL_CIRCUMFERENCE = (Math.PI * 0.2032);

    /**
     * Robot Characterization Link:
     * https://docs.wpilib.org/en/latest/docs/software/wpilib-tools/robot-characterization/introduction.html#installing-and-launching-the-toolsuite
     * The following section of variables is specifically for use with the S-Curve Trajectories / RAMSETE
     */

    // The scale factor required to convert encoder pulses into a useable value, divide by 4 because of how WPI handles reading encoder pulses
    public static final double kEncoderDistancePerPulse = WHEEL_CIRCUMFERENCE / PULSES_PER_REV;

    // The maximum voltage the drive motors can draw during this
    public static final double kMaxUsableVoltage = 10;

    // Weather or not the gyro is flipped
    public static final boolean kGyroReversed = true;

    // Voltage for static friction velocity and acceleration
    public static final double kSVolts = 0.197;
    public static final double kvVoltMetersPerSecond = 1.6; // 1.6 is functional , 1.3 results in pretty graph, but not right position
    public static final double kaVoltMetersPerSecondSquared = 0.329;

    // PID values (Only P is required for velocity)
    public static final double kPDriveVal = 2.6;

    // Kinematic information about our robot
    public static final double kTrackWidthMeters = 0.501360651651025;
    
    public static final DifferentialDriveKinematics kDriveKinematics = new DifferentialDriveKinematics(
            kTrackWidthMeters);

    // Sets values for the speed at which we will reach the max velocity and what
    // the max velocity
    public static final double kMaxVelocityMetersPerSecond = 2;
    public static final double kMaxAccelerationMetersPerSecondSquared = 1;

    // Reasonable baseline values for a RAMSETE follower in units of meters and
    // seconds
    public static final double kRamseteB = 2;
    public static final double kRamseteZeta = 0.7;
}
