// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.hardware.sensors.NavX;
import frc.robot.hardware.vision.Limelight;
import frc.robot.subsystems.DriveTrainSystem;

import static frc.robot.Constants.*;

/**
 * Command to align the robot to vision tape on the target
 */
public class LimelightAlignment extends CommandBase {
  // The rate at which the motion profile will update
  private final double profileUpdateRate = 0.02;

  private final Limelight m_limelight;
  private final DriveTrainSystem m_drive;
  private final NavX m_navX;

  // The goal to reach of the Profile
  private double profileGoal;
  private final TrapezoidProfile.Constraints m_constraints;
  private final ProfiledPIDController m_controller;

  private boolean finished = false;
  
  /** Creates a new LimelightAlignment. */
  public LimelightAlignment(Limelight limeSub, DriveTrainSystem driveSub, NavX navX) {
    m_limelight = limeSub;
    m_drive = driveSub;
    m_navX = navX;

    // Physical constraints of the profile
    m_constraints = new TrapezoidProfile.Constraints(kMaxVelocityMetersPerSecond, kMaxAccelerationMetersPerSecondSquared);

    // Motion profile controller itself
    m_controller = new ProfiledPIDController(kPDriveVal, 0.0, 0.7, m_constraints, profileUpdateRate);

    // Number of degrees the loop can be off to be deemed acceptable
    m_controller.setTolerance(0.01);

    // Require the limelight subsystem and the drive train subsystem
    addRequirements(m_limelight, m_drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // Stop all drive train movement before alignment begins
    m_drive.tankDriveVolts(0, 0);

    // If no targets were detected stop the command from even starting
    if(!m_limelight.getValidTarget()){
      finished = true;
    }

    // If a target is found try to set the offset equal to a value equivalent to that of a normal gyro reading
    else{
      
      // Set the goal of the profile by adding the offset be that positive or negative to get the goal gyro angle
      profileGoal = m_navX.getAngle() + m_limelight.getX();
      m_controller.setGoal(profileGoal);

    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // If at the final goal end the command, if not try to feed data to make it reach that goal
    if(!m_controller.atGoal()){
      m_drive.arcadeDrive(0, m_controller.calculate(m_navX.getAngle()), false);
    }
    else{
      finished = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // Stop all drive train movement when alignment is complete / ends
    m_drive.tankDriveVolts(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return finished;
  }
}
