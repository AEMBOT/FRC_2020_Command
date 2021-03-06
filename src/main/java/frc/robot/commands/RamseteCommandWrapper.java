// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.commands.utilities.RamseteCommand;
import frc.robot.hardware.sensors.NavX;
import frc.robot.shuffleboard.utilities.RAMSETEPlottingManager;
import frc.robot.subsystems.DriveTrainSystem;

import static frc.robot.Constants.*;

import java.io.IOException;
import java.nio.file.Path;
import java.util.List;

import com.revrobotics.CANSparkMax.IdleMode;

public class RamseteCommandWrapper extends CommandBase {

  private final DriveTrainSystem m_drive;
  private TrajectoryConfig config;

  private Trajectory trajectory;
  private RamseteCommand ramseteCommand;

   /** Creates a new RamsetCommand. */
   public RamseteCommandWrapper(DriveTrainSystem subsystem, List<Translation2d> waypoints, Pose2d initalPose, Pose2d finalPose) {
    m_drive = subsystem;
    

    // Create voltage constraints
    var autoVoltageConstraint = new DifferentialDriveVoltageConstraint(
        new SimpleMotorFeedforward(kSVolts, kvVoltMetersPerSecond, kaVoltMetersPerSecondSquared), kDriveKinematics, 10);

    // Apply aforementioned constraints
    config = new TrajectoryConfig(kMaxVelocityMetersPerSecond, kMaxAccelerationMetersPerSecondSquared)
        // Obey max speed
        .setKinematics(kDriveKinematics)
        // Apply the voltage constraint
        .addConstraint(autoVoltageConstraint);

    // Generate the trajectory from given information
    trajectory = TrajectoryGenerator.generateTrajectory(initalPose, waypoints, finalPose, config);
  }

  /** Creates a new RamsetCommand. */
  public RamseteCommandWrapper(DriveTrainSystem subsystem, String pathName) {
    m_drive = subsystem;

    // Generate the trajectory from given information
    Path filePath = Filesystem.getDeployDirectory().toPath().resolve("paths/" + pathName + ".wpilib.json");
    try {
      trajectory = TrajectoryUtil.fromPathweaverJson(filePath);
    } catch (IOException e) {
      System.out.println("Unable to open path file");
    }
  }


  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    RAMSETEPlottingManager.resetChart();
    // Create the ramsete command that will run the trajectory
    ramseteCommand = new RamseteCommand(
      trajectory,
      m_drive::getPose,
      new RamseteController(kRamseteB, kRamseteZeta),
      new SimpleMotorFeedforward(kSVolts,
                                 kvVoltMetersPerSecond,
                                 kaVoltMetersPerSecondSquared),
      kDriveKinematics,
      m_drive::getWheelSpeeds,
      new PIDController(kPDriveVal, 0, 0),
      new PIDController(kPDriveVal, 0, 0),
      m_drive::tankDriveVolts,
      m_drive
    );
    
    
    m_drive.resetNavX();
    // Reset the odometry pod to the initial pose of the trajectory
    m_drive.resetOdometry(trajectory.getInitialPose());
    ramseteCommand.schedule();
  }
  
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    
    return ramseteCommand.isFinished();
  }
}
