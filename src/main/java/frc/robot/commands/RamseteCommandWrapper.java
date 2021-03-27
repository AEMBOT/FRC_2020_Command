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

import com.revrobotics.CANSparkMax.IdleMode;

public class RamseteCommandWrapper extends CommandBase {

  private final DriveTrainSystem m_drive;

  private Trajectory trajectory;
  private RamseteCommand ramseteCommand;

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

    NavX.get().resetLastHeading();
    m_drive.setBrakeMode(IdleMode.kBrake);
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
    // When the auto path is over set the drive mode back to coast
    m_drive.setBrakeMode(IdleMode.kCoast);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    
    return ramseteCommand.isFinished();
  }
}
