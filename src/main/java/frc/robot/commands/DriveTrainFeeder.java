// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.io.FileNotFoundException;
import java.nio.file.Path;
import java.util.Scanner;

import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrainSystem;

/**
 * Feeds pre-recorded motor powers into the drive train
 */
public class DriveTrainFeeder extends CommandBase {

  // Drive train
  private DriveTrainSystem m_drive;

  // Name of the input data name
  private String fileName;
  private Scanner fileReader;

  private boolean isFinished = false;

  /** Creates a new DriveTrainFeeder. */
  public DriveTrainFeeder(DriveTrainSystem subsystem, String inputFileName) {
    m_drive = subsystem;
    fileName = inputFileName;
    addRequirements(m_drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // Generate the trajectory from given information
    Path filePath = Filesystem.getDeployDirectory().toPath().resolve("input/" + fileName + ".csv");
    try {
      fileReader = new Scanner(filePath.toFile());
      fileReader.nextLine();
    } catch (FileNotFoundException e) {
    
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(fileReader.hasNextLine()){
      String[] powerStr = fileReader.nextLine().trim().split(",");
      m_drive.tankDrive(Double.parseDouble(powerStr[0]), Double.parseDouble(powerStr[1]));
    }
    else{
      isFinished = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drive.tankDrive(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isFinished;
  }
}
