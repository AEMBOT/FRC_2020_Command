// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static frc.robot.Constants.*;

import java.util.List;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.commands.LimelightAlignment;
import frc.robot.commands.RamseteCommandWrapper;
import frc.robot.hardware.sensors.NavX;
import frc.robot.hardware.vision.Limelight;
import frc.robot.subsystems.ArcShooter;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.DriveTrainSystem;
import frc.robot.subsystems.ballSystem.Hopper;
import frc.robot.subsystems.ballSystem.Indexer;
import frc.robot.subsystems.ballSystem.Intake;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final ArcShooter m_arcShooter = new ArcShooter();

  // Get the static instance of the navX
  private final NavX m_navX = NavX.get();

  // Hopper general subsystems
  private final Hopper m_hopperSubsystem = new Hopper();
  private final Indexer m_indexerSubsystem = new Indexer();
  private final Intake m_intakeSubsystem = new Intake();

  private final Limelight m_limelight = new Limelight();

  private final ClimberSubsystem m_ClimberSubsystem = new ClimberSubsystem();

  // Drive train
  private final DriveTrainSystem m_driveTrain = new DriveTrainSystem();

  // Limelight Alignment command
  private final LimelightAlignment m_alignmentCommand = new LimelightAlignment(m_limelight, m_driveTrain, m_navX);

  // Construct both controllers
  private XboxController primaryController = new XboxController(0);
  private XboxController secondaryController = new XboxController(1);

  // region Auto Commands

  private final Command testRamsetePathCommand = new RamseteCommandWrapper(m_driveTrain, Paths.getTestPoints(),
      new Pose2d(0, 0, new Rotation2d(0)), new Pose2d(1, 0, Rotation2d.fromDegrees(0)))
          .andThen(() -> m_driveTrain.tankDriveVolts(0, 0));

  private final Command barrelRamsetePathCommand = new RamseteCommandWrapper(m_driveTrain, Paths.getBarrelPoints(),
      new Pose2d(0.76886, -2.52096, new Rotation2d(0)), new Pose2d(0.91970, -1.86537, Rotation2d.fromDegrees(180)))
          .andThen(() -> m_driveTrain.tankDriveVolts(0, 0));

  private final Command bounceRamsetePathCommand = new RamseteCommandWrapper(m_driveTrain, Paths.getBouncePoints(),
      new Pose2d(1.23879, -2.28890, new Rotation2d(0)),
      new Pose2d(8.357457973291437, -2.3817277176868688, new Rotation2d(0)))
          .andThen(() -> m_driveTrain.tankDriveVolts(0, 0));

  private final Command slalomRamsetePathCommand = new RamseteCommandWrapper(m_driveTrain, Paths.getSlalomPoints(),
      new Pose2d(1.28521, -3.93077, new Rotation2d(0)), new Pose2d(0.60641, -2.15546, Rotation2d.fromDegrees(180)))
          .andThen(() -> m_driveTrain.tankDriveVolts(0, 0));

  SendableChooser<Command> autoChooser = new SendableChooser<>();
  // endregion

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {

    // Set the default option to the 1 meter drive
    autoChooser.setDefaultOption("Test Path", testRamsetePathCommand);

    // Add the remaining complex paths as options
    autoChooser.addOption("Barrel Path", barrelRamsetePathCommand);
    autoChooser.addOption("Bounce Path", bounceRamsetePathCommand);
    autoChooser.addOption("Slalom Path", slalomRamsetePathCommand);

    // Add the auto selector to the dashboard
    SmartDashboard.putData("Auto-Options", autoChooser);

    // If no other command is using the drive train subsystem allow it to be
    // controllable with the joysticks
    m_driveTrain.setDefaultCommand(new RunCommand(
        () -> m_driveTrain.arcadeDrive(-primaryController.getY(Hand.kLeft), primaryController.getX(Hand.kRight), true),
        m_driveTrain));

    // Set the default command for the climber to be controlling the winch motors
    m_ClimberSubsystem.setDefaultCommand(new RunCommand(() -> m_ClimberSubsystem
        .manualWinch(secondaryController.getY(Hand.kLeft), secondaryController.getY(Hand.kRight)), m_ClimberSubsystem));

    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Define button mappings in this method
   */
  private void configureButtonBindings() {

    // Run the shooter while the left bumper is held down
    new JoystickButton(primaryController, XboxController.Button.kBumperLeft.value).whileHeld(
        new StartEndCommand(() -> m_arcShooter.runShooter(1), () -> m_arcShooter.runShooter(0), m_arcShooter));

    // When A is pressed start the alignment command if it is not already running,
    // if it is cancel it
    new JoystickButton(primaryController, XboxController.Button.kA.value).whenPressed(new InstantCommand(() -> {
      if (m_alignmentCommand.isScheduled()) {
        m_alignmentCommand.cancel();
      } else {
        m_alignmentCommand.schedule();
      }
    }));

    // Run the indexer and the indexer belts while X is pressed
    new JoystickButton(secondaryController, XboxController.Button.kX.value).whileHeld(new StartEndCommand(() -> {
      m_indexerSubsystem.runIndexers(1);
      m_hopperSubsystem.runBelts(0.75);
    }, () -> {
      m_indexerSubsystem.runIndexers(0);
      m_hopperSubsystem.runBelts(0);
    }, m_indexerSubsystem, m_hopperSubsystem));

    // Intake Extension
    new JoystickButton(secondaryController, XboxController.Button.kA.value)
        .whenPressed((new InstantCommand(m_intakeSubsystem::extendIntake, m_intakeSubsystem)));

    // Intake retraction
    new JoystickButton(secondaryController, XboxController.Button.kB.value)
        .whenPressed((new InstantCommand(m_intakeSubsystem::retractIntake, m_intakeSubsystem)));

    // Deploy climber when D-pad right and Y are pushed at the same time
    new JoystickButton(secondaryController, XboxController.Button.kY.value).and(new POVButton(secondaryController, 90))
        .whenActive(m_ClimberSubsystem::deployClimber);

    // Retract the climber when D-pad left and Y are pressed
    new JoystickButton(secondaryController, XboxController.Button.kY.value).and(new POVButton(secondaryController, 270))
        .whenActive(m_ClimberSubsystem::retractClimber);

    // Run the intake in reverse when up is pushed on the D-pad
    new POVButton(secondaryController, 0).whenPressed(() -> m_intakeSubsystem.runIntake(-0.6))
        .whenReleased(() -> m_intakeSubsystem.runIntake(0));

    // Run intake "forward" / into the robot when the down arrow on the D-pad is
    // pressed
    new POVButton(secondaryController, 180).whenPressed(() -> m_intakeSubsystem.runIntake(0.6))
        .whenReleased(() -> m_intakeSubsystem.runIntake(0));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
