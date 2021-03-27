package frc.robot.commands.utilities;

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// NOTE !!!! THIS IS A COPY OF THE ORIGINAL WITH MODIFICATIONS MADE TO ALLOW FOR BETTER LOGGING !!!

import static edu.wpi.first.wpilibj.util.ErrorMessages.requireNonNullParam;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.Trajectory.State;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.shuffleboard.utilities.RAMSETEPlottingManager;
import frc.robot.subsystems.DriveTrainSystem;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.util.function.BiConsumer;
import java.util.function.Supplier;

/**
 * A command that uses a RAMSETE controller ({@link RamseteController}) to follow a trajectory
 * {@link Trajectory} with a differential drive.
 *
 * <p>The command handles trajectory-following, PID calculations, and feedforwards internally. This
 * is intended to be a more-or-less "complete solution" that can be used by teams without a great
 * deal of controls expertise.
 *
 * <p>Advanced teams seeking more flexibility (for example, those who wish to use the onboard PID
 * functionality of a "smart" motor controller) may use the secondary constructor that omits the PID
 * and feedforward functionality, returning only the raw wheel speeds from the RAMSETE controller.
 */
@SuppressWarnings("PMD.TooManyFields")
public class RamseteCommand extends CommandBase {
  private final Timer m_timer = new Timer();
  private final boolean m_usePID;
  private final Trajectory m_trajectory;
  private final Supplier<Pose2d> m_pose;
  private final RamseteController m_follower;
  private final SimpleMotorFeedforward m_feedforward;
  private final DifferentialDriveKinematics m_kinematics;
  private final Supplier<DifferentialDriveWheelSpeeds> m_speeds;
  private final PIDController m_leftController;
  private final PIDController m_rightController;
  private final BiConsumer<Double, Double> m_output;
  private DifferentialDriveWheelSpeeds m_prevSpeeds;
  private double m_prevTime;

  private State currentRobotState;
  private FileWriter myWriter;
  private DriveTrainSystem m_drive;

  /**
   * Constructs a new RamseteCommand that, when executed, will follow the provided trajectory. PID
   * control and feedforward are handled internally, and outputs are scaled -12 to 12 representing
   * units of volts.
   *
   * <p>Note: The controller will *not* set the outputVolts to zero upon completion of the path -
   * this is left to the user, since it is not appropriate for paths with nonstationary endstates.
   *
   * @param trajectory The trajectory to follow.
   * @param pose A function that supplies the robot pose - use one of the odometry classes to
   *     provide this.
   * @param controller The RAMSETE controller used to follow the trajectory.
   * @param feedforward The feedforward to use for the drive.
   * @param kinematics The kinematics for the robot drivetrain.
   * @param wheelSpeeds A function that supplies the speeds of the left and right sides of the robot
   *     drive.
   * @param leftController The PIDController for the left side of the robot drive.
   * @param rightController The PIDController for the right side of the robot drive.
   * @param outputVolts A function that consumes the computed left and right outputs (in volts) for
   *     the robot drive.
   * @param requirements The subsystems to require.
   */
  @SuppressWarnings("PMD.ExcessiveParameterList")
  public RamseteCommand(
      Trajectory trajectory,
      Supplier<Pose2d> pose,
      RamseteController controller,
      SimpleMotorFeedforward feedforward,
      DifferentialDriveKinematics kinematics,
      Supplier<DifferentialDriveWheelSpeeds> wheelSpeeds,
      PIDController leftController,
      PIDController rightController,
      BiConsumer<Double, Double> outputVolts,
      Subsystem... requirements) {
    m_trajectory = requireNonNullParam(trajectory, "trajectory", "RamseteCommand");
    m_pose = requireNonNullParam(pose, "pose", "RamseteCommand");
    m_follower = requireNonNullParam(controller, "controller", "RamseteCommand");
    m_feedforward = feedforward;
    m_kinematics = requireNonNullParam(kinematics, "kinematics", "RamseteCommand");
    m_speeds = requireNonNullParam(wheelSpeeds, "wheelSpeeds", "RamseteCommand");
    m_leftController = requireNonNullParam(leftController, "leftController", "RamseteCommand");
    m_rightController = requireNonNullParam(rightController, "rightController", "RamseteCommand");
    m_output = requireNonNullParam(outputVolts, "outputVolts", "RamseteCommand");

    m_usePID = true;

    addRequirements(requirements);
  }

  /**
   * Constructs a new RamseteCommand that, when executed, will follow the provided trajectory.
   * Performs no PID control and calculates no feedforwards; outputs are the raw wheel speeds from
   * the RAMSETE controller, and will need to be converted into a usable form by the user.
   *
   * @param trajectory The trajectory to follow.
   * @param pose A function that supplies the robot pose - use one of the odometry classes to
   *     provide this.
   * @param follower The RAMSETE follower used to follow the trajectory.
   * @param kinematics The kinematics for the robot drivetrain.
   * @param outputMetersPerSecond A function that consumes the computed left and right wheel speeds.
   * @param requirements The subsystems to require.
   */
  public RamseteCommand(
      Trajectory trajectory,
      Supplier<Pose2d> pose,
      RamseteController follower,
      DifferentialDriveKinematics kinematics,
      BiConsumer<Double, Double> outputMetersPerSecond,
      Subsystem... requirements) {
    m_trajectory = requireNonNullParam(trajectory, "trajectory", "RamseteCommand");
    m_pose = requireNonNullParam(pose, "pose", "RamseteCommand");
    m_follower = requireNonNullParam(follower, "follower", "RamseteCommand");
    m_kinematics = requireNonNullParam(kinematics, "kinematics", "RamseteCommand");
    m_output = requireNonNullParam(outputMetersPerSecond, "output", "RamseteCommand");

    m_feedforward = null;
    m_speeds = null;
    m_leftController = null;
    m_rightController = null;

    m_usePID = false;

    addRequirements(requirements);
  }

  @Override
  public void initialize() {

    // Reset chart before running the RAMSETE command
    RAMSETEPlottingManager.resetChart();
    
    m_drive = (DriveTrainSystem)getRequirements().toArray()[0];
    try {
        // Delete the file and create a new one
        new File("/home/lvuser/OutputData.csv").delete();
        myWriter = new FileWriter("/home/lvuser/OutputData.csv");
        myWriter.write("Real X, Real Y, Expected X, Expected Y");
        
    } catch (IOException e) {
        System.out.println("Failed to write to RAMSETE output");
        e.printStackTrace();
    }

    m_prevTime = -1;
    var initialState = m_trajectory.sample(0);
    m_prevSpeeds =
        m_kinematics.toWheelSpeeds(
            new ChassisSpeeds(
                initialState.velocityMetersPerSecond,
                0,
                initialState.curvatureRadPerMeter * initialState.velocityMetersPerSecond));
    m_timer.reset();
    m_timer.start();
    if (m_usePID) {
      m_leftController.reset();
      m_rightController.reset();
    }
  }

  @Override
  public void execute() {
    double curTime = m_timer.get();
    double dt = curTime - m_prevTime;

    if (m_prevTime < 0) {
      m_output.accept(0.0, 0.0);
      m_prevTime = curTime;
      return;
    }

    currentRobotState = m_trajectory.sample(curTime);

    // Write the Real robot pose as well as the precalculate pose to a file
    try {
        myWriter.write(
          Units.metersToFeet(getRealRobotPose().getX()) + "," + 
          Units.metersToFeet(getRealRobotPose().getY()) + "," +
          Units.metersToFeet(getCurrentCalculatedPose().getX()) + "," + 
          Units.metersToFeet(getCurrentCalculatedPose().getY()) + "\n"
        );

        // Plot the points in realtime on the RAMSETE tracker
        RAMSETEPlottingManager.addWaypoint(getRealRobotPose().getX(), getRealRobotPose().getY(), getCurrentCalculatedPose().getX(), getCurrentCalculatedPose().getY());

    } catch (IOException e) {
        System.out.println("Failed to append real/expected values");
        e.printStackTrace();
    }

    var targetWheelSpeeds =
        m_kinematics.toWheelSpeeds(
            m_follower.calculate(m_pose.get(), m_trajectory.sample(curTime)));

    var leftSpeedSetpoint = targetWheelSpeeds.leftMetersPerSecond;
    var rightSpeedSetpoint = targetWheelSpeeds.rightMetersPerSecond;

    double leftOutput;
    double rightOutput;

    if (m_usePID) {
      double leftFeedforward =
          m_feedforward.calculate(
              leftSpeedSetpoint, (leftSpeedSetpoint - m_prevSpeeds.leftMetersPerSecond) / dt);

      double rightFeedforward =
          m_feedforward.calculate(
              rightSpeedSetpoint, (rightSpeedSetpoint - m_prevSpeeds.rightMetersPerSecond) / dt);

      leftOutput =
          leftFeedforward
              + m_leftController.calculate(m_speeds.get().leftMetersPerSecond, leftSpeedSetpoint);

      rightOutput =
          rightFeedforward
              + m_rightController.calculate(
                  m_speeds.get().rightMetersPerSecond, rightSpeedSetpoint);
    } else {
      leftOutput = leftSpeedSetpoint;
      rightOutput = rightSpeedSetpoint;
    }

    m_output.accept(leftOutput, rightOutput);
    m_prevSpeeds = targetWheelSpeeds;
    m_prevTime = curTime;
  }

  /**
   * Get the calculate current robot pose
   * @return the expected robot state at the current time
   */
  public Pose2d getCurrentCalculatedPose(){
      return currentRobotState.poseMeters;
  }

  public Pose2d getRealRobotPose(){
    return m_drive.getPose();
  }

  @Override
  public void end(boolean interrupted) {
    try {
        myWriter.close();
    } catch (IOException e) {
        System.out.println("Failed to close File writer");
        e.printStackTrace();
    }
    m_timer.stop();
  }

  @Override
  public boolean isFinished() {
    return m_timer.hasElapsed(m_trajectory.getTotalTimeSeconds());
  }
}

