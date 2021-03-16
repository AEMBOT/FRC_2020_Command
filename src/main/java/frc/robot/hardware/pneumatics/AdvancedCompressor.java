// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.hardware.pneumatics;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.Timer;

public class AdvancedCompressor {
  // Variable to hold the actual compressor object
  private static Compressor compressor;

  // Variable to hold the value of this class
  private static AdvancedCompressor advancedCompressor;

  /**
   * Construct the actual WPILIB compressor class
   */
  private AdvancedCompressor() {
    compressor = new Compressor();
  }

  /**
   * Get a static reference to the current compressor class
   * 
   * @return the reference to the compressor
   */
  public static AdvancedCompressor get() {
    if (advancedCompressor == null) {
      advancedCompressor = new AdvancedCompressor();
    }

    return advancedCompressor;
  }

  /**
   * Starts running the actual compressor
   */
  public void startCompressor() {
    compressor.start();
  }

  /**
   * Stop running the compressor
   */
  public void stopCompressor() {
    compressor.stop();
  }

  /**
   * When called this method will turn the compressor on for a set amount of time
   * and then turn it back off
   * 
   * @param seconds the time to keep the compressor on
   */
  public void startTimedRecharge(double seconds) {
    startCompressor();
    Timer.delay(seconds);
    stopCompressor();
  }

  /**
   * Return the status of the pressure switch
   * 
   * @return boolean for pressure switch
   */
  public boolean getPressureSwitchStatus() {
    return compressor.getPressureSwitchValue();
  }

  /**
   * Clears all the faults on the PCM/Compressor
   */
  public void clearFaults() {
    compressor.clearAllPCMStickyFaults();
  }

  /**
   * Returns the current current draw from the compressor
   * 
   * @return double value of the current draw
   */
  public double getCurrent() {
    return compressor.getCompressorCurrent();
  }

  /**
   * Runs the compressor until the pressure switch is hit
   */
  public void runUntilFull() {
    if (!getPressureSwitchStatus()) {
      startCompressor();
    } else {
      stopCompressor();
    }
  }

  /**
   * Returns the current status of the compressor
   * 
   * @return boolean showing true / false for on or off
   */
  public boolean getCompressorStatus() {
    return compressor.enabled();
  }

}
