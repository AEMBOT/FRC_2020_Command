package frc.robot.shuffleboard.utilities;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.shuffleboard.WaypointData;

/**
 * Class containing static methods to allow for easy interaction with the waypoint plotter
 */
public class RAMSETEPlottingManager {

    /**
     * Add a waypoint to the chart.
     */
    public static void addWaypoint(double realX, double realY, double calculatedX, double calculatedY){
        SmartDashboard.putData("RAMSETE-Chart", new WaypointData(realX, realY, calculatedX, calculatedY, false));
    }

    /**
     * Reset the chart in-preparation for a new series of data points.
     */
    public static void resetChart(){
        SmartDashboard.putData("RAMSETE-Chart", new WaypointData(0, 0, 0, 0, true));
    }
}
