package frc.robot.shuffleboard.utilities;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.shuffleboard.WaypointData;

public class RAMSETEPlottingManager {

    /**
     * Add a waypoint to the chart
     */
    public static void addWaypoint(double realX, double realY, double calculatedX, double calculatedY){
        SmartDashboard.putData("RAMSETE-Chart", new WaypointData(realX, realY, calculatedX, calculatedY, false));
    }

    public static void resetChart(){
        SmartDashboard.putData("RAMSETE-Chart", new WaypointData(0, 0, 0, 0, true));
    }
}
