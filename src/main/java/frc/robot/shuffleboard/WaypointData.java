package frc.robot.shuffleboard;

import edu.wpi.first.wpilibj.Sendable;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder;

public class WaypointData implements Sendable{
    private double x, y;
    private boolean isCalculated;
    
    public WaypointData(double x, double y, boolean isCalculated){
        this.x = x;
        this.y = y;
        this.isCalculated = isCalculated;
    }

    public double getX() {
        return x;
    }
    public boolean isCalculated() {
        return isCalculated;
    }
    public void setCalculated(boolean isCalculated) {
        this.isCalculated = isCalculated;
    }
    public double getY() {
        return y;
    }
    public void setY(double y) {
        this.y = y;
    }
    public void setX(double x) {
        this.x = x;
    }

    public void initSendable(SendableBuilder builder){
        builder.setSmartDashboardType("WaypointData");
        builder.addDoubleProperty("x", () -> x, null);
        builder.addDoubleProperty("y", () -> y, null);
        builder.addBooleanProperty("isCalculated", () -> isCalculated, null);
    }

    
}
