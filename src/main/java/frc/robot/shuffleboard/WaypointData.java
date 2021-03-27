package frc.robot.shuffleboard;

import edu.wpi.first.wpilibj.Sendable;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder;

public class WaypointData implements Sendable{
    private double realX, realY, calculatedX, calculatedY;
    private boolean resetChart;
    
    public WaypointData(double realX, double realY, double calculatedX, double calculatedY, boolean resetChart){
        this.realX = realX;
        this.realY = realY;
        this.calculatedX = calculatedX;
        this.calculatedY = calculatedY;
        this.resetChart = resetChart;
    }


    public void initSendable(SendableBuilder builder){
        builder.setSmartDashboardType("WaypointData");
        builder.addDoubleProperty("realX", () -> realX, null);
        builder.addDoubleProperty("realY", () -> realY, null);
        builder.addDoubleProperty("calculatedX", () -> calculatedX, null);
        builder.addDoubleProperty("calculatedY", () -> calculatedY, null);
        builder.addBooleanProperty("resetChart", () -> resetChart, null);
        
    }

    
}
