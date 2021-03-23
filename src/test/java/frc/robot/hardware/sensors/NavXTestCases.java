package frc.robot.hardware.sensors;

public class NavXTestCases {
    public double expectedOutput;
    public double currentAngle;
    public double lastAngle;
    public double currentRate;

    public NavXTestCases(double expectedOutput, double currentAngle, double lastAngle, double currentRate){
        this.expectedOutput = expectedOutput;
        this.currentAngle = currentAngle;
        this.lastAngle = lastAngle;
        this.currentRate = currentRate;
    }
}
