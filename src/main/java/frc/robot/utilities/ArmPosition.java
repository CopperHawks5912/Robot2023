package frc.robot.utilities;

public class ArmPosition {
    private double kShoulderPosition;
    private double kElbowPosition;
    private String kName;

    public ArmPosition(double shoulderPosition, double elbowPosition, String name) {
        kShoulderPosition = shoulderPosition;
        kElbowPosition = elbowPosition;
        kName = name;
    }

    
    public void setPosition( ArmPosition position) {
        kElbowPosition = position.getElbowPosition();
        kShoulderPosition = position.getShoulderPosition();
        kName = position.getName();
    }
     
    public void setElbowPosition( double position) {
        kElbowPosition = position;
        kName = "manualOverride";
    }


    public double getShoulderPosition() {
        return kShoulderPosition;
    }

    public double getElbowPosition() {
        return kElbowPosition;
    }

    public String getName() {
        return kName;
    }
}
