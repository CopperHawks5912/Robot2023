package frc.robot.utilities;

public class ArmPosition {
    private final double kShoulderPosition;
    private final double kElbowPosition;
    private final String kName;

    public ArmPosition(double shoulderPosition, double elbowPosition, String name) {
        kShoulderPosition = shoulderPosition;
        kElbowPosition = elbowPosition;
        kName = name;
    }

    public double GetShoulderPosition() {
        return kShoulderPosition;
    }

    public double GetElbowPosition() {
        return kElbowPosition;
    }

    public String GetName() {
        return kName;
    }
}
