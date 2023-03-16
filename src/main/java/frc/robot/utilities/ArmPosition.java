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
