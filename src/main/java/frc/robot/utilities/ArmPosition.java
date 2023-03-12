package frc.robot.utilities;

public class ArmPosition {
    private final double kShoulderPosition;
	private final double kElbowPosition;
	
	public ArmPosition(double shoulderPosition, double elbowPosition){
		kShoulderPosition = shoulderPosition;
		kElbowPosition = elbowPosition;
	}   
    public double GetShoulderPosition()
    {
        return kShoulderPosition;
    }
    public double GetElbowPosition()
    {
        return kElbowPosition;
    }
}
