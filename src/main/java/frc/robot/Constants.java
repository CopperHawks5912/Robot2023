// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController.Axis;
import frc.robot.utilities.ArmPosition;
import frc.robot.utilities.Gains;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {  
  public static class DriveConstants {
    public static final int kForwardAxis = Axis.kLeftY.value;
    public static final int kRotateAxis = Axis.kRightX.value;
    public static final double kJoystickDeadZone = 0.05;
    public static final double kMaxSpeed = 0.75;
    public static final double kMaxAccelerationRate = 0.1;  //this is what was in the old code.. might want to test different values
    public static final double kS = 1.5;
    public static final double kV = 2.5;//1.2932;
    public static final double kA = 0;// 1.6557;  
    public static final double kTrackwidthMeters = 0.67;
    public static final double kEncoderCountsPerRev = 2048;
    public static final double kGearRatio = 24;
    public static final double kWheelRadius = 4; 
  }
  public static class ArmConstants {
    public static final int kPIDProfileSlotIndex = 0;
    public static final int kPIDLoopIndex = 0;

    /**
	   * Gains used in Motion Magic, to be adjusted accordingly
     * Gains(kp, ki, kd, kf, izone, peak output);
     */
    public static final Gains kShoulderGains = new Gains(0.2, 0.0, 0.0, 0.2, 0, 1.0);
    public static final Gains kElbowGains = new Gains(0.2, 0.0, 0.0, 0.2, 0, 1.0);         
     
    /*positions are an array of angles in encoder positions - 
         the first is the angle from default position for the shoulder (ArmConstants.kShoulderPositionIndex), 
         the second is the angle from default position for the elbow (ArmConstants.kElbowPositionIndex), */
    public static final ArmPosition kDefaultPosition = new ArmPosition(0, 0);
    public static final ArmPosition kLowerConePosition = new ArmPosition(0, 600);
    public static final ArmPosition kUpperConePosition = new ArmPosition(2000, 0);
    public static final ArmPosition kUpperCubePosition = new ArmPosition(0, 0);
    public static final ArmPosition kLowerCubePosition = new ArmPosition(0, 0);
    public static final ArmPosition kSubstationPosition = new ArmPosition(0, 0);
    public static final ArmPosition kGroundPosition = new ArmPosition(0, 0);

    public static final int kEncoderCountsPerRev = 4096;
    public static final double kShoulderGearRatio = 25.0;  //100:1 gearbox * 1:4 chain sprockets = 25:1 overall ratio
    public static final double kElbowGearRatio = 25.0;     //100:1 gearbox * 1:4 chain sprockets = 25:1 overall ratio
  }
  public static class ControllerConstants
  {
    public static final int kDriverControllerPort = 0;
    public static final int kSecondControllerPort = 1;
    
  }
  public static class DIOConstants
  {
    public static final int kShoulderLimitSwitchPort = 9;
    public static final int kElbowLimitSwitchPort = 8;
    
  }
  public static class CANConstants{
    public static final int kNone = -1;
  
    public static final int kLeftFrontDriveMotorID = 1;
    public static final int kLeftTopDriveMotorID = 2;
    public static final int kLeftBackDriveMotorID = 3;
  
    public static final int kRightFrontDriveMotorID = 4;
    public static final int kRightTopDriveMotorID = 5;
    public static final int kRightBackDriveMotorID = 6;
    
    public static final int kExtraMotor1ID = 7;
    public static final int kExtraMotor2ID = 8;
    
    public static final int kPCMID = 11;
  
    public static final int kShoulderTalonID = 13;
    public static final int kShoulderVictorID = 14;
    public static final int kElbowTalonID = 15;

  }
  public static class PWMConstants{
    public static final int kLEDStringID = 0;
  }
  public static class PCMConstants{
    public static final int kHighGearID =1;
    public static final int kLowGearID = 0; 
    public static final int kGrabberOpenID = 2;
    public static final int kGrabberCloseID = 3;
  }
  public static class LLConstants{
    public static final int kLowerConePipeline = 0;
    public static final int kUpperConePipeline = 1;
    public static final int kAprilTagPipeline = 2;
    public static final double kLimeLightHeightMeters = 0.495;
    public static final double kLimeLightAngle = 0;
  }
  public static class FieldConstants{
    public static final double kLowerConeHeightMeters = 0.61;
    public static final double kUpperConeHeightMeters = 1.11;
  }

  public static class LEDConstants{
    public static final int kLEDStringLength = 42;
    public static final int kLEDModeOff = -1;
    public static final int kLEDModeRainbow = 0;
    public static final int kLEDModeCone = 1;
    public static final int kLEDModeCube = 2;
    public static final int kLEDModeAllianceBlue = 3;
    public static final int kLEDModeAllianceRed = 4;  
  }
}