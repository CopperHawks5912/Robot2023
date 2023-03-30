// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController.Axis;
import frc.robot.utilities.ArmPosition;
import frc.robot.utilities.Gains;

public final class Constants {  
  public static class DriveConstants {
    public static final int kForwardAxis = Axis.kLeftY.value;
    public static final int kRotateAxis = Axis.kRightX.value;
    public static final double kJoystickDeadZone = 0.05;
    public static final double kMaxForwardSpeed = 1.0;
    public static final double kMaxRotateSpeed = 0.6;
    public static final double kMaxPrecisionForwardSpeed = 0.47;
    public static final double kMaxPrecisionRotateSpeed = 0.30;
    public static final double kMaxAccelerationRate = 0.1;  //this is what was in the old code.. might want to test different values
    public static final double kS = 1.5;
    public static final double kV = 2.5;//1.2932;
    public static final double kA = 0;// 1.6557;  
    public static final double kTrackwidthMeters = 0.67;
    public static final double kEncoderCountsPerRev = 2048;
    public static final double kGearRatio = 12;
    public static final double kWheelRadius = 3; 
  }

  public static class ArmConstants {
    public static final int kPIDProfileSlotIndex = 0;
    public static final int kPIDLoopIndex = 0;

    public static final double kElbowMaxManualForwardSpeed = 0.05;
    public static final double kElbowMaxManualReverseSpeed = 0.12;
    
    public static final double kElbowManualForwardPositionMultiplier = 30;
    public static final double kElbowManualReversePositionMultiplier = 30;

    public static final double kShoulderMaxPeakOutputForward = 0.7;
    public static final double kShoulderMaxPeakOutputReverse = -0.7;
    public static final double kElbowMaxPeakOutputForward = 1.0;//0.4;
    public static final double kElbowMaxPeakOutputReverse = -0.4;

    public static final double kShoulderCruiseVelocity = 700;//450; //200
    public static final double kShoulderAcceleration = 500;//450; //200
    public static final double kElbowCruiseVelocity = 1000;//900; //200
    public static final double kElbowAcceleration = 600;//900; //200

    public static final double kElbowFirstPercentage = 0.15; //0.35

    public static final int kShoulderHorizontalPosition = -5500;
    public static final int kElbowHorizontalPosition = 2200;

    public static final double kShoulderMaxGravityFFHorizontalElbow = 0.12;  //power required to hold arm horizontal when forearm is horizontal.
    public static final double kShoulderMaxGravityFFVerticalElbow = 0.12;  //power required to hold arm horizontal  when forearm is vertical.
    public static final double kElbowMaxGravityFF = 0.08;  //power required to hold forearm horizontal.

     
    /**
	   * Gains used in Motion Magic, to be adjusted accordingly
     * Gains(kp, ki, kd, kf, izone, peak output);
     */
    public static final Gains kShoulderGains = new Gains(4, 0.0, 0.0, 0.32, 0, 1.0);
    public static final Gains kElbowGains = new Gains(4, 0.0, 0.0, 0.32, 0, 1.0);         
     
    /*positions are an array of angles in encoder positions - 
         the first is the angle from default position for the shoulder (ArmConstants.kShoulderPositionIndex), 
         the second is the angle from default position for the elbow (ArmConstants.kElbowPositionIndex), */
    public static final ArmPosition kDefaultPosition = new ArmPosition(0, 0, "Default");
    public static final ArmPosition kSubstationPosition = new ArmPosition(-400, 2850, "Substation");
    public static final ArmPosition kGroundPosition = new ArmPosition(-2250, 750, "Ground");

    public static final ArmPosition kLowerConePosition = new ArmPosition(-1650, 3900, "Lower Cone");
    public static final ArmPosition kUpperConePosition = new ArmPosition(-2960, 6800, "Upper Cone");
    public static final ArmPosition kUpperCubePosition = new ArmPosition( -2600, 5675, "Upper Cube");
    public static final ArmPosition kLowerCubePosition = new ArmPosition( -1600, 3175, "Lower Cube");

    public static final ArmPosition kAutoLowerConePosition = new ArmPosition(-1300, 3750, "Lower Cone");
    public static final ArmPosition kAutoUpperConePosition = new ArmPosition(-2900, 6700, "Upper Cone");
    public static final ArmPosition kAutoUpperCubePosition = new ArmPosition( -2750, 5800, "Upper Cube");
    public static final ArmPosition kAutoLowerCubePosition = new ArmPosition( -1450, 3100, "Lower Cube");
    
    
    public static final int kEncoderCountsPerRev = 4096;
    public static final double kShoulderGearRatio = 4.0;  //4:1 chain sprockets (100:1 gearbox is in front of the encoder)
    public static final double kElbowGearRatio = 4.0;  //4:1 chain sprockets (100:1 gearbox is in front of the encoder)
  }
  
  public static class ControllerConstants
  {
    public static final int kDriverControllerPort = 0;
    public static final int kSecondControllerPort = 1;
    
    public static final int kButtonBlueUpper = 1;
    public static final int kButtonBlueLower =  5;
    public static final int kButtonRedUpper1 =  2;
    public static final int kButtonRedUpper2 =  3;
    public static final int kButtonRedUpper3 =  4;
    public static final int kButtonRedLower1 =  6;
    public static final int kButtonRedLower2 =  7;
    public static final int kButtonRedLower3 =  8;
    public static final int kButtonBlack1    = 10;
    public static final int kButtonBlack2    =  9;
    public static final int kHorizontalAxis  = 0;
    public static final int kVerticalAxis    = 1;
  }
  
  public static class AutoConstants
  {
    public static final String kUpperCone = "UpperCone";
    public static final String kLowerCone = "LowerCone";
    public static final String kUpperCube = "UpperCube";
    public static final String kLowerCube = "LowerCube";
    public static final String kLongReverseAndSpin = "LongReverseAndSpin";
    public static final String kShortReverseAndSpin = "ShortReverseAndSpin";
    public static final String kNoReverse = "NoReverse";
    public static final String kAutoBalanceNoNavX = "AutoBalance-NoNavX";
    public static final String kAutoBalanceWithNavX = "AutoBalance-WITHNavX";

    public static final double kAutoBalanceFastSpeed = 0.4;
    public static final double kAutoBalanceSlowSpeed = 0.2;
    public static final double kAutoBalanceOnChargeStationDegree = 13.0;
    public static final double kAutoBalanceLevelDegree = 6.0;
    public static final double kAutoBalanceDebounceSeconds = 0.2;

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
    public static final int kLEDStringLength = 140;
    public static final int kLEDModeOff = -1;
    public static final int kLEDModeRainbow = 0;
    public static final int kLEDModeCone = 1;
    public static final int kLEDModeCube = 2;
    public static final int kLEDModeAllianceBlue = 3;
    public static final int kLEDModeAllianceRed = 4;  
  }
}
