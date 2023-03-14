// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.CANConstants;
import frc.robot.Constants.DIOConstants;
import frc.robot.utilities.ArmArbitraryFFMode;
import frc.robot.utilities.ArmPosition;

public class ArmSubsystem extends SubsystemBase {
  private WPI_TalonSRX m_shoulderTalon = new WPI_TalonSRX(CANConstants.kShoulderTalonID);
  private WPI_VictorSPX m_shoulderVictor = new WPI_VictorSPX(CANConstants.kShoulderVictorID);
  private WPI_TalonSRX m_elbowTalon = new WPI_TalonSRX(CANConstants.kElbowTalonID);
  private DigitalInput m_shoulderLimitSwitch = new DigitalInput( DIOConstants.kShoulderLimitSwitchPort);
  private DigitalInput m_elbowLimitSwitch = new DigitalInput(DIOConstants.kElbowLimitSwitchPort);
  private int kTimeoutMs = 30;
  private ArmPosition m_currentTarget;
  private double m_shoulderArbitraryFeedForward;
  private double m_elbowArbitraryFeedForward;

  public ArmSubsystem() {    
    /* (sample code comment)
	     set to zero to skip waiting for confirmation, set to nonzero to wait and
	     report to DS if action fails.*/
    
    m_currentTarget = ArmConstants.kDefaultPosition;
    
    m_shoulderTalon.configFactoryDefault();
    m_shoulderVictor.configFactoryDefault();
    m_elbowTalon.configFactoryDefault();
    
    m_shoulderVictor.stopMotor();
    m_shoulderVictor.setInverted(InvertType.InvertMotorOutput);
    m_shoulderVictor.setNeutralMode( NeutralMode.Brake); 

    m_shoulderTalon.stopMotor();
    m_shoulderTalon.follow(m_shoulderTalon);
    m_shoulderTalon.setInverted(InvertType.OpposeMaster);
    m_shoulderTalon.setNeutralMode( NeutralMode.Brake); 
    
    m_elbowTalon.setSensorPhase(true);   
    m_elbowTalon.stopMotor();
    m_elbowTalon.setInverted(InvertType.InvertMotorOutput);
    m_elbowTalon.setNeutralMode( NeutralMode.Brake);   
    
    /* (sample code comment)
       set deadband to super small 0.001 (0.1 %).
			 The default deadband is 0.04 (4 %) */
    m_shoulderTalon.configNeutralDeadband(0.001, kTimeoutMs);
    m_shoulderVictor.configNeutralDeadband(0.001, kTimeoutMs);
    m_elbowTalon.configNeutralDeadband(0.001, kTimeoutMs);
    
    /* Set relevant frame periods to be at least as fast as periodic rate */
		m_shoulderTalon.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10, kTimeoutMs);
		m_shoulderTalon.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10, kTimeoutMs);
		m_elbowTalon.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10, kTimeoutMs);
		m_elbowTalon.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10, kTimeoutMs);

		/* Set the peak and nominal outputs */
		m_shoulderTalon.configNominalOutputForward(0, kTimeoutMs);
		m_shoulderTalon.configNominalOutputReverse(0, kTimeoutMs);
		m_shoulderTalon.configPeakOutputForward(0.3, kTimeoutMs);
		m_shoulderTalon.configPeakOutputReverse( -0.3, kTimeoutMs);
    m_elbowTalon.configNominalOutputForward(0, kTimeoutMs);
		m_elbowTalon.configNominalOutputReverse(0, kTimeoutMs);
		m_elbowTalon.configPeakOutputForward(0.4, kTimeoutMs);
		m_elbowTalon.configPeakOutputReverse(-0.08, kTimeoutMs);

		/* Set Motion Magic gains in slot0 - see documentation */
		m_shoulderTalon.selectProfileSlot( ArmConstants.kPIDProfileSlotIndex, ArmConstants.kPIDLoopIndex);
		m_shoulderTalon.config_kF(ArmConstants.kPIDProfileSlotIndex, ArmConstants.kShoulderGains.kF, kTimeoutMs);
		m_shoulderTalon.config_kP(ArmConstants.kPIDProfileSlotIndex, ArmConstants.kShoulderGains.kP, kTimeoutMs);
		m_shoulderTalon.config_kI(ArmConstants.kPIDProfileSlotIndex, ArmConstants.kShoulderGains.kI, kTimeoutMs);
		m_shoulderTalon.config_kD(ArmConstants.kPIDProfileSlotIndex, ArmConstants.kShoulderGains.kD, kTimeoutMs);
		m_elbowTalon.selectProfileSlot(ArmConstants.kPIDProfileSlotIndex, ArmConstants.kPIDLoopIndex);
		m_elbowTalon.config_kF(ArmConstants.kPIDProfileSlotIndex, ArmConstants.kElbowGains.kF, kTimeoutMs);
		m_elbowTalon.config_kP(ArmConstants.kPIDProfileSlotIndex, ArmConstants.kElbowGains.kP, kTimeoutMs);
		m_elbowTalon.config_kI(ArmConstants.kPIDProfileSlotIndex, ArmConstants.kElbowGains.kI, kTimeoutMs);
		m_elbowTalon.config_kD(ArmConstants.kPIDProfileSlotIndex, ArmConstants.kElbowGains.kD, kTimeoutMs);

    /* Set acceleration and vcruise velocity - see documentation */
		m_shoulderTalon.configMotionCruiseVelocity(3000, kTimeoutMs);
		m_shoulderTalon.configMotionAcceleration(3000, kTimeoutMs);
		m_elbowTalon.configMotionCruiseVelocity(3000, kTimeoutMs);
		m_elbowTalon.configMotionAcceleration(3000, kTimeoutMs);

		/* Zero the sensor once on robot boot up */
		m_shoulderTalon.setSelectedSensorPosition(0, ArmConstants.kPIDLoopIndex, kTimeoutMs);
		m_elbowTalon.setSelectedSensorPosition(0, ArmConstants.kPIDLoopIndex, kTimeoutMs); 
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putData(this);
    SmartDashboard.putNumber( "Shoulder Position", m_shoulderTalon.getSelectedSensorPosition(ArmConstants.kPIDLoopIndex) );
    SmartDashboard.putNumber( "Elbow Position", m_elbowTalon.getSelectedSensorPosition(ArmConstants.kPIDLoopIndex) );
    SmartDashboard.putNumber( "Shoulder Power", m_shoulderTalon.getMotorOutputPercent() );
    SmartDashboard.putNumber( "Elbow Power", m_elbowTalon.getMotorOutputPercent() );
    SmartDashboard.putNumber( "Shoulder Target", m_currentTarget.GetShoulderPosition() );
    SmartDashboard.putNumber( "Elbow Target", m_currentTarget.GetElbowPosition() );
    SmartDashboard.putBoolean( "Shoulder Switch", m_shoulderLimitSwitch.get() );
    SmartDashboard.putBoolean( "Elbow Switch", m_elbowLimitSwitch.get() );  

    m_shoulderArbitraryFeedForward = calculateShoulderArbitraryFeedForward();
    m_elbowArbitraryFeedForward = calculateElbowArbitraryFeedForward();
    SmartDashboard.putNumber( "Shoulder Arb FF", m_shoulderArbitraryFeedForward );
    SmartDashboard.putNumber( "Elbow Arb FF", m_elbowArbitraryFeedForward );  
            
  }

  public void manualControl( double shoulderSpeed, double elbowSpeed ) {
    controlJoint( m_elbowTalon, m_elbowLimitSwitch, ControlMode.PercentOutput, elbowSpeed, ArmArbitraryFFMode.kElbow );
    //controlJoint( m_shoulderTalon, m_shoulderLimitSwitch, ControlMode.PercentOutput, shoulderSpeed, ArmArbitraryFFMode.kNone );
  }
  
  public void controlJoint( WPI_TalonSRX talon, DigitalInput limitSwitch, ControlMode controlMode, double controlValue, ArmArbitraryFFMode arbitraryFFMode ) { 
    if( limitSwitch.get() ) {     
      talon.stopMotor();  
      talon.setSelectedSensorPosition(0, ArmConstants.kPIDLoopIndex, kTimeoutMs);
      // if( ( talon.getInverted() && controlValue > 0 ) ||
      //     ( !talon.getInverted() && controlValue < 0 ) )
      //   return;//don't do anything - we do dont want to push the arm through the limit switch
    }   
    if( controlMode == ControlMode.PercentOutput && controlValue == 0 ) 
      talon.stopMotor();  
    else {
      switch( arbitraryFFMode ) { 
        case kNone:
          talon.set( controlMode, controlValue );
          break;
        case kShoulder:
          talon.set( controlMode, controlValue, DemandType.ArbitraryFeedForward, m_shoulderArbitraryFeedForward );
          break;   
        case kElbow:
          talon.set( controlMode, controlValue, DemandType.ArbitraryFeedForward, m_elbowArbitraryFeedForward );
          break;
      }
    }
  }

  public void moveArmToPosition( ArmPosition armPosition )
  {
    m_currentTarget = armPosition;
    
    double targetShoulderPos = armPosition.GetShoulderPosition();
    double targetElbowPos = armPosition.GetElbowPosition();
     
    //controlJoint( m_shoulderTalon, m_shoulderLimitSwitch, ControlMode.MotionMagic, targetShoulderPos, ArmArbitraryFFMode.kShoulder );
    controlJoint( m_elbowTalon, m_elbowLimitSwitch, ControlMode.MotionMagic, targetElbowPos, ArmArbitraryFFMode.kElbow );
  }

  private double calculateShoulderArbitraryFeedForward( )
  {
    int kShoulderMeasuredPosHorizontal = 5400; // Position measured when arm is horizontal
    int kElbowMeasuredEffectivePosHorizontal = 2200; //Position measured when arm is horizontal
    double maxGravityFFHorizontalElbow = 0.08;  //power required to hold arm horizontal.
    double maxGravityFFVerticalElbow = 0.08;  //power required to hold arm horizontal.

    double kElbowTicksPerDegree = ArmConstants.kEncoderCountsPerRev * ArmConstants.kElbowGearRatio / 360; //100:1 gear box, 4:1 sprocket reduction\
    double kShoulderTicksPerDegree = ArmConstants.kEncoderCountsPerRev * ArmConstants.kShoulderGearRatio / 360; //100:1 gear box, 4:1 sprocket reduction\
    double currentShoulderPos = m_shoulderTalon.getSelectedSensorPosition();
    double currentElbowPos = m_elbowTalon.getSelectedSensorPosition();
    
    double effectiveElbowPos = currentShoulderPos + currentElbowPos;

    //calculate the maxGravityFF for the shoulder based on the elbow position
    double degrees = (effectiveElbowPos - kElbowMeasuredEffectivePosHorizontal) / kElbowTicksPerDegree;
    double radians = java.lang.Math.toRadians(degrees);
    double cosineScalar = java.lang.Math.cos(radians);
    double maxGravityFF = maxGravityFFVerticalElbow + ( cosineScalar * ( maxGravityFFHorizontalElbow - maxGravityFFVerticalElbow ) );
    
    degrees = (currentShoulderPos - kShoulderMeasuredPosHorizontal) / kShoulderTicksPerDegree;
    radians = java.lang.Math.toRadians(degrees);
    cosineScalar = java.lang.Math.cos(radians);

    double arbitraryFF = maxGravityFF * cosineScalar;
    return arbitraryFF;
  }  

  private double calculateElbowArbitraryFeedForward( )
  {
    int kMeasuredEffectivePosHorizontal = 2200; //Position measured when arm is horizontal
    double kTicksPerDegree = ArmConstants.kEncoderCountsPerRev * ArmConstants.kElbowGearRatio / 360; 
    double currentShoulderPos = m_shoulderTalon.getSelectedSensorPosition();
    double currentElbowPos = m_elbowTalon.getSelectedSensorPosition();
    double maxGravityFF = 0.08;  //NEED TO CALC YET. power required to hold arm horizontal.

    double effectiveElbowPos = currentShoulderPos + currentElbowPos;

    double degrees = (effectiveElbowPos - kMeasuredEffectivePosHorizontal) / kTicksPerDegree;
    double radians = java.lang.Math.toRadians(degrees);
    double cosineScalar = java.lang.Math.cos(radians);

    
    double arbitraryFF = maxGravityFF * cosineScalar;
     return arbitraryFF;
  }  
}
