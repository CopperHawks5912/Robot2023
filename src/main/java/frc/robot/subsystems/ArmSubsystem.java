// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.CANConstants;
import frc.robot.Constants.DIOConstants;
import frc.robot.Library;
import frc.robot.utilities.ArmPosition;

public class ArmSubsystem extends SubsystemBase {
  private WPI_TalonSRX m_shoulderTalon = new WPI_TalonSRX(CANConstants.kShoulderTalonID);
  private WPI_VictorSPX m_shoulderVictor = new WPI_VictorSPX(CANConstants.kShoulderVictorID);
  private WPI_TalonSRX m_elbowTalon = new WPI_TalonSRX(CANConstants.kElbowTalonID);
  private DigitalInput m_shoulderLimitSwitch = new DigitalInput( DIOConstants.kShoulderLimitSwitchPort);
  private DigitalInput m_elbowLimitSwitch = new DigitalInput(DIOConstants.kElbowLimitSwitchPort);
  private int kTimeoutMs = 30;
  private ArmPosition m_currentTarget;

  public ArmSubsystem() {    
    /* (sample code comment)
	     set to zero to skip waiting for confirmation, set to nonzero to wait and
	     report to DS if action fails.*/
    
    m_currentTarget = ArmConstants.kDefaultPosition;
    Library.initializeTalonSRX(m_shoulderTalon, false, null);
    Library.initializeVictorSPX(m_shoulderVictor, true, m_shoulderTalon);
    Library.initializeTalonSRX(m_elbowTalon, false, null);
    
    
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
		m_elbowTalon.configPeakOutputForward(0.3, kTimeoutMs);
		m_elbowTalon.configPeakOutputReverse(-0.3, kTimeoutMs);

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
     SmartDashboard.putNumber( "Shoulder Position", m_shoulderTalon.getSelectedSensorPosition(ArmConstants.kPIDLoopIndex) );
     SmartDashboard.putNumber( "Elbow Position", m_elbowTalon.getSelectedSensorPosition(ArmConstants.kPIDLoopIndex) );
     SmartDashboard.putNumber( "Shoulder Power", m_shoulderTalon.getMotorOutputPercent() );
     SmartDashboard.putNumber( "Elbow Power", m_elbowTalon.getMotorOutputPercent() );
     SmartDashboard.putNumber( "Shoulder Target", m_currentTarget.GetShoulderPosition() );
     SmartDashboard.putNumber( "Elbow Target", m_currentTarget.GetElbowPosition() );
     SmartDashboard.putBoolean( "Shoulder Switch", m_shoulderLimitSwitch.get() );
     SmartDashboard.putBoolean( "Elbow Switch", m_elbowLimitSwitch.get() );
  
  }

  public void moveArmToPosition( ArmPosition armPosition )
  {
     m_currentTarget = armPosition;
    
     double targetShoulderPos = armPosition.GetShoulderPosition();// * ArmConstants.kEncoderCountsPerRev * ArmConstants.kShoulderGearRatio;
     double targetElbowPos = armPosition.GetElbowPosition();// * ArmConstants.kEncoderCountsPerRev * ArmConstants.kElbowGearRatio;
    
    // if( m_shoulderLimitSwitch.get() )
    // {     
    //   m_shoulderTalon.stopMotor();  
    //   m_shoulderTalon.setSelectedSensorPosition(0, ArmConstants.kPIDLoopIndex, kTimeoutMs);
    // }
    // else
    // {
    //   double arbitraryFF = calculateShoulderArbitraryFeedForward();
    //   m_shoulderTalon.set( ControlMode.MotionMagic, targetShoulderPos,  DemandType.ArbitraryFeedForward, arbitraryFF );
    // }
    if( m_elbowLimitSwitch.get() )
    {     
      m_elbowTalon.stopMotor();  
      m_elbowTalon.setSelectedSensorPosition(0, ArmConstants.kPIDLoopIndex, kTimeoutMs);
    }
    //else
    {
      if( m_elbowLimitSwitch.get() && targetElbowPos <= m_shoulderTalon.getSelectedSensorPosition(ArmConstants.kPIDLoopIndex))
      {}
      else
      {
        double arbitraryFF = calculateElbowArbitraryFeedForward();
        m_elbowTalon.set( ControlMode.MotionMagic, targetElbowPos, DemandType.ArbitraryFeedForward, arbitraryFF );
      }
    }
  }

  private double calculateShoulderArbitraryFeedForward( )
  {
    int kShoulderMeasuredPosHorizontal = 5400; // Position measured when arm is horizontal
    int kElbowMeasuredEffectivePosHorizontal = 2400; //Position measured when arm is horizontal
    double maxGravityFFHorizontalElbow = 0.08;  //NEED TO CALC YET. power required to hold arm horizontal.
    double maxGravityFFVerticalElbow = 0.08;  //NEED TO CALC YET. power required to hold arm horizontal.

    double kTicksPerDegree = 4096 * 4 / 360; //100:1 gear box, 4:1 sprocket reduction\
    double currentShoulderPos = m_shoulderTalon.getSelectedSensorPosition();
    double currentElbowPos = m_elbowTalon.getSelectedSensorPosition();
    
    double effectiveElbowPos = currentShoulderPos + currentElbowPos;

    //calculate the maxGravityFF for the shoulder based on the elbow position
    double degrees = (effectiveElbowPos - kElbowMeasuredEffectivePosHorizontal) / kTicksPerDegree;
    double radians = java.lang.Math.toRadians(degrees);
    double cosineScalar = java.lang.Math.cos(radians);
    double maxGravityFF = maxGravityFFVerticalElbow + ( cosineScalar * ( maxGravityFFHorizontalElbow - maxGravityFFVerticalElbow ) );
    
    degrees = (currentShoulderPos - kShoulderMeasuredPosHorizontal) / kTicksPerDegree;
    radians = java.lang.Math.toRadians(degrees);
    cosineScalar = java.lang.Math.cos(radians);

    double arbitraryFF = maxGravityFF * cosineScalar;
    return arbitraryFF;
  }  

  private double calculateElbowArbitraryFeedForward( )
  {
    int kMeasuredEffectivePosHorizontal = 2400; //Position measured when arm is horizontal
    double kTicksPerDegree = 4096 * 4 / 360; //100:1 gear box, 4:1 sprocket reduction\
    double currentShoulderPos = m_shoulderTalon.getSelectedSensorPosition();
    double currentElbowPos = m_elbowTalon.getSelectedSensorPosition();
    double maxGravityFF = 0.07;  //NEED TO CALC YET. power required to hold arm horizontal.

    double effectiveElbowPos = currentShoulderPos + currentElbowPos;

    double degrees = (effectiveElbowPos - kMeasuredEffectivePosHorizontal) / kTicksPerDegree;
    double radians = java.lang.Math.toRadians(degrees);
    double cosineScalar = java.lang.Math.cos(radians);

    
    double arbitraryFF = maxGravityFF * cosineScalar;
     return arbitraryFF;
  }  
}
