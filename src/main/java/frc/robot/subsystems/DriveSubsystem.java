// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;


import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPRamseteCommand;

import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;

import static frc.robot.Constants.*;

public class DriveSubsystem extends SubsystemBase {
  private WPI_TalonFX m_left1 = new WPI_TalonFX(CANConstants.kLeftFrontDriveMotorID);
  private WPI_TalonFX m_left2 = new WPI_TalonFX(CANConstants.kLeftTopDriveMotorID);
  private WPI_TalonFX m_left3 = new WPI_TalonFX(CANConstants.kLeftBackDriveMotorID);
  private WPI_TalonFX m_right1 = new WPI_TalonFX(CANConstants.kRightFrontDriveMotorID);
  private WPI_TalonFX m_right2 = new WPI_TalonFX(CANConstants.kRightTopDriveMotorID);
  private WPI_TalonFX m_right3 = new WPI_TalonFX(CANConstants.kRightBackDriveMotorID);
  private DifferentialDrive m_drive = new DifferentialDrive(m_left1, m_right1);    
  private AHRS m_navX = new AHRS(SPI.Port.kMXP);  
  private final DifferentialDriveOdometry m_odometry;
  private DifferentialDriveKinematics m_Kinematics = new DifferentialDriveKinematics(DriveConstants.kTrackwidthMeters);
  private NeutralMode m_neutralMode;

  public DriveSubsystem() {    
    
    m_neutralMode = NeutralMode.Coast;
    initializeTalonFX(m_left1, true, null);
    initializeTalonFX(m_left2, false, m_left1);
    initializeTalonFX(m_left3, true, m_left1);
    initializeTalonFX(m_right1, false, null);
    initializeTalonFX(m_right2, true, m_right1);
    initializeTalonFX(m_right3, false, m_right1);
    resetEncoders();
    m_odometry = new DifferentialDriveOdometry(
        m_navX.getRotation2d(), nativeUnitsToDistanceMeters( m_left1.getSelectedSensorPosition() ), nativeUnitsToDistanceMeters( m_right1.getSelectedSensorPosition() ) );
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    //SmartDashboard.putNumber( "Pose X", m_odometry.getPoseMeters().getX() );
    //SmartDashboard.putNumber( "Pose Y", m_odometry.getPoseMeters().getY() );
    //SmartDashboard.putNumber( "Left Sensor", m_left1.getSelectedSensorPosition() );
    //SmartDashboard.putNumber( "Right Sensor", m_right1.getSelectedSensorPosition() );
     m_odometry.update(
        m_navX.getRotation2d(), nativeUnitsToDistanceMeters( m_left1.getSelectedSensorPosition() ), nativeUnitsToDistanceMeters( m_right1.getSelectedSensorPosition() ) );
  }

  private void initializeTalonFX( WPI_TalonFX talon, boolean invert, WPI_TalonFX masterTalon ) { // Creates and configures a TalonFX
  
    talon.configFactoryDefault();
    if( masterTalon != null ) 
      talon.follow(masterTalon);
    else
      talon.stopMotor();
    talon.setInverted(invert);
    talon.setNeutralMode( NeutralMode.Coast);
  }
  
  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds( nativeUnitsToDistanceMeters( m_left1.getSelectedSensorVelocity() ) , nativeUnitsToDistanceMeters( m_right1.getSelectedSensorVelocity() ) );
  }

   public void resetOdometry(Pose2d pose) {
    resetEncoders();
    //SmartDashboard.putNumber( "Init Left Sensor 1", m_left1.getSelectedSensorPosition() );
    //SmartDashboard.putNumber( "Init Right Sensor 1", m_right1.getSelectedSensorPosition() );
    m_odometry.resetPosition(
        m_navX.getRotation2d(), nativeUnitsToDistanceMeters( m_left1.getSelectedSensorPosition() ),nativeUnitsToDistanceMeters( m_right1.getSelectedSensorPosition() ), pose);
    //SmartDashboard.putNumber( "Init Pose X", m_odometry.getPoseMeters().getX() );
    //SmartDashboard.putNumber( "Init Pose Y", m_odometry.getPoseMeters().getY() );
    //SmartDashboard.putNumber( "Init Left Sensor 2", m_left1.getSelectedSensorPosition() );
    //SmartDashboard.putNumber( "Init Right Sensor 2", m_right1.getSelectedSensorPosition() );
  }
       
  public void arcadeDrive(double fwd, double rot) {
    SmartDashboard.putNumber( "Forward Speed", fwd);
    //SmartDashboard.putNumber( "Rotate Speed", rot);
    m_drive.arcadeDrive(fwd, rot);
  }

  public Command followTrajectoryCommand(PathPlannerTrajectory traj, boolean isFirstPath) {
    return new SequentialCommandGroup(
        new InstantCommand(() -> {
          // Reset odometry for the first path you run during auto
          if(isFirstPath){
              this.resetOdometry(traj.getInitialPose());
          }
        }),
        new PPRamseteCommand(
            traj, 
            this::getPose, // Pose supplier
            new RamseteController(),
            new SimpleMotorFeedforward( DriveConstants.kS, DriveConstants.kV, DriveConstants.kA),
            this.m_Kinematics, // DifferentialDriveKinematics
            this::getWheelSpeeds, // DifferentialDriveWheelSpeeds supplier
            new PIDController(0, 0, 0), // Left controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
            new PIDController(0, 0, 0), // Right controller (usually the same values as left controller)
            this::tankDriveVolts, // Voltage biconsumer
            true, // Should the path be automatically mirrored depending on alliance color. Optional, defaults to true
            this // Requires this drive subsystem
        )
    );
}

  public void tankDriveVolts(double leftVolts, double rightVolts) {
    m_left1.setVoltage(leftVolts);
    m_right1.setVoltage(rightVolts);
    m_drive.feed();
  }


  public void resetEncoders() {
    m_left1.setSelectedSensorPosition(0);
    m_right1.setSelectedSensorPosition(0);
    Timer m_timer = new Timer();
    m_timer.start();
    while( !m_timer.hasElapsed(0.5) )
    {}
    m_timer.stop();
  }

  public double getAverageEncoderDistance() {
    return (m_left1.getSelectedSensorPosition());// + m_right1.getSelectedSensorPosition()) / 2.0;
  }

  /**
   * Sets the max output of the drive. Useful for scaling the drive to drive more slowly.
   *
   * @param maxOutput the maximum output to which the drive will be constrained
   */
  public void setMaxOutput(double maxOutput) {
    m_drive.setMaxOutput(maxOutput);
  }

  /** Zeroes the heading of the robot. */
  public void zeroHeading() {
    m_navX.reset();
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  public double getHeading() {
    return m_navX.getRotation2d().getDegrees();
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    return -m_navX.getRate();
  }

  public double getDistanceTravelledMeters(double startingSensorCount) {
    double currentSensorCount = getAverageEncoderDistance();
    double distance = nativeUnitsToDistanceMeters(currentSensorCount- startingSensorCount );
    return distance;
  }

  private static double nativeUnitsToDistanceMeters(double sensorCounts){
    double motorRotations = (double)sensorCounts / DriveConstants.kEncoderCountsPerRev;
    double wheelRotations = motorRotations / DriveConstants.kGearRatio;
    double positionMeters = wheelRotations * (2 * Math.PI * Units.inchesToMeters(DriveConstants.kWheelRadius));
    return positionMeters;
  }
  public void setNeutralMode( NeutralMode mode )
  {
    if( m_neutralMode != mode )
    {
      m_left1.setNeutralMode( mode);
      m_left2.setNeutralMode( mode);
      m_left3.setNeutralMode( mode);
      m_right1.setNeutralMode( mode);
      m_right2.setNeutralMode( mode);
      m_right3.setNeutralMode( mode);
    }
    m_neutralMode = mode;
  }
}
