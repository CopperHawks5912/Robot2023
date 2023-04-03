// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drivetrain;

import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** Manual Drive Command */
public class AutoDriveCircleCommand extends CommandBase {
  private final DriveSubsystem m_driveSubsystem;
  private double m_targetTurnEncoderUnits;
  private double m_driveSpeed;
  private double m_startingEncoderUnits;
  /**
   * Creates a new AutoDriveDistanceCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public AutoDriveCircleCommand(DriveSubsystem driveSubsystem, double turnSensorUnits, double driveSpeed ) {
    m_driveSubsystem = driveSubsystem;
    m_targetTurnEncoderUnits = turnSensorUnits;
    m_driveSpeed = driveSpeed;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_startingEncoderUnits = m_driveSubsystem.getLeftEncoderUnits();
    SmartDashboard.putNumber( "Init AutoDrive Sensor", m_startingEncoderUnits );
    SmartDashboard.putNumber( "Drive Turn Units", m_targetTurnEncoderUnits );
    SmartDashboard.putNumber( "Drive Speed", m_driveSpeed );   
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double m_currentEncoderUnits = m_driveSubsystem.getLeftEncoderUnits( );
    SmartDashboard.putNumber( "Current Turn Units", m_currentEncoderUnits );
    if( m_currentEncoderUnits - m_startingEncoderUnits < m_targetTurnEncoderUnits )
    {
      m_driveSubsystem.arcadeDrive( 0, m_driveSpeed );
    }
    else
    {
      m_driveSubsystem.arcadeDrive(0, 0);
    }   
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
