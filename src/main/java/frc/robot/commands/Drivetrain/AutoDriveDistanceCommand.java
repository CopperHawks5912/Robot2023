// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drivetrain;

import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** Manual Drive Command */
public class AutoDriveDistanceCommand extends CommandBase {
  private final DriveSubsystem m_driveSubsystem;
  private double m_driveDistanceMeters;
  private double m_driveSpeed;
  private double m_startingSensorCount;
  /**
   * Creates a new AutoDriveDistanceCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public AutoDriveDistanceCommand(DriveSubsystem driveSubsystem, double driveDistanceMeters, double driveSpeed ) {
    m_driveSubsystem = driveSubsystem;
    m_driveDistanceMeters = driveDistanceMeters;
    m_driveSpeed = driveSpeed;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_startingSensorCount = m_driveSubsystem.getAverageEncoderDistance();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double m_DistanceTravelledMeters = m_driveSubsystem.getDistanceTravelledMeters( m_startingSensorCount );
    if ( m_DistanceTravelledMeters < m_driveDistanceMeters)
    {
      m_driveSubsystem.arcadeDrive(m_driveSpeed, 0);
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
