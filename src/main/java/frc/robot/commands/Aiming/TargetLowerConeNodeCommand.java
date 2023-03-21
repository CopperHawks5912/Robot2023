// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Aiming;

import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.utilities.LimelightHelpers;
import frc.robot.Library;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import static frc.robot.Constants.*;

/** Manual Drive Command */
public class TargetLowerConeNodeCommand extends CommandBase {
  private final DriveSubsystem m_driveSubsystem;
  private double m_distanceMeters;
  private double m_angleX;
  //private Accelerator forwardAccelerator = new Accelerator(DriveConstants.kMaxAccelerationRate);
  //private Accelerator rotateAccelerator = new Accelerator(DriveConstants.kMaxAccelerationRate);
  
  /**
   * Creates a new ManualDriveCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public TargetLowerConeNodeCommand(DriveSubsystem driveSubsystem) {
    m_driveSubsystem = driveSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    LimelightHelpers.setPipelineIndex("limelight",LLConstants.kLowerConePipeline);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_distanceMeters = Library.calcDistance(FieldConstants.kLowerConeHeightMeters);
    m_angleX = LimelightHelpers.getTX("limeLightName");
    //SmartDashboard.putNumber( "targetDistance", m_distanceMeters);
    //SmartDashboard.putNumber("targetX", m_angleX );
    
    //m_driveSubsystem.arcadeDrive(forward, rotate);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (m_distanceMeters <= 1.0)
      return true;
    else 
      return false;
  }
}
