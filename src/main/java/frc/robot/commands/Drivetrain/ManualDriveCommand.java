// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drivetrain;

import frc.robot.subsystems.DriveSubsystem;
import frc.robot.utilities.Accelerator;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;

import static frc.robot.Constants.*;

/** Manual Drive Command */
public class ManualDriveCommand extends CommandBase {
  private final DriveSubsystem m_driveSubsystem;
  private CommandXboxController m_xboxController;
  private Accelerator forwardAccelerator = new Accelerator(DriveConstants.kMaxAccelerationRate);
  private Accelerator rotateAccelerator = new Accelerator(DriveConstants.kMaxAccelerationRate);
  
  /**
   * Creates a new ManualDriveCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public ManualDriveCommand(DriveSubsystem driveSubsystem, CommandXboxController xboxController ) {
    m_driveSubsystem = driveSubsystem;
    m_xboxController = xboxController;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double forward = m_xboxController.getRawAxis(DriveConstants.kForwardAxis);
    double rotate = m_xboxController.getRawAxis(DriveConstants.kRotateAxis);
    
    if(forward < DriveConstants.kJoystickDeadZone && 
        forward > -DriveConstants.kJoystickDeadZone) {
      forward = 0;
    }
    
    if(rotate < DriveConstants.kJoystickDeadZone && 
       rotate > -DriveConstants.kJoystickDeadZone) {
      rotate = 0;
    }
 
    forward = forwardAccelerator.adjustSpeed(forward);
    rotate = rotateAccelerator.adjustSpeed(rotate);
    m_driveSubsystem.arcadeDrive(forward, rotate);
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
