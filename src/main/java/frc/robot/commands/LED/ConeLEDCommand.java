// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.LED;

import frc.robot.Constants.LEDConstants;
import frc.robot.subsystems.AddressableLEDSubsystem;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class ConeLEDCommand extends CommandBase {
  private final AddressableLEDSubsystem m_addressableLEDSubsystem;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public ConeLEDCommand(AddressableLEDSubsystem addressableLEDSubsystem) {
    m_addressableLEDSubsystem = addressableLEDSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(addressableLEDSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() { 
    if( m_addressableLEDSubsystem.getLEDMode() == LEDConstants.kLEDModeCone )
    {
      if( DriverStation.getAlliance() == Alliance.Blue)
        m_addressableLEDSubsystem.setLEDMode(LEDConstants.kLEDModeAllianceBlue);
      else 
        m_addressableLEDSubsystem.setLEDMode(LEDConstants.kLEDModeAllianceRed);
    }
    else
      m_addressableLEDSubsystem.setLEDMode(LEDConstants.kLEDModeCone);  
     
  }
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
