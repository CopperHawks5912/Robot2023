// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Arm;

import frc.robot.subsystems.ArmSubsystem;
import frc.robot.utilities.ArmPosition;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class AutoPositionArmCommand extends CommandBase {
  private final ArmSubsystem m_armSubsystem;
  private final ArmPosition m_targetArmPosition;
  
  public AutoPositionArmCommand(ArmSubsystem armSubsystem, ArmPosition targetArmPosition) {
    m_armSubsystem = armSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(armSubsystem);
    m_targetArmPosition = targetArmPosition;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_armSubsystem.setStartingPosition();
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {  
    m_armSubsystem.moveArmToPosition(m_targetArmPosition);    
  }
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    //return false;
    boolean isGoodEnough =  m_armSubsystem.isGoodEnough();
    SmartDashboard.putBoolean( "Good Enough!", isGoodEnough );
    return isGoodEnough;
  }
}
