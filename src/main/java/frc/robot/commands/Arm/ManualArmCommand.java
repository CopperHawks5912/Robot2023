// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Arm;

import frc.robot.subsystems.ArmSubsystem;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

import static frc.robot.Constants.*;

/** Manual Drive Command */
public class ManualArmCommand extends CommandBase {
  private final ArmSubsystem m_armSubsystem;
  private GenericHID m_controller; 
  
  public ManualArmCommand(ArmSubsystem armSubsystem, GenericHID controller ) {
    m_armSubsystem = armSubsystem;
    m_controller = controller;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_armSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double elbowSpeed = 0;
    
    elbowSpeed = m_controller.getRawAxis(ControllerConstants.kVerticalAxis);
    if(elbowSpeed > 1)
      elbowSpeed = elbowSpeed * ArmConstants.kElbowMaxManualForwardSpeed;      
    if(elbowSpeed < 1)
      elbowSpeed = elbowSpeed * ArmConstants.kElbowMaxManualReverseSpeed;      
    m_armSubsystem.manualControl(elbowSpeed);
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
