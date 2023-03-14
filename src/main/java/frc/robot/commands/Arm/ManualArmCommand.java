// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Arm;

import frc.robot.subsystems.ArmSubsystem;
import edu.wpi.first.wpilibj.GenericHID;
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
    double shoulderSpeed = 0;
    
    // if( m_controller.getRawButton(ControllerConstants.kJoystickUp) )
    //   elbowSpeed = -1;  
    // else if( m_controller.getRawButton(ControllerConstants.kJoystickDown) )
    //   elbowSpeed = 1;  
    
    // if( m_controller.getRawButton(ControllerConstants.kJoystickLeft) )
    //   shoulderSpeed = -1;  
    // else if( m_controller.getRawButton(ControllerConstants.kJoystickRight) )
    //   shoulderSpeed = 1;  

    elbowSpeed = m_controller.getRawAxis(ControllerConstants.kVerticalAxis);
    shoulderSpeed = m_controller.getRawAxis(ControllerConstants.kHorizontalAxis);


    elbowSpeed = elbowSpeed * ArmConstants.kMaxManualElbowSpeed;
    shoulderSpeed = shoulderSpeed * ArmConstants.kMaxManualShoulderSpeed;
      
    m_armSubsystem.manualControl(shoulderSpeed, elbowSpeed);
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
