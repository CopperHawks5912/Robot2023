// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
    SmartDashboard.putNumber( "Button Count", RobotContainer.m_secondController.getHID().getButtonCount() );
    SmartDashboard.putBoolean( "Button 1", RobotContainer.m_secondController.getHID().getRawButton(1) );
    SmartDashboard.putBoolean( "Button 2", RobotContainer.m_secondController.getHID().getRawButton(2) );
    SmartDashboard.putBoolean( "Button 3", RobotContainer.m_secondController.getHID().getRawButton(3) );
    SmartDashboard.putBoolean( "Button 4", RobotContainer.m_secondController.getHID().getRawButton(4) );
    SmartDashboard.putBoolean( "Button 5", RobotContainer.m_secondController.getHID().getRawButton(5) );
    SmartDashboard.putBoolean( "Button 6", RobotContainer.m_secondController.getHID().getRawButton(6) );
    SmartDashboard.putBoolean( "Button 7", RobotContainer.m_secondController.getHID().getRawButton(7) );
    SmartDashboard.putBoolean( "Button 8", RobotContainer.m_secondController.getHID().getRawButton(8) );
    SmartDashboard.putBoolean( "Button 9", RobotContainer.m_secondController.getHID().getRawButton(9) );
    SmartDashboard.putBoolean( "Button 10", RobotContainer.m_secondController.getHID().getRawButton(10) );
    
    SmartDashboard.putNumber( "Axis Count", RobotContainer.m_secondController.getHID().getAxisCount() );
    SmartDashboard.putNumber( "Axis 0", RobotContainer.m_secondController.getHID().getRawAxis(0) );
    SmartDashboard.putNumber( "Axis 1", RobotContainer.m_secondController.getHID().getRawAxis(1) );
    


  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();
    
    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {}

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}
