// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.ControllerConstants;
import frc.robot.commands.ManualDriveCommand;
import frc.robot.commands.Aiming.TargetLowerConeNodeCommand;
import frc.robot.commands.Aiming.TargetUpperConeNodeCommand;
import frc.robot.commands.Arm.DefaultArmPositionCommand;
import frc.robot.commands.Arm.LowerConeArmPositionCommand;
import frc.robot.commands.Arm.UpperConeArmPositionCommand;
import frc.robot.commands.Grabber.SwitchGrabberCommand;
import frc.robot.commands.LED.AllianceLEDCommand;
import frc.robot.commands.LED.ConeLEDCommand;
import frc.robot.commands.LED.CubeLEDCommand;
import frc.robot.commands.LED.RainbowLEDCommand;
import frc.robot.commands.ShiftGear.HighGearCommand;
import frc.robot.commands.ShiftGear.LowGearCommand;
import frc.robot.subsystems.AddressableLEDSubsystem;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.GearShiftSubsystem;
import frc.robot.subsystems.GrabberSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  //private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();

  private final DriveSubsystem m_driveSubsystem = new DriveSubsystem();
  private final GearShiftSubsystem m_GearShiftSubsystem = new GearShiftSubsystem();
  private final GrabberSubsystem m_GrabberSubsystem = new GrabberSubsystem();
  private final AddressableLEDSubsystem m_addressableLEDSubsystem = new AddressableLEDSubsystem();
  private final ArmSubsystem m_armSubsystem = new ArmSubsystem();
 
  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController = new CommandXboxController(ControllerConstants.kDriverControllerPort);
  //private final CommandGenericHID m_secondController = new CommandGenericHID(ControllerConstants.kSecondControllerPort);
      
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();

    // Configure default commands
    // Set the default drive command to the ManualDriveCommand
    m_driveSubsystem.setDefaultCommand( new ManualDriveCommand(m_driveSubsystem, m_driverController) );
    m_addressableLEDSubsystem.setDefaultCommand( new AllianceLEDCommand(m_addressableLEDSubsystem).ignoringDisable( true) );     
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    m_driverController.x()
      .onTrue( new HighGearCommand(m_GearShiftSubsystem))
      .onFalse( new LowGearCommand(m_GearShiftSubsystem));
    // m_driverController.y()
    //   .onTrue( new SwitchGrabberCommand(m_GrabberSubsystem));
    //  m_driverController.b()
    //    .onTrue( new DefaultArmPositionCommand(m_armSubsystem));
    //  m_driverController.a()
    //   .onTrue( new LowerConeArmPositionCommand(m_armSubsystem));
    // m_driverController.leftBumper()
    //   .whileTrue( new ConeLEDCommand(m_addressableLEDSubsystem));
    // m_driverController.rightBumper()
    //   .whileTrue( new CubeLEDCommand(m_addressableLEDSubsystem));

    // m_secondController.button(1)
    //   .onTrue( new LowerConeArmPositionCommand(m_armSubsystem) );
    // m_secondController.button(2)
    //   .onTrue( new LowerConeArmPositionCommand(m_armSubsystem) );
    // m_secondController.button(3)
    //   .onTrue( new LowerConeArmPositionCommand(m_armSubsystem) );
    //   m_secondController.button(4)
    //   .onTrue( new LowerConeArmPositionCommand(m_armSubsystem) );
    //   m_secondController.button(5)
    //   .onTrue( new LowerConeArmPositionCommand(m_armSubsystem) );
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    PathPlannerTrajectory examplePath = PathPlanner.loadPath("SBend", new PathConstraints(4, 3));
    
    ParallelCommandGroup group = new ParallelCommandGroup(
      new RainbowLEDCommand(m_addressableLEDSubsystem), 
      m_driveSubsystem.followTrajectoryCommand(examplePath, true) );
      return group;
     //return m_driveSubsystem.followTrajectoryCommand(examplePath, true);
  }
}
