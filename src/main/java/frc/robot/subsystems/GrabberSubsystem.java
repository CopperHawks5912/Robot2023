// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

//import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.*;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import static frc.robot.Constants.*;

public class GrabberSubsystem extends SubsystemBase {
  private int m_GrabberState;

  public static final int kOpenGrabber = 1;
  public static final int kCloseGrabber = 2;

  private DoubleSolenoid m_grabberDoubleSolenoid = new DoubleSolenoid(CANConstants.kPCMID, 
                                                        PneumaticsModuleType.CTREPCM, 
                                                        PCMConstants.kGrabberOpenID, 
                                                        PCMConstants.kGrabberCloseID);

  public GrabberSubsystem() {    
    m_GrabberState = kCloseGrabber;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
   }

  public int getGrabberState() {
    return m_GrabberState;
  }

  public void openGrabber() {
    m_GrabberState = kOpenGrabber;
      m_grabberDoubleSolenoid.set(kReverse);
  }
  
  public void closeGrabber() {
    m_GrabberState = kCloseGrabber;
      m_grabberDoubleSolenoid.set(kForward);
  }  
}
