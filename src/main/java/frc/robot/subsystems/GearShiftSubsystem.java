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

public class GearShiftSubsystem extends SubsystemBase {
  private int currentGear;

  public static final int kLowGear = 1;
  public static final int kHighGear = 2;

  private DoubleSolenoid m_shifter = new DoubleSolenoid(CANConstants.kPCMID, 
                                                        PneumaticsModuleType.CTREPCM, 
                                                        PCMConstants.kHighGearID, 
                                                        PCMConstants.kLowGearID);

  public GearShiftSubsystem() {    
     currentGear = kLowGear;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
   }

  public int getCurrentGear() {
    return this.currentGear;
  }

  public void setCurrentGear(int gear) {
    this.currentGear = gear;
    if (this.currentGear == kLowGear) {
        m_shifter.set(kReverse);
    } else if (this.currentGear == kHighGear) {
        m_shifter.set(kForward);
     }
}

public void switchGear() {
    if (this.currentGear == kLowGear) {
        setCurrentGear(kHighGear);
    } else {
        setCurrentGear(kLowGear);
    }
}

  
}
