/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.utilities;

import frc.robot.Library;
import static frc.robot.Constants.*;

/**
 * Add your docs here.
 */
public class Accelerator {
    private double m_currentSpeed;
    private double m_accelerationIncrement;
    boolean m_isAtTargetSpeed = false;

    public Accelerator(double accelerationIncrement) {
        m_currentSpeed = 0;
        m_accelerationIncrement = accelerationIncrement;
    }

    public boolean isAtTargetSpeed() {
        return m_isAtTargetSpeed;
    }

    public double adjustSpeed(double target) {
        double diff = target - m_currentSpeed;
        if (Math.abs(diff) >= m_accelerationIncrement) {
            if (diff > 0) {
                m_currentSpeed += m_accelerationIncrement;
            } else {
                m_currentSpeed -= m_accelerationIncrement;
            }
        } else {
            m_currentSpeed = target;
        }
        
        m_currentSpeed = Library.bound(m_currentSpeed, -DriveConstants.kMaxSpeed, DriveConstants.kMaxSpeed);

        // MJK 03-19-2022 if the new speed is less than the dead band setting of the
        // motor, then just set currentspeed to 0.
        // but don't do that if the acceleration increment is less than the deadband,
        // because otherwise we'll never move (i think).
        if (Math.abs(m_currentSpeed) < DriveConstants.kJoystickDeadZone &&
            Math.abs(m_accelerationIncrement) > DriveConstants.kJoystickDeadZone) {
                m_currentSpeed = 0;
        }
        m_isAtTargetSpeed = (m_currentSpeed == target);
        return m_currentSpeed;
    }

    public void reset() {
        m_currentSpeed = 0;
    }
}
