/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import frc.robot.Constants.LLConstants;
import frc.robot.utilities.LimelightHelpers;

/**
 * Common functions that are used in multiple classes
 */

public final class Library {

    public static double bound(double value, double min, double max) {
        if (value < min) {
            value = min;
        } else if (value > max) {
            value = max;
        }
        return value;
    }
    public static double calcDistance (double goalHeightMeters){
        double targetOffsetAngle_Vertical = LimelightHelpers.getTY("limelight");

        double angleToGoalDegrees = LLConstants.kLimeLightAngle + targetOffsetAngle_Vertical;
        double angleToGoalRadians = angleToGoalDegrees * (3.14159 / 180.0);

        //calculate distance
        double distanceFromLimelightToGoalMeters = (goalHeightMeters - LLConstants.kLimeLightHeightMeters)/Math.tan(angleToGoalRadians);
        return (distanceFromLimelightToGoalMeters);
    }
}
