package frc.robot.utilities;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.AutoConstants;

public class AutoBalanceWithNavX {
    private AHRS m_navX;
    private int state;
    private int debounceCount;
    private double robotSpeedSlow;
    private double robotSpeedFast;
    private double onChargeStationDegree;
    private double levelDegree;
    private double debounceTime;
    private double singleTapTime;
    private double scoringBackUpTime;
    private double doubleTapTime;

    public AutoBalanceWithNavX() {        
        m_navX = new AHRS(SPI.Port.kMXP); 
        state = 0;
        debounceCount = 0;

        /**********
         * CONFIG *
         **********/
        // Speed the robot drived while scoring/approaching station, default = 0.4
        robotSpeedFast = AutoConstants.kAutoBalanceFastSpeed;//0.4;

        // Speed the robot drives while balancing itself on the charge station.
         // Should be roughly half the fast speed, to make the robot more accurate,
        // default = 0.2
        robotSpeedSlow = AutoConstants.kAutoBalanceSlowSpeed; //0.2;

        // Angle where the robot knows it is on the charge station, default = 13.0
        onChargeStationDegree = AutoConstants.kAutoBalanceOnChargeStationDegree;//13.0;

        // Angle where the robot can assume it is level on the charging station
        // Used for exiting the drive forward sequence as well as for auto balancing,
        // default = 6.0
        levelDegree = AutoConstants.kAutoBalanceLevelDegree; //6.0;

        // Amount of time a sensor condition needs to be met before changing states in
        // seconds
        // Reduces the impact of sensor noice, but too high can make the auto run
        // slower, default = 0.2
        debounceTime = AutoConstants.kAutoBalanceDebounceSeconds;//0.2;

        // Amount of time to drive towards to scoring target when trying to bump the
        // game piece off
        // Time it takes to go from starting position to hit the scoring target
        singleTapTime = 0.4;

        // Amount of time to drive away from knocked over gamepiece before the second
        // tap
        scoringBackUpTime = 0.2;

        // Amount of time to drive forward to secure the scoring of the gamepiece
        doubleTapTime = 0.3;

    }

    public double getPitch() {
        return m_navX.getPitch();
        // return Math.atan2((-m_navX.getWorldLinearAccelX()),
        //         Math.sqrt(m_navX.getWorldLinearAccelY() * m_navX.getWorldLinearAccelY() + m_navX.getWorldLinearAccelZ() * m_navX.getWorldLinearAccelZ())) * 57.3;
    }

    public double getRoll() {
        return m_navX.getRoll();
        //return Math.atan2(m_navX.getWorldLinearAccelY(), m_navX.getWorldLinearAccelZ()) * 57.3;
    }

    // returns the magnititude of the robot's tilt calculated by the root of
    // pitch^2 + roll^2, used to compensate for diagonally mounted rio
    public double getTilt() {
        double tilt;
        double pitch = getPitch();
        double roll = getRoll();
        
        tilt = roll;
        // if ((pitch + roll) >= 0) {
        //     tilt = Math.sqrt(pitch * pitch + roll * roll);
        // } else {
        //     tilt = -Math.sqrt(pitch * pitch + roll * roll);
        // }
        SmartDashboard.putNumber("Pitch", pitch);
        SmartDashboard.putNumber("Roll", roll);
        SmartDashboard.putNumber("Tilt", tilt);
        return tilt;
    }

    public int secondsToTicks(double time) {
        return (int) (time * 50);
    }

    // routine for automatically driving onto and engaging the charge station.
    // returns a value from -1.0 to 1.0, which left and right motors should be set
    // to.
    public double autoBalanceRoutine() {
        SmartDashboard.putNumber("Balance State", state);
        switch (state) {
            // drive forwards to approach station, exit when tilt is detected
            case 0:
                if (getTilt() > onChargeStationDegree) {
                    debounceCount++;
                }
                if (debounceCount > secondsToTicks(debounceTime)) {
                    state = 1;
                    debounceCount = 0;
                    return robotSpeedSlow;
                }
                return robotSpeedFast;
            // driving up charge station, drive slower, stopping when level
            case 1:
                if (getTilt() < levelDegree) {
                    debounceCount++;
                }
                if (debounceCount > secondsToTicks(debounceTime)) {
                    state = 2;
                    debounceCount = 0;
                    return 0;
                }
                return robotSpeedSlow;
            // on charge station, stop motors and wait for end of auto
            case 2:
                if (Math.abs(getTilt()) <= levelDegree / 2) {
                    debounceCount++;
                }
                if (debounceCount > secondsToTicks(debounceTime)) {
                    state = 4;
                    debounceCount = 0;
                    return 0;
                }
                if (getTilt() >= levelDegree) {
                    return 0.1;
                } else if (getTilt() <= -levelDegree) {
                    return -0.1;
                }
            case 3:
                return 0;
        }
        return 0;
    }

    // Same as auto balance above, but starts auto period by scoring
    // a game piece on the back bumper of the robot
    public double scoreAndBalance() {
        switch (state) {
            // drive back, then forwards, then back again to knock off and score game piece
            case 0:
                debounceCount++;
                if (debounceCount < secondsToTicks(singleTapTime)) {
                    return -robotSpeedFast;
                } else if (debounceCount < secondsToTicks(singleTapTime + scoringBackUpTime)) {
                    return robotSpeedFast;
                } else if (debounceCount < secondsToTicks(singleTapTime + scoringBackUpTime + doubleTapTime)) {
                    return -robotSpeedFast;
                } else {
                    debounceCount = 0;
                    state = 1;
                    return 0;
                }
                // drive forwards until on charge station
            case 1:
                if (getTilt() > onChargeStationDegree) {
                    debounceCount++;
                }
                if (debounceCount > secondsToTicks(debounceTime)) {
                    state = 2;
                    debounceCount = 0;
                    return robotSpeedSlow;
                }
                return robotSpeedFast;
            // driving up charge station, drive slower, stopping when level
            case 2:
                if (getTilt() < levelDegree) {
                    debounceCount++;
                }
                if (debounceCount > secondsToTicks(debounceTime)) {
                    state = 3;
                    debounceCount = 0;
                    return 0;
                }
                return robotSpeedSlow;
            // on charge station, ensure robot is flat, then end auto
            case 3:
                if (Math.abs(getTilt()) <= levelDegree / 2) {
                    debounceCount++;
                }
                if (debounceCount > secondsToTicks(debounceTime)) {
                    state = 4;
                    debounceCount = 0;
                    return 0;
                }
                if (getTilt() >= levelDegree) {
                    return robotSpeedSlow / 2;
                } else if (getTilt() <= -levelDegree) {
                    return -robotSpeedSlow / 2;
                }
            case 4:
                return 0;
        }
        return 0;
    } 
}
