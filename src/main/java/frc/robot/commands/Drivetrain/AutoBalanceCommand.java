package frc.robot.commands.Drivetrain;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.utilities.AutoBalanceNoNavX;
import frc.robot.utilities.AutoBalanceWithNavX;

public class AutoBalanceCommand extends CommandBase {
        private final AutoBalanceNoNavX m_autoBalanceNoNavX = new AutoBalanceNoNavX();
        private final AutoBalanceWithNavX m_autoBalanceWithNavX = new AutoBalanceWithNavX();
        private final DriveSubsystem m_driveSubsystem;
        private boolean m_useNavX;
        // private double m_targetTurnEncoderUnits;
        // private double m_driveSpeed;
        // private double m_startingEncoderUnits;
        /**
         * Creates a new AutoDriveDistanceCommand.
         *
         * @param subsystem The subsystem used by this command.
         */
        public AutoBalanceCommand( DriveSubsystem driveSubsystem, boolean useNavX ) {
          m_driveSubsystem = driveSubsystem;
          m_useNavX = useNavX;
          // Use addRequirements() here to declare subsystem dependencies.
          addRequirements(driveSubsystem);
        }
      
        // Called when the command is initially scheduled.
        @Override
        public void initialize() {
        //  m_startingEncoderUnits = m_driveSubsystem.getLeftEncoderUnits();
        //   SmartDashboard.putNumber( "Init AutoDrive Sensor", m_startingEncoderUnits );
        //   SmartDashboard.putNumber( "Drive Turn Units", m_targetTurnEncoderUnits );
        //   SmartDashboard.putNumber( "Drive Speed", m_driveSpeed );   
        }
      
        // Called every time the scheduler runs while the command is scheduled.
        @Override
        public void execute() {
          double speed = 0;
          if( m_useNavX )
            m_autoBalanceWithNavX.autoBalanceRoutine();
          else
            m_autoBalanceNoNavX.autoBalanceRoutine();
          m_driveSubsystem.tankDrive(speed, speed, false);
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
      
