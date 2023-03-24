package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.Constants.AutoConstants;

public class DriveStraightAutoCommand extends CommandBase {

    /**
     * Creates a new AutoCommand.
     */
    DriveSubsystem driveSubsystem;
    Timer timer;

    public DriveStraightAutoCommand(DriveSubsystem subsystem) {
        // Use addRequirements() here to declare subsystem dependencies.
        driveSubsystem = subsystem;
        addRequirements(driveSubsystem);
        timer = new Timer();
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        timer.reset();
        timer.start();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        driveSubsystem.arcadeDrive(.5, 0);// drive straight at half
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        // end the command if we have run for a specific amount of time
        return timer.get() > AutoConstants.kDriveTimeSeconds;

    }
}
