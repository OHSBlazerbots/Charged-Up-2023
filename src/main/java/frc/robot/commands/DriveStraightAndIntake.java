package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClawSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.UpperWenchSubsystem;
import frc.robot.Constants.AutoConstants;

public class DriveStraightAndIntake extends CommandBase {

    /**
     * Creates a new AutoCommand.
     */
    DriveSubsystem driveSubsystem;
    Timer timer;
    ClawSubsystem claw;
    double duration;

    public DriveStraightAndIntake(DriveSubsystem dSubsystem, ClawSubsystem uwSubsystem, double time) {
        // Use addRequirements() here to declare subsystem dependencies.
        duration = time;
        driveSubsystem = dSubsystem;
        claw = uwSubsystem;
        addRequirements(driveSubsystem);
        timer = new Timer();
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        driveSubsystem.setDriveStraight();
        timer.reset();
        timer.start();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        // driveSubsystem.arcadeDrive(AutoConstants.kDriveSpeed, 0);// drive straight at
        // half
        claw.setClawSpeed(0.75); // Upper wench moves at 75% speed to shoot cube
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        // end the command if we have run for a specific amount of time
        return timer.get() > duration;

    }
}
