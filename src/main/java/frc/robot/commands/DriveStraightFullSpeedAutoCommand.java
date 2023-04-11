package frc.robot.commands;

import frc.robot.Constants.AutoConstants;
import frc.robot.subsystems.DriveSubsystem;

public class DriveStraightFullSpeedAutoCommand extends DriveStraightAutoCommand {

    public DriveStraightFullSpeedAutoCommand(DriveSubsystem subsystem) {
        super(subsystem, AutoConstants.kLongDriveTimeSeconds);
    }

    @Override
    public void execute() {
        driveSubsystem.arcadeDrive(1, 0);
    }

}
