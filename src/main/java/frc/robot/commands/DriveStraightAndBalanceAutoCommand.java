package frc.robot.commands;

import frc.robot.Constants.AutoConstants;
import frc.robot.subsystems.DriveSubsystem;

public class DriveStraightAndBalanceAutoCommand extends DriveStraightAutoCommand {

    AutoBalance autoBalance;

    public DriveStraightAndBalanceAutoCommand(DriveSubsystem subsystem) {
        super(subsystem, AutoConstants.kLongDriveTimeSeconds);
        autoBalance = new AutoBalance();
    }

    @Override
    public void execute() {
        driveSubsystem.arcadeDrive(autoBalance.autoBalanceRoutine(), 0);// drive straight at half
    }

}
