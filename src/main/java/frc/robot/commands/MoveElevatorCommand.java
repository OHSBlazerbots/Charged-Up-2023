package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ElevatorSubsystem;

public class MoveElevatorCommand extends CommandBase {
    private final ElevatorSubsystem m_ElevatorSubsystem;

    public MoveElevatorCommand(ElevatorSubsystem subsystem) {
        m_ElevatorSubsystem = subsystem;
        addRequirements(m_ElevatorSubsystem);
    }

    public void initialize() {

        m_ElevatorSubsystem.setElevatorSpeed(1);
        // code for elevator to go down

    }

    // Called repeatedly when this Command is scheduled to run
    public void execute() {
        SmartDashboard.putNumber("Elevator Postion", m_ElevatorSubsystem.getPosition());
    }

    // Make this return true when this Command no longer needs to run execute()
    public boolean isFinished() {
        return m_ElevatorSubsystem.isAtpositionA();
    }

    public void end() {
        m_ElevatorSubsystem.setElevatorSpeed(0);

    }

}
