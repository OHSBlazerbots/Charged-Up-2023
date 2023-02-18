// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import frc.robot.Constants.IOConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ClawSubsystem;

public class RobotContainer {
  // The robot's subsystems
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();
  private final ElevatorSubsystem m_robotElevator = new ElevatorSubsystem();
  private final ArmSubsystem m_robotArm = new ArmSubsystem();
  private final ClawSubsystem m_robotClaw = new ClawSubsystem();
  // The driver's controller
  CommandXboxController m_driverController =
        new CommandXboxController(IOConstants.kDriverControllerPort);

  CommandGenericHID m_CoDriverController = 
        new CommandGenericHID(IOConstants.kCoDriverControllerPort);

  public RobotContainer() {
    configureBindings();

    // Configure default commands
    // Set the default drive command to split-stick arcade drive
    m_robotDrive.setDefaultCommand(
        // A split-stick arcade command, with forward/backward controlled by the left
        // hand, and turning controlled by the right.
        Commands.run(
            () ->
                m_robotDrive.arcadeDrive(
                    m_driverController.getLeftY(), -m_driverController.getLeftX()),
            m_robotDrive));
  }

  /**
   * Use this method to define your button->command mappings
   */
  private void configureBindings() {
    // While holding the shoulder button, drive at half speed
    m_driverController
        .rightBumper()
        .onTrue(Commands.runOnce(() -> m_robotDrive.setMaxOutput(0.5)))
        .onFalse(Commands.runOnce(() -> m_robotDrive.setMaxOutput(1)));
    // This is for elevator up and down movement.
    m_driverController // This moves elevator up
        .y()
        .onTrue(Commands.runOnce(() -> m_robotElevator.setElevatorSpeed(0.5)))
        .onFalse(Commands.runOnce(() -> m_robotElevator.setElevatorSpeed(0.1)));
    
    m_driverController // This moves elevator down
        .a()
        .onTrue(Commands.runOnce(() -> m_robotElevator.setElevatorSpeed(-0.5)))
        .onFalse(Commands.runOnce(() -> m_robotElevator.setElevatorSpeed(0.1)));

    // This is for arm movement forward and backward.
    m_driverController // This moves arm backwards
        .x()
        .onTrue(Commands.runOnce(() -> m_robotArm.setArmSpeed(-0.5)))
        .onFalse(Commands.runOnce(() -> m_robotArm.setArmSpeed(0)));

    m_driverController // This moves arm forwards
        .b()
        .onTrue(Commands.runOnce(() -> m_robotArm.setArmSpeed(0.5)))
        .onFalse(Commands.runOnce(() -> m_robotArm.setArmSpeed(0)));
    
    //This is for claw movement to open.
    m_CoDriverController
        .button(1)
        .onTrue(Commands.runOnce(() -> m_robotClaw.setClawSpeed(0.5)))
        .onFalse(Commands.runOnce(() -> m_robotClaw.setClawSpeed(0)));
    //This is for claw movement to close.
    m_CoDriverController    
        .button(2)
        .onTrue(Commands.runOnce(() -> m_robotClaw.setClawSpeed(-0.5)))
        .onFalse(Commands.runOnce(() -> m_robotClaw.setClawSpeed(0)));    
        
  }

  public Command getAutonomousCommand() {
    //Drives forward at half-speed for half a second.
    return Commands.sequence(
        () -> m_robotDrive.arcadeDrive(0.5,0.0), 

        //Waits half of a second.
        CommandBase.waitSeconds(0.5),

        //Stops.
        () -> m_robotDrive.arcadeDrive(0.0,0.0)
        );
                    //.waitSeconds(0.5)
                    //Stops.
                    //.runOnce(() -> m_robotDrive.arcadeDrive(0.0,0.0));
  }
}
