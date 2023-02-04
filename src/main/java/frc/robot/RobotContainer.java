// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.IOConstants;
import frc.robot.subsystems.DriveSubsystem;

public class RobotContainer {
  // The robot's subsystems
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();

  // The driver's controller
  CommandXboxController m_driverController =
      new CommandXboxController(IOConstants.kDriverControllerPort);

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
                    -m_driverController.getLeftY(), -m_driverController.getRightX()),
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
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
