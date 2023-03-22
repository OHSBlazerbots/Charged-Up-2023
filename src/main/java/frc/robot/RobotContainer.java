// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import frc.robot.Constants.IOConstants;
import frc.robot.commands.DriveStraightAutoCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.WenchSubsystem;
import frc.robot.subsystems.ClawSubsystem;
import frc.robot.commands.MoveElevatorCommand;
import frc.robot.subsystems.CameraSubsystem;

public class RobotContainer {
        // The robot's subsystems
        private final DriveSubsystem m_robotDrive = new DriveSubsystem();
        private final ElevatorSubsystem m_robotElevator = new ElevatorSubsystem();
        private final WenchSubsystem m_robotWench = new WenchSubsystem();
        private final ClawSubsystem m_robotClaw = new ClawSubsystem();
        private final CameraSubsystem m_robotCamera = new CameraSubsystem();
        private final Command m_SimpleAuto = new DriveStraightAutoCommand(m_robotDrive);
        private final Command m_ComplexAuto = new DriveStraightAutoCommand(m_robotDrive);
        private final Command m_NothingAuto = null;
        SendableChooser<Command> m_chooser = new SendableChooser<>();

        // The driver's controller
        CommandXboxController m_driverController = new CommandXboxController(IOConstants.kDriverControllerPort);

        CommandGenericHID m_CoDriverController = new CommandGenericHID(IOConstants.kCoDriverControllerPort);

        public RobotContainer() {
                configureBindings();

                // Configure default commands
                // Set the default drive command to split-stick arcade drive
                m_robotDrive.setDefaultCommand(
                                // A split-stick arcade command, with forward/backward controlled by the left
                                // hand, and turning controlled by the right.
                                Commands.run(
                                                () -> m_robotDrive.arcadeDrive(
                                                                -m_driverController.getLeftY(),
                                                                m_driverController.getLeftX()),
                                                m_robotDrive));
                // Add commands to the autonomous command chooser
                m_chooser.setDefaultOption("Simple Auto", m_SimpleAuto);
                // m_chooser.addOption("Complex Auto", m_ComplexAuto);
                m_chooser.addOption("Nothing Auto", m_NothingAuto);
                // Put the chooser on the dashboard
                SmartDashboard.putData(m_chooser);

        }

        /**
         * Use this method to define your button->command mappings
         */
        private void configureBindings() {
                // Use bumpers to change virtual "gears"
                m_driverController
                                .leftTrigger()
                                .onTrue(Commands.runOnce(() -> m_robotDrive.decrementMaxSpeed()));
                m_driverController
                                .rightTrigger()
                                .onTrue(Commands.runOnce(() -> m_robotDrive.incrementMaxSpeed()));

                m_driverController // This toggle drive modes
                                .rightBumper()
                                .onTrue(Commands.runOnce(() -> m_robotDrive.toggleDriveMode()));

                // This is for elevator up and down movement.
                double elevHold = 0.10;
                m_driverController // This moves elevator up
                                .y()
                                .onTrue(Commands.runOnce(() -> m_robotElevator.setElevatorSpeed(-0.5)))
                                .onFalse(Commands.runOnce(() -> m_robotElevator.setElevatorSpeed(-elevHold)));

                m_driverController // This moves elevator down
                                .a()
                                .onTrue(Commands.runOnce(() -> m_robotElevator.setElevatorSpeed(0.5)))
                                .onFalse(Commands.runOnce(() -> m_robotElevator.setElevatorSpeed(-elevHold)));
                m_driverController // This moves elevator down
                                .start()
                                .onTrue(Commands.runOnce(() -> m_robotElevator.zeroSensors()));
                m_driverController // This moves elevator down
                                .back()
                                .onTrue(Commands.runOnce(() -> m_robotElevator.setPositionZero()));
                m_driverController // This moves elevator down
                                .x()
                                .onTrue(Commands.runOnce(() -> m_robotElevator.setElevatorPosition()));
                // m_CoDriverController
                // .button(3)
                // .onTrue(new MoveElevatorCommand(m_robotElevator));
                // m_driverController
                // .a()
                // .onTrue(Commands.runOnce(() -> m_robotCamera.toggleCameraSelection()));

                // This is for wench movement forward and backward.
                m_driverController // This moves wench backwards
                                .b()
                                .onTrue(Commands.runOnce(() -> m_robotWench.setWenchSpeed(-0.1)))
                                .onFalse(Commands.runOnce(() -> m_robotWench.setWenchSpeed(0.1)));

                m_driverController // This moves wench forwards
                                .x()
                                .onTrue(Commands.runOnce(() -> m_robotWench.setWenchSpeed(1)))
                                .onFalse(Commands.runOnce(() -> m_robotWench.setWenchSpeed(0.1)));

                // Reset claw's encoder logic
                m_CoDriverController
                                // button start+B (red for reset)
                                .button(IOConstants.kCoDriverButtonStart)
                                .and(m_CoDriverController.button(IOConstants.kCoDriverButtonB))
                                // TODO: determine what reset function to call
                                .onTrue(Commands.runOnce(() -> m_robotClaw.zeroSensors()));

                // Moving claw to pre-set positions
                m_CoDriverController
                                .button(IOConstants.kCoDriverButtonY) // button Y is yellow, like cones
                                .onTrue(Commands.runOnce(() -> m_robotClaw.goToConePosition()));
                m_CoDriverController
                                .button(IOConstants.kCoDriverButtonX) // button X is blue/purple, like cubes
                                .onTrue(Commands.runOnce(() -> m_robotClaw.goToCubePosition()));
                m_CoDriverController
                                .button(IOConstants.kCoDriverButtonA) // button A is green for open
                                .onTrue(Commands.runOnce(() -> m_robotClaw.goToMaxOpenPosition()));

                // This is for claw movement to manually open.
                m_CoDriverController
                                .button(IOConstants.kCoDriverButtonRightBumber)
                                .onTrue(Commands.runOnce(() -> m_robotClaw.setClawSpeed(0.5)))
                                .onFalse(Commands.runOnce(() -> m_robotClaw.setClawSpeed(0)));
                // This is for claw movement to manually close.
                m_CoDriverController
                                .axisGreaterThan(IOConstants.kCoDriverAxisRightTrigger, 0)
                                .onTrue(Commands.runOnce(() -> m_robotClaw.setClawSpeed(-0.5)))
                                .onFalse(Commands.runOnce(() -> m_robotClaw.setClawSpeed(0)));

        }

        public Command getAutonomousCommand() {
                // Drives forward at specific speed at a specific time
                return m_chooser.getSelected();

        }

}