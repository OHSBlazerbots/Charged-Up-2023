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
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.IOConstants;
import frc.robot.commands.DriveStraightAutoCommand;
import frc.robot.commands.DriveStraightAndBalanceAutoCommand;
import frc.robot.commands.DriveStraightAndIntake;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.UpperWenchSubsystem;
import frc.robot.subsystems.WenchSubsystem;
import frc.robot.subsystems.ClawSubsystem;
import frc.robot.commands.MoveElevatorCommand;
import frc.robot.subsystems.CameraSubsystem;
import frc.robot.commands.DriveStraightFullSpeedAutoCommand;

public class RobotContainer {
        // The robot's subsystems
        private final DriveSubsystem m_robotDrive = new DriveSubsystem();
        private final ElevatorSubsystem m_robotElevator = new ElevatorSubsystem();
        private final UpperWenchSubsystem m_upperWench = new UpperWenchSubsystem();
        private final WenchSubsystem m_robotWench = new WenchSubsystem();
        private final ClawSubsystem m_robotClaw = new ClawSubsystem();
        private final CameraSubsystem m_robotCamera = new CameraSubsystem();
        private final Command m_SimpleShortAuto = new DriveStraightAutoCommand(m_robotDrive,
                        AutoConstants.kShortDriveTimeSeconds);
        private final Command m_SimpleLongAuto = new DriveStraightAutoCommand(m_robotDrive,
                        AutoConstants.kLongDriveTimeSeconds);
        private final Command m_DriveStraightAndIntake = new DriveStraightAndIntake(m_robotDrive,
                        m_upperWench, 1);
        private final Command m_ComplexAuto = new DriveStraightAndBalanceAutoCommand(m_robotDrive);
        private final Command m_NothingAuto = null;
        private final Command m_DriveStraightFullSpeedAutoCommand = new DriveStraightFullSpeedAutoCommand(m_robotDrive);

        SendableChooser<Command> m_chooser = new SendableChooser<>();

        // The driver's controller
        CommandXboxController m_driverController = new CommandXboxController(IOConstants.kDriverControllerPort);

        CommandXboxController m_CoDriverController = new CommandXboxController(IOConstants.kCoDriverControllerPort);

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

                m_chooser.addOption("Balance on charging station Auto", m_ComplexAuto);
                m_chooser.addOption("Nothing Auto", m_NothingAuto);
                m_chooser.addOption("Drive straight Long Auto(7 seconds)", m_SimpleLongAuto);
                m_chooser.addOption("Drive straight Short Auto(2.5 Seconds)", m_SimpleShortAuto);
                m_chooser.addOption("Shoot cube auto for 1 second", m_DriveStraightAndIntake);
                m_chooser.addOption("Drive straight full speed", m_DriveStraightFullSpeedAutoCommand);
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
                m_CoDriverController // This moves elevator up
                                .y()
                                .onTrue(Commands.runOnce(() -> m_robotElevator.setElevatorSpeed(-0.6)))
                                .onFalse(Commands.runOnce(() -> m_robotElevator.setElevatorSpeed(-elevHold)));

                m_CoDriverController // This moves elevator down
                                .a()
                                // .povDown()
                                .onTrue(Commands.runOnce(() -> m_robotElevator.setElevatorSpeed(0.5)))
                                .onFalse(Commands.runOnce(() -> m_robotElevator.setElevatorSpeed(-elevHold)));
                // m_CoDriverController
                // .povUp()
                // .onTrue(Commands.runOnce(() -> m_robotElevator.setSafeWenchPosition()));
                // Elevator PID
                // m_driverController // Reset? elevator
                // .start()
                // .onTrue(Commands.runOnce(() -> m_robotElevator.zeroSensors()));
                // m_driverController // Reset? elevator
                // .back()
                // .onTrue(Commands.runOnce(() -> m_robotElevator.setPositionZero()));
                // m_driverController // Moves elevator to set position
                // .x()
                // .onTrue(Commands.runOnce(() -> m_robotElevator.setElevatorPosition()));
                // // m_CoDriverController
                // .button(3)
                // .onTrue(new MoveElevatorCommand(m_robotElevator));

                // This is for wench movement forward and backward.
                double winchHold = 0.08;
                m_CoDriverController // This moves wench backwards
                                .b()
                                .onTrue(Commands.runOnce(() -> m_robotWench.setWenchSpeed(-0.1)))
                                .onFalse(Commands.runOnce(() -> m_robotWench.setWenchSpeed(winchHold)));

                m_CoDriverController // This moves wench forwards
                                .x()
                                .onTrue(Commands.runOnce(() -> m_robotWench.setWenchSpeed(1)))
                                .onFalse(Commands.runOnce(() -> m_robotWench.setWenchSpeed(winchHold)));
                // m_driverController // This moves wench backwards
                // .b()
                // .onTrue(Commands.runOnce(() -> m_robotWench.setWenchSpeed(-0.1)))
                // .onFalse(Commands.runOnce(() -> m_robotWench.setWenchSpeed(0.1)));

                // m_driverController // This moves wench forwards
                // .x()
                // .onTrue(Commands.runOnce(() -> m_robotWench.setWenchSpeed(1)))
                // .onFalse(Commands.runOnce(() -> m_robotWench.setWenchSpeed(0.1)));

                // Reset claw's encoder logic
                m_driverController
                                // button start+B (red for reset)
                                .back()
                                .and(m_driverController.b())
                                // TODO: determine what reset function to call
                                .onTrue(Commands.runOnce(() -> m_robotClaw.zeroSensors()));

                // Moving claw to pre-set positions
                m_driverController
                                .y() // button Y is yellow, like cones
                                .onTrue(Commands.runOnce(() -> m_robotClaw.goToConePosition()));
                m_driverController
                                .x() // button X is blue/purple, like cubes
                                .onTrue(Commands.runOnce(() -> m_robotClaw.goToCubePosition()));
                m_driverController
                                .a() // button A is green for open
                                .onTrue(Commands.runOnce(() -> m_robotClaw.goToMaxOpenPosition()));
                m_driverController
                                .start()
                                .onTrue(Commands.runOnce(() -> m_robotCamera.nextCameraSelection()));

                // // This is for claw movement to manually open.
                double manualClawSpeed = 0.5;
                m_driverController
                                .povLeft()
                                .onTrue(Commands.runOnce(
                                                () -> m_robotClaw.setClawSpeed(manualClawSpeed)))
                                .onFalse(Commands.runOnce(() -> m_robotClaw.setClawSpeed(0)));
                m_driverController
                                .povRight()
                                .onTrue(Commands.runOnce(
                                                () -> m_robotClaw.setClawSpeed(-manualClawSpeed)))
                                .onFalse(Commands.runOnce(() -> m_robotClaw.setClawSpeed(0)));

                m_CoDriverController
                                .povLeft()
                                .onTrue(Commands.runOnce(
                                                () -> m_upperWench.setUpperWenchSpeed(0.1)))
                                .onFalse(Commands.runOnce(() -> m_upperWench.setUpperWenchSpeed(0)));
                m_CoDriverController
                                .povRight()
                                .onTrue(Commands.runOnce(
                                                () -> m_upperWench.setUpperWenchSpeed(-0.1)))
                                .onFalse(Commands.runOnce(() -> m_upperWench.setUpperWenchSpeed(0)));

                // // This is for claw movement to manually open.
                // m_CoDriverController
                // .button(IOConstants.kCoDriverButtonRightBumber)
                // .onTrue(Commands.runOnce(() -> m_robotClaw.setClawSpeed(0.5)))
                // .onFalse(Commands.runOnce(() -> m_robotClaw.setClawSpeed(0)));
                // // This is for claw movement to manually close.
                // m_CoDriverController
                // .axisGreaterThan(IOConstants.kCoDriverAxisRightTrigger, 0)
                // .onTrue(Commands.runOnce(() -> m_robotClaw.setClawSpeed(-0.5)))
                // .onFalse(Commands.runOnce(() -> m_robotClaw.setClawSpeed(0)));

        }

        public Command getAutonomousCommand() {
                // Drives forward at specific speed at a specific time
                return m_chooser.getSelected();

        }

}