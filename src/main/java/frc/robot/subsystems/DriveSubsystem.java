// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import frc.robot.Constants.DriveConstants;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveSubsystem extends SubsystemBase {
    private static final WPI_TalonSRX m_leftMotor1 = new WPI_TalonSRX(DriveConstants.kLeftMotor1Port);
    private static final WPI_TalonSRX m_leftMotor2 = new WPI_TalonSRX(DriveConstants.kLeftMotor2Port);
    private static final WPI_TalonSRX m_rightMotor1 = new WPI_TalonSRX(DriveConstants.kRightMotor1Port);
    private static final WPI_TalonSRX m_rightMotor2 = new WPI_TalonSRX(DriveConstants.kRightMotor2Port);

    // The motors on the left side of the drive.
    private final MotorControllerGroup m_leftMotors = new MotorControllerGroup(
            m_leftMotor1,
            m_leftMotor2);

    // The motors on the right side of the drive.
    private final MotorControllerGroup m_rightMotors = new MotorControllerGroup(
            m_rightMotor1,
            m_rightMotor2);

    // The robot's drive
    private final DifferentialDrive m_drive = new DifferentialDrive(m_leftMotors, m_rightMotors);

    /** Creates a new DriveSubsystem. */
    public DriveSubsystem() {
        // We need to invert one side of the drivetrain so that positive voltages
        // result in both sides moving forward. Depending on how your robot's
        // gearbox is constructed, you might have to invert the left side instead.
        m_rightMotors.setInverted(true);

        // Reset each talon to factory default
        // If we have to swap talons, we want to make sure
        // the new talon is configured properly
        m_leftMotor1.configFactoryDefault();
        m_leftMotor2.configFactoryDefault();
        m_rightMotor1.configFactoryDefault();
        m_rightMotor2.configFactoryDefault();

        // Set all motors to brake mode to prevent coasting
        m_leftMotor1.setNeutralMode(NeutralMode.Brake);
        m_leftMotor2.setNeutralMode(NeutralMode.Brake);
        m_rightMotor1.setNeutralMode(NeutralMode.Brake);
        m_rightMotor2.setNeutralMode(NeutralMode.Brake);
    }

    /**
     * Drives the robot using arcade controls.
     *
     * @param fwd the commanded forward movement
     * @param rot the commanded rotation
     */
    public void arcadeDrive(double fwd, double rot) {
        m_drive.arcadeDrive(fwd, rot);
    }

    /**
     * Sets the max output of the drive. Useful for scaling the drive to drive more
     * slowly.
     *
     * @param maxOutput the maximum output to which the drive will be constrained
     */
    public void setMaxOutput(double maxOutput) {
        m_drive.setMaxOutput(maxOutput);
    }
}
