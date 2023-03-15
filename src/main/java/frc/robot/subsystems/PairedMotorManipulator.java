// Following example from here: https://github.com/CrossTheRoadElec/Phoenix-Examples-Languages/blob/master/Java%20General/PositionClosedLoop/src/main/java/frc/robot/Robot.java
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.ControlMode;
import frc.robot.Constants.DriveConstants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.InvertType;

public class PairedMotorManipulator extends SubsystemBase {
        private static final WPI_TalonSRX m_motorPrimary = new WPI_TalonSRX(DriveConstants.kMotorPrimaryPort);
        private static final WPI_TalonSRX m_motorSecondary = new WPI_TalonSRX(
                        DriveConstants.kMotorSecondaryPort);

        public static double kMaxSpeed = 0.5;

        private double targetRotations = 0.0;

        public PairedMotorManipulator() {
                // Reset each talon to factory default
                // If we have to swap talons, we want to make sure
                // the new talon is configured properly
                m_motorPrimary.configFactoryDefault();
                m_motorSecondary.configFactoryDefault();

                m_motorPrimary.set(ControlMode.PercentOutput, 0);
                m_motorSecondary.follow(m_motorPrimary);

                // Set all motors to brake mode to prevent coasting
                m_motorPrimary.setNeutralMode(NeutralMode.Brake);
                m_motorSecondary.setNeutralMode(NeutralMode.Brake);

                /* Config the local sensor used for Primary PID and sensor direction */
                m_motorPrimary.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder,
                                DriveConstants.PID_PRIMARY,
                                DriveConstants.kTimeoutMs);

                /* Configure output and sensor direction */
                m_motorPrimary.setInverted(true);
                m_motorSecondary.setInverted(InvertType.FollowMaster);
                m_motorPrimary.setSensorPhase(true);

                /* Config the peak and nominal outputs */
                m_motorPrimary.configNominalOutputForward(0.0, DriveConstants.kTimeoutMs);
                m_motorPrimary.configNominalOutputReverse(0.0, DriveConstants.kTimeoutMs);
                m_motorPrimary.configPeakOutputForward(1.0, DriveConstants.kTimeoutMs);
                m_motorPrimary.configPeakOutputReverse(-1.0, DriveConstants.kTimeoutMs);

                /**
                 * Config the allowable closed-loop error, Closed-Loop output will be
                 * neutral within this range. See Table in Section 17.2.1 for native
                 * units per rotation.
                 */
                m_motorPrimary.configAllowableClosedloopError(0, DriveConstants.PID_PRIMARY,
                                DriveConstants.kTimeoutMs);

                /*
                 * Config Position Closed Loop gains for primary PID, tsypically kF stays zero.
                 */
                m_motorPrimary.config_kF(DriveConstants.PID_PRIMARY, Constants.kGains.kF,
                                DriveConstants.kTimeoutMs);
                m_motorPrimary.config_kP(DriveConstants.PID_PRIMARY, Constants.kGains.kP,
                                DriveConstants.kTimeoutMs);
                m_motorPrimary.config_kI(DriveConstants.PID_PRIMARY, Constants.kGains.kI,
                                DriveConstants.kTimeoutMs);
                m_motorPrimary.config_kD(DriveConstants.PID_PRIMARY, Constants.kGains.kD,
                                DriveConstants.kTimeoutMs);

                /*
                 * 1ms per loop. PID loop can be slowed down if need be.
                 * For example,
                 * - if sensor updates are too slow
                 * - sensor deltas are very small per update, so derivative error never gets
                 * large enough to be useful.
                 * - sensor movement is very slow causing the derivative error to be near zero.
                 */
                int closedLoopTimeMs = 1;
                m_motorPrimary.configClosedLoopPeriod(0, closedLoopTimeMs, DriveConstants.kTimeoutMs);
                m_motorPrimary.configClosedLoopPeriod(1, closedLoopTimeMs, DriveConstants.kTimeoutMs);

                /* Initialize */
                zeroSensors();
        }

        /**
         * Rotates the motors to achive the desired position on the encoders.
         * TODO: determine if this should be called from a loop
         *
         * @param fwd the commanded forward movement
         * @param rot the commanded rotation
         */
        public void goToPosition(double position) {
                targetRotations = position;
                SmartDashboard.putNumber("Goal", targetRotations);
                m_motorPrimary.set(ControlMode.Position, targetRotations);
        }

        /**
         * Sets the max output of the drive. Useful for scaling the drive to drive more
         * slowly.
         *
         * @param maxOutput the maximum output to which the drive will be constrained
         */
        public void setMaxOutput(double maxOutput) {
                kMaxSpeed = maxOutput;
                // TODO: prevent setting outside range (0, +1)

        }

        void zeroSensors() {
                /**
                 * Grab the 360 degree position of the MagEncoder's absolute
                 * position, and intitally set the relative sensor to match.
                 */
                int absolutePosition = m_motorPrimary.getSensorCollection().getPulseWidthPosition();

                /* Mask out overflows, keep bottom 12 bits */
                absolutePosition &= 0xFFF;

                // negate absolutePosition if (motorInverted==true) XOR (sensorPhase==true)
                // TODO: code to ensure this

                /* Set the quadrature (relative) sensor to match absolute */
                m_motorPrimary.setSelectedSensorPosition(absolutePosition, DriveConstants.PID_PRIMARY,
                                DriveConstants.kTimeoutMs);
        }

        void setPositionZero() {
                m_motorPrimary.getSensorCollection().setQuadraturePosition(0, DriveConstants.kTimeoutMs);
                System.out.println("[Quadrature Encoders] All sensors are zeroed.\n");
        }
}
