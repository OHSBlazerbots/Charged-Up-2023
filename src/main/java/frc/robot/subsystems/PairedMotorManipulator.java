// Following example from here: https://github.com/CrossTheRoadElec/Phoenix-Examples-Languages/blob/master/Java%20General/PositionClosedLoop/src/main/java/frc/robot/Robot.java
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.ControlMode;
import frc.robot.Constants.DriveConstants;
import com.ctre.phoenix.motorcontrol.SensorTerm;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.RemoteSensorSource;
import com.ctre.phoenix.motorcontrol.FollowerType;

public class PairedMotorManipulator extends SubsystemBase {
        private static final WPI_TalonSRX m_rightMotorPrimary = new WPI_TalonSRX(DriveConstants.kRightMotorPrimaryPort);
        private static final WPI_TalonSRX m_rightMotorSecondary = new WPI_TalonSRX(
                        DriveConstants.kRightMotorSecondaryPort);
        // we need to find these values
        public static double kMaxSpeed = 3.0;

        private double _targetAngle = 0.0;

        public PairedMotorManipulator() {
                // Reset each talon to factory default
                // If we have to swap talons, we want to make sure
                // the new talon is configured properly
                m_rightMotorPrimary.configFactoryDefault();
                m_rightMotorSecondary.configFactoryDefault();

                m_rightMotorPrimary.set(ControlMode.PercentOutput, 0);
                m_rightMotorSecondary.follow(m_rightMotorPrimary);

                // Set all motors to brake mode to prevent coasting
                m_rightMotorPrimary.setNeutralMode(NeutralMode.Brake);
                m_rightMotorSecondary.setNeutralMode(NeutralMode.Brake);

                /* Config the local sensor used for Primary PID and sensor direction */
                m_rightMotorPrimary.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder,
                                DriveConstants.PID_PRIMARY,
                                DriveConstants.kTimeoutMs);

                /* Configure output and sensor direction */
                m_rightMotorPrimary.setInverted(true);
                m_rightMotorSecondary.setInverted(InvertType.FollowMaster);
                m_rightMotorPrimary.setSensorPhase(true);

                /* Config the peak and nominal outputs */
                m_rightMotorPrimary.configNominalOutputForward(0.0, DriveConstants.kTimeoutMs);
                m_rightMotorPrimary.configNominalOutputReverse(0.0, DriveConstants.kTimeoutMs);
                m_rightMotorPrimary.configPeakOutputForward(1.0, DriveConstants.kTimeoutMs);
                m_rightMotorPrimary.configPeakOutputReverse(-1.0, DriveConstants.kTimeoutMs);

                /**
                 * Config the allowable closed-loop error, Closed-Loop output will be
                 * neutral within this range. See Table in Section 17.2.1 for native
                 * units per rotation.
                 */
                m_rightMotorPrimary.configAllowableClosedloopError(0, DriveConstants.PID_PRIMARY,
                                DriveConstants.kTimeoutMs);

                /* Config Position Closed Loop gains in slot0, tsypically kF stays zero. */
                m_rightMotorPrimary.config_kF(DriveConstants.PID_PRIMARY, Constants.kGains.kF, Constants.kTimeoutMs);
                m_rightMotorPrimary.config_kP(DriveConstants.PID_PRIMARY, Constants.kGains.kP, Constants.kTimeoutMs);
                m_rightMotorPrimary.config_kI(DriveConstants.PID_PRIMARY, Constants.kGains.kI, Constants.kTimeoutMs);
                m_rightMotorPrimary.config_kD(DriveConstants.PID_PRIMARY, Constants.kGains.kD, Constants.kTimeoutMs);

                /*
                 * 1ms per loop. PID loop can be slowed down if need be.
                 * For example,
                 * - if sensor updates are too slow
                 * - sensor deltas are very small per update, so derivative error never gets
                 * large enough to be useful.
                 * - sensor movement is very slow causing the derivative error to be near zero.
                 */
                int closedLoopTimeMs = 1;
                m_rightMotorPrimary.configClosedLoopPeriod(0, closedLoopTimeMs, DriveConstants.kTimeoutMs);
                m_rightMotorPrimary.configClosedLoopPeriod(1, closedLoopTimeMs, DriveConstants.kTimeoutMs);

                /* Initialize */
                zeroSensors();
        }

        /**
         * Drives the robot using arcade controls.
         *
         * @param fwd the commanded forward movement
         * @param rot the commanded rotation
         */
        public void arcadeDrive(double fwd, double rot) {

                SmartDashboard.putBoolean("Drive straight mode", _isDriveStraightMode);
                if (_isDriveStraightMode) {
                        encoderStraightDrive(fwd);
                } else {
                        encoderArcadeDrive(fwd, rot);
                }
        }

        public void encoderArcadeDrive(double fwd, double rot) {
                m_rightMotorPrimary.set(ControlMode.PercentOutput, fwd, DemandType.ArbitraryFeedForward, -rot);
        }

        public void initEncoderStraightDrive() {
                System.out.println("This is Drive Straight using the auxiliary feature with " +
                                "the difference between two encoders to maintain current heading.\n");

                /* Determine which slot affects which PID */
                m_rightMotorPrimary.selectProfileSlot(DriveConstants.kSlot_Turning, DriveConstants.PID_TURN);
                _targetAngle = m_rightMotorPrimary.getSelectedSensorPosition(1);
        }

        public void encoderStraightDrive(double fwd) {
                m_rightMotorPrimary.set(ControlMode.PercentOutput, fwd, DemandType.AuxPID, _targetAngle);
        }

        /**
         * Sets the max output of the drive. Useful for scaling the drive to drive more
         * slowly.
         *
         * @param maxOutput the maximum output to which the drive will be constrained
         */
        public void setMaxOutput(double maxOutput) {
                kMaxSpeed = maxOutput;

        }

        void zeroSensors() {
                /**
                 * Grab the 360 degree position of the MagEncoder's absolute
                 * position, and intitally set the relative sensor to match.
                 */
                int absolutePosition = m_rightMotorPrimary.getSensorCollection().getPulseWidthPosition();

                /* Mask out overflows, keep bottom 12 bits */
                absolutePosition &= 0xFFF;
                if (Constants.kSensorPhase) {
                        absolutePosition *= -1;
                }
                if (Constants.kMotorInvert) {
                        absolutePosition *= -1;
                }

                /* Set the quadrature (relative) sensor to match absolute */
                m_rightMotorPrimary.setSelectedSensorPosition(absolutePosition, Constants.kPIDLoopIdx,
                                Constants.kTimeoutMs);

                m_rightMotorPrimary.getSensorCollection().setQuadraturePosition(0, DriveConstants.kTimeoutMs);
                System.out.println("[Quadrature Encoders] All sensors are zeroed.\n");
        }
}
