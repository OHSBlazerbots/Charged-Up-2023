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

public class DriveSubsystem extends SubsystemBase {
        private static final WPI_TalonSRX m_leftMotorPrimary = new WPI_TalonSRX(DriveConstants.kLeftMotorPrimaryPort);
        private static final WPI_TalonSRX m_leftMotorSecondary = new WPI_TalonSRX(
                        DriveConstants.kLeftMotorSecondaryPort);
        private static final WPI_TalonSRX m_rightMotorPrimary = new WPI_TalonSRX(DriveConstants.kRightMotorPrimaryPort);
        private static final WPI_TalonSRX m_rightMotorSecondary = new WPI_TalonSRX(
                        DriveConstants.kRightMotorSecondaryPort);
        // we need to find these values
        public static double kMaxSpeed = 3.0;

        private double _targetAngle = 0.0;
        private static boolean _isDriveStraightMode = false;

        public void toggleDriveMode() {
                _isDriveStraightMode = !_isDriveStraightMode;

                // Initialize based on the mode
                if (_isDriveStraightMode) {
                        initEncoderStraightDrive();
                }
        }

        public void setDriveStraight() {
                if (!_isDriveStraightMode) {
                        toggleDriveMode();
                }
        }

        public DriveSubsystem() {
                // Reset each talon to factory default
                // If we have to swap talons, we want to make sure
                // the new talon is configured properly
                m_leftMotorPrimary.configFactoryDefault();
                m_leftMotorSecondary.configFactoryDefault();
                m_rightMotorPrimary.configFactoryDefault();
                m_rightMotorSecondary.configFactoryDefault();

                m_rightMotorPrimary.set(ControlMode.PercentOutput, 0);
                m_leftMotorPrimary.set(ControlMode.PercentOutput, 0);

                m_rightMotorSecondary.follow(m_rightMotorPrimary);
                m_leftMotorSecondary.follow(m_leftMotorPrimary);

                // Set all motors to brake mode to prevent coasting
                m_leftMotorPrimary.setNeutralMode(NeutralMode.Brake);
                m_leftMotorSecondary.setNeutralMode(NeutralMode.Brake);
                m_rightMotorPrimary.setNeutralMode(NeutralMode.Brake);
                m_rightMotorSecondary.setNeutralMode(NeutralMode.Brake);

                m_leftMotorPrimary.configPeakOutputForward(1.0, DriveConstants.kTimeoutMs);
                m_leftMotorSecondary.configPeakOutputReverse(1.0, DriveConstants.kTimeoutMs);
                m_rightMotorPrimary.configPeakOutputForward(1.0, DriveConstants.kTimeoutMs);
                m_rightMotorPrimary.configPeakOutputReverse(1.0, DriveConstants.kTimeoutMs);

                /*
                 * Configure the drivetrain's left side Feedback Sensor as a Quadrature Encoder
                 */
                m_leftMotorPrimary.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, // Local Feedback Source
                                DriveConstants.PID_PRIMARY, // PID Slot for Source [0, 1]
                                DriveConstants.kTimeoutMs); // Configuration Timeout

                /*
                 * Configure the left Talon's Selected Sensor to be a remote sensor for the
                 * right Talon
                 */
                m_rightMotorPrimary.configRemoteFeedbackFilter(m_leftMotorPrimary.getDeviceID(), // Device ID of Source
                                RemoteSensorSource.TalonSRX_SelectedSensor, // Remote Feedback Source
                                DriveConstants.REMOTE_0, // Source number [0, 1]
                                DriveConstants.kTimeoutMs); // Configuration Timeout

                /*
                 * Setup difference signal to be used for turn when performing Drive Straight
                 * with encoders
                 */
                m_rightMotorPrimary.configSensorTerm(SensorTerm.Diff1, FeedbackDevice.RemoteSensor0,
                                DriveConstants.kTimeoutMs); // Feedback
                                                            // Device
                                                            // of
                                                            // Remote
                                                            // Talon
                m_rightMotorPrimary.configSensorTerm(SensorTerm.Diff0, FeedbackDevice.QuadEncoder,
                                DriveConstants.kTimeoutMs); // Quadrature
                                                            // Encoder
                                                            // of
                                                            // current
                                                            // Talon

                /*
                 * Difference term calculated by right Talon configured to be selected sensor of
                 * turn PID
                 */
                m_rightMotorPrimary.configSelectedFeedbackSensor(FeedbackDevice.SensorDifference,
                                DriveConstants.PID_TURN,
                                DriveConstants.kTimeoutMs);

                /* Scale the Feedback Sensor using a coefficient */
                /**
                 * Heading units should be scaled to ~4000 per 360 deg, due to the following
                 * limitations...
                 * - Target param for aux PID1 is 18bits with a range of [-131072,+131072]
                 * units.
                 * - Target for aux PID1 in motion profile is 14bits with a range of
                 * [-8192,+8192] units.
                 * ... so at 3600 units per 360', that ensures 0.1 degree precision in firmware
                 * closed-loop
                 * and motion profile trajectory points can range +-2 rotations.
                 */
                m_rightMotorPrimary.configSelectedFeedbackCoefficient(
                                DriveConstants.kTurnTravelUnitsPerRotation / DriveConstants.kEncoderUnitsPerRotation, // Coefficient
                                DriveConstants.PID_TURN, // PID Slot of Source
                                DriveConstants.kTimeoutMs); // Configuration Timeout

                /* Configure output and sensor direction */
                m_leftMotorPrimary.setInverted(false);
                m_leftMotorSecondary.setInverted(InvertType.FollowMaster);
                m_leftMotorPrimary.setSensorPhase(true);
                m_rightMotorPrimary.setInverted(true);
                m_rightMotorSecondary.setInverted(InvertType.FollowMaster);
                m_rightMotorPrimary.setSensorPhase(true);

                /* Set status frame periods */
                m_rightMotorPrimary.setStatusFramePeriod(StatusFrame.Status_12_Feedback1, 20,
                                DriveConstants.kTimeoutMs);
                m_rightMotorPrimary.setStatusFramePeriod(StatusFrame.Status_14_Turn_PIDF1, 20,
                                DriveConstants.kTimeoutMs);
                m_leftMotorPrimary.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 5, DriveConstants.kTimeoutMs); // Used
                // remotely
                // by
                // right
                // Talon,
                // speed up

                /* Configure neutral deadband */
                m_rightMotorPrimary.configNeutralDeadband(DriveConstants.kNeutralDeadband, DriveConstants.kTimeoutMs);
                m_leftMotorPrimary.configNeutralDeadband(DriveConstants.kNeutralDeadband, DriveConstants.kTimeoutMs);

                /*
                 * max out the peak output (for all modes). However you can
                 * limit the output of a given PID object with configClosedLoopPeakOutput().
                 */
                m_leftMotorPrimary.configPeakOutputForward(+0.3, DriveConstants.kTimeoutMs);
                m_leftMotorPrimary.configPeakOutputReverse(-0.3, DriveConstants.kTimeoutMs);
                m_rightMotorPrimary.configPeakOutputForward(+0.3, DriveConstants.kTimeoutMs);
                m_rightMotorPrimary.configPeakOutputReverse(-0.3, DriveConstants.kTimeoutMs);

                /* FPID Gains for turn servo */
                m_rightMotorPrimary.config_kP(DriveConstants.kSlot_Turning, DriveConstants.kGains_Turning.kP,
                                DriveConstants.kTimeoutMs);
                m_rightMotorPrimary.config_kI(DriveConstants.kSlot_Turning, DriveConstants.kGains_Turning.kI,
                                DriveConstants.kTimeoutMs);
                m_rightMotorPrimary.config_kD(DriveConstants.kSlot_Turning, DriveConstants.kGains_Turning.kD,
                                DriveConstants.kTimeoutMs);
                m_rightMotorPrimary.config_kF(DriveConstants.kSlot_Turning, DriveConstants.kGains_Turning.kF,
                                DriveConstants.kTimeoutMs);
                m_rightMotorPrimary.config_IntegralZone(DriveConstants.kSlot_Turning,
                                DriveConstants.kGains_Turning.kIzone,
                                DriveConstants.kTimeoutMs);
                m_rightMotorPrimary.configClosedLoopPeakOutput(DriveConstants.kSlot_Turning,
                                DriveConstants.kGains_Turning.kPeakOutput,
                                DriveConstants.kTimeoutMs);
                m_rightMotorPrimary.configAllowableClosedloopError(DriveConstants.kSlot_Turning, 0,
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
                m_rightMotorPrimary.configClosedLoopPeriod(0, closedLoopTimeMs, DriveConstants.kTimeoutMs);
                m_rightMotorPrimary.configClosedLoopPeriod(1, closedLoopTimeMs, DriveConstants.kTimeoutMs);

                /*
                 * configAuxPIDPolarity(boolean invert, int timeoutMs)
                 * false means talon's local output is PID0 + PID1, and other side Talon is PID0
                 * - PID1
                 * true means talon's local output is PID0 - PID1, and other side Talon is PID0
                 * + PID1
                 */
                m_rightMotorPrimary.configAuxPIDPolarity(false, DriveConstants.kTimeoutMs);

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
                m_leftMotorPrimary.set(ControlMode.PercentOutput, fwd, DemandType.ArbitraryFeedForward, +rot);
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
                /*
                 * Configured for percentOutput with Auxiliary PID on Quadrature Encoders'
                 * Difference
                 */
                // TODO: try .follow() before .set()
                m_rightMotorPrimary.set(ControlMode.PercentOutput, fwd, DemandType.AuxPID, _targetAngle);
                m_leftMotorPrimary.follow(m_rightMotorPrimary, FollowerType.AuxOutput1);
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
                m_leftMotorPrimary.getSensorCollection().setQuadraturePosition(0, DriveConstants.kTimeoutMs);
                m_rightMotorPrimary.getSensorCollection().setQuadraturePosition(0, DriveConstants.kTimeoutMs);
                System.out.println("[Quadrature Encoders] All sensors are zeroed.\n");
        }
}
