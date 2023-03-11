// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import com.ctre.phoenix.motorcontrol.DemandType;
//import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.ControlMode;
import frc.robot.Constants.DriveConstants;
import com.ctre.phoenix.motorcontrol.SensorTerm;
import com.ctre.phoenix.motorcontrol.StatusFrame;
//import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.RemoteSensorSource;

public class DriveSubsystem extends SubsystemBase {
        private static final WPI_TalonSRX m_leftMotorPrimary = new WPI_TalonSRX(DriveConstants.kLeftMotorPrimaryPort);
        private static final WPI_TalonSRX m_leftMotorSecondary = new WPI_TalonSRX(
                        DriveConstants.kLeftMotorSecondaryPort);
        private static final WPI_TalonSRX m_rightMotorPrimary = new WPI_TalonSRX(DriveConstants.kRightMotorPrimaryPort);
        private static final WPI_TalonSRX m_rightMotorSecondary = new WPI_TalonSRX(
                        DriveConstants.kRightMotorSecondaryPort);
        // we need to find these values
        public static double kMaxSpeed = 3.0; // meters per second
        public static final double kMaxAngularSpeed = 2 * Math.PI; // one rotation per second
        private static final double kTrackWidth = 0.381 * 2; // meters
        private static final double kWheelRadius = 0.0508; // meter
        private static final int kEncoderResolution = 4096;
        private final Encoder m_leftEncoder = new Encoder(0, 1);
        private final Encoder m_rightEncoder = new Encoder(2, 3);
        // private final AnalogGyro m_gyro = new AnalogGyro(0);
        // we need to find these values
        private final PIDController m_leftPIDController = new PIDController(1, 0, 0);
        private final PIDController m_rightPIDController = new PIDController(1, 0, 0);
        // The motors on the left side of the drive.
        private final MotorControllerGroup m_leftMotors = new MotorControllerGroup(
                        m_leftMotorPrimary,
                        m_leftMotorSecondary);

        // The motors on the right side of the drive.
        private final MotorControllerGroup m_rightMotors = new MotorControllerGroup(
                        m_rightMotorPrimary,
                        m_rightMotorSecondary);

        // The robot's drive

        private final DifferentialDriveKinematics m_kinematics = new DifferentialDriveKinematics(kTrackWidth);

        // private final DifferentialDriveOdometry m_odometry;

        // Gains are for example purposes only - must be determined for your own robot!
        private final SimpleMotorFeedforward m_feedforward = new SimpleMotorFeedforward(1, 3);

        public DriveSubsystem() {
                // m_gyro.reset();
                // We need to invert one side of the drivetrain so that positive voltages
                // result in both sides moving forward. Depending on how your robot's
                // gearbox is constructed, you might have to invert the left side instead.
                // m_rightMotors.setInverted(true);
                // wpi.java.configureTestTasks(test)

                // Set the distance per pulse for the drive encoders. We can simply use the
                // distance traveled for one rotation of the wheel divided by the encoder
                // resolution.
                /**
                 * m_leftEncoder.setDistancePerPulse(2 * Math.PI * kWheelRadius /
                 * kEncoderResolution);
                 * m_rightEncoder.setDistancePerPulse(2 * Math.PI * kWheelRadius /
                 * kEncoderResolution);
                 * 
                 * m_leftEncoder.reset();
                 * m_rightEncoder.reset();
                 **/
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

                // m_odometry = new DifferentialDriveOdometry(
                // m_gyro.getRotation2d(), m_leftEncoder.getDistance(),
                // m_rightEncoder.getDistance());
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
                                DriveConstants.kTimeoutMs);
                m_rightMotorPrimary.setInverted(true);
                m_leftMotorPrimary.setInverted(false);
                m_leftMotorSecondary.setInverted(InvertType.FollowMaster);
                m_rightMotorSecondary.setInverted(InvertType.FollowMaster);
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
                                                                                                                       // speed
                                                                                                                       // up

                /* Configure neutral deadband */
                m_rightMotorPrimary.configNeutralDeadband(DriveConstants.kNeutralDeadband, DriveConstants.kTimeoutMs);
                m_leftMotorPrimary.configNeutralDeadband(DriveConstants.kNeutralDeadband, DriveConstants.kTimeoutMs);

                /*
                 * max out the peak output (for all modes). However you can
                 * limit the output of a given PID object with configClosedLoopPeakOutput().
                 */
                m_leftMotorPrimary.configPeakOutputForward(+1.0, DriveConstants.kTimeoutMs);
                m_leftMotorPrimary.configPeakOutputReverse(-1.0, DriveConstants.kTimeoutMs);
                m_rightMotorPrimary.configPeakOutputForward(+1.0, DriveConstants.kTimeoutMs);
                m_rightMotorPrimary.configPeakOutputReverse(-1.0, DriveConstants.kTimeoutMs);

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
                                DriveConstants.kGains_Turning.kPeakOutput, DriveConstants.kTimeoutMs);
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
        }

        public void setSpeeds(DifferentialDriveWheelSpeeds speeds) {
                final double leftFeedforward = m_feedforward.calculate(speeds.leftMetersPerSecond);
                final double rightFeedforward = m_feedforward.calculate(speeds.rightMetersPerSecond);

                final double leftOutput = m_leftPIDController.calculate(m_leftEncoder.getRate(),
                                speeds.leftMetersPerSecond);
                final double rightOutput = m_rightPIDController.calculate(m_rightEncoder.getRate(),
                                speeds.rightMetersPerSecond);
                m_leftMotors.setVoltage(leftOutput + leftFeedforward);
                m_rightMotors.setVoltage(rightOutput + rightFeedforward);
        }

        /**
         * Drives the robot using arcade controls.
         *
         * @param fwd the commanded forward movement
         * @param rot the commanded rotation
         */
        public void arcadeDrive(double fwd, double rot) {
                var wheelSpeeds = m_kinematics.toWheelSpeeds(new ChassisSpeeds(fwd, 0.0, rot));
                setSpeeds(wheelSpeeds);
        }

        public void encoderArcadeDrive(double fwd, double rot) {
                m_leftMotorPrimary.set(ControlMode.PercentOutput, fwd, DemandType.ArbitraryFeedForward, rot);
                m_rightMotorPrimary.set(ControlMode.PercentOutput, fwd, DemandType.ArbitraryFeedForward, rot);

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

        // public void updateOdometry() {
        // m_odometry.update(
        // m_gyro.getRotation2d(), m_leftEncoder.getDistance(),
        // m_rightEncoder.getDistance());
        // }
        void zeroSensors() {
                m_leftMotorPrimary.getSensorCollection().setQuadraturePosition(0, DriveConstants.kTimeoutMs);
                m_rightMotorPrimary.getSensorCollection().setQuadraturePosition(0, DriveConstants.kTimeoutMs);
                System.out.println("[Quadrature Encoders] All sensors are zeroed.\n");
        }
}
