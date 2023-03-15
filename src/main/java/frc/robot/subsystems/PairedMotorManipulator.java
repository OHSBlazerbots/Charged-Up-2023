// Following example from here: https://github.com/CrossTheRoadElec/Phoenix-Examples-Languages/blob/master/Java%20General/PositionClosedLoop/src/main/java/frc/robot/Robot.java
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.ControlMode;

import frc.robot.Gains;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.InvertType;

public class PairedMotorManipulator {
        private static final int PID_PRIMARY = 0;
        private static final int TIMEOUT_MS = 30;

        private final WPI_TalonSRX m_motorPrimary;
        private final WPI_TalonSRX m_motorSecondary;

        private final Gains _kGains;
        private double maxSpeed = 0.5;
        private double targetRotations = 0.0;

        public PairedMotorManipulator(Gains gains, int primaryPort, int secondaryPort) {
                _kGains = gains;

                m_motorPrimary = new WPI_TalonSRX(primaryPort);
                m_motorSecondary = new WPI_TalonSRX(
                                secondaryPort);

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
                m_motorPrimary.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, PID_PRIMARY, TIMEOUT_MS);

                /* Configure output and sensor direction */
                m_motorPrimary.setInverted(true);
                m_motorSecondary.setInverted(InvertType.FollowMaster);
                m_motorPrimary.setSensorPhase(true);

                /* Config the peak and nominal outputs */
                m_motorPrimary.configNominalOutputForward(0.0, TIMEOUT_MS);
                m_motorPrimary.configNominalOutputReverse(0.0, TIMEOUT_MS);
                m_motorPrimary.configPeakOutputForward(1.0, TIMEOUT_MS);
                m_motorPrimary.configPeakOutputReverse(-1.0, TIMEOUT_MS);

                /**
                 * Config the allowable closed-loop error, Closed-Loop output will be
                 * neutral within this range. See Table in Section 17.2.1 for native
                 * units per rotation.
                 */
                m_motorPrimary.configAllowableClosedloopError(0, PID_PRIMARY, TIMEOUT_MS);

                /*
                 * Config Position Closed Loop gains for primary PID, tsypically kF stays zero.
                 */
                m_motorPrimary.config_kF(PID_PRIMARY, _kGains.kF, TIMEOUT_MS);
                m_motorPrimary.config_kP(PID_PRIMARY, _kGains.kP, TIMEOUT_MS);
                m_motorPrimary.config_kI(PID_PRIMARY, _kGains.kI, TIMEOUT_MS);
                m_motorPrimary.config_kD(PID_PRIMARY, _kGains.kD, TIMEOUT_MS);

                /*
                 * 1ms per loop. PID loop can be slowed down if need be.
                 * For example,
                 * - if sensor updates are too slow
                 * - sensor deltas are very small per update, so derivative error never gets
                 * large enough to be useful.
                 * - sensor movement is very slow causing the derivative error to be near zero.
                 */
                int closedLoopTimeMs = 1;
                m_motorPrimary.configClosedLoopPeriod(0, closedLoopTimeMs, TIMEOUT_MS);
                m_motorPrimary.configClosedLoopPeriod(1, closedLoopTimeMs, TIMEOUT_MS);

                /* Initialize */
                zeroSensors();
        }

        /**
         * Rotates the motors to achive the desired position on the encoders.
         * TODO: determine if this should be called from a loop
         *
         * @param position the commanded position
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
                maxSpeed = maxOutput;
                // TODO: prevent setting outside range (0, +1)
                // TODO: make this actually affect motors

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
                m_motorPrimary.setSelectedSensorPosition(absolutePosition, PID_PRIMARY,
                                TIMEOUT_MS);
        }

        void setPositionZero() {
                m_motorPrimary.getSensorCollection().setQuadraturePosition(0, TIMEOUT_MS);
                System.out.println("[Quadrature Encoders] All sensors are zeroed.\n");
        }
}
