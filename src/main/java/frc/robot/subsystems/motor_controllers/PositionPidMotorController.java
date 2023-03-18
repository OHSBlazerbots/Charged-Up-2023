// Following example from here: https://github.com/CrossTheRoadElec/Phoenix-Examples-Languages/blob/master/Java%20General/PositionClosedLoop/src/main/java/frc/robot/Robot.java
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.motor_controllers;

import frc.robot.Gains;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;

public class PositionPidMotorController {
        private static final int PID_PRIMARY = 0;
        private static final int TIMEOUT_MS = 30;

        private final WPI_TalonSRX m_motorPrimary;

        private final Gains _kGains;
        private double maxSpeed = 0.5;
        private double targetRotations = 0.0;
        private double appliedOutput;

        public PositionPidMotorController(Gains gains, int primaryMotorPort) {
                _kGains = gains;

                m_motorPrimary = new WPI_TalonSRX(primaryMotorPort);

                // Reset each talon to factory default
                // If we have to swap talons, we want to make sure
                // the new talon is configured properly
                m_motorPrimary.configFactoryDefault();

                m_motorPrimary.set(ControlMode.PercentOutput, 0);

                // Set all motors to brake mode to prevent coasting
                m_motorPrimary.setNeutralMode(NeutralMode.Brake);

                /* Config the local sensor used for Primary PID and sensor direction */
                m_motorPrimary.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, PID_PRIMARY, TIMEOUT_MS);

                /* Configure output and sensor direction */
                m_motorPrimary.setInverted(false);
                m_motorPrimary.setSensorPhase(false);

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
                m_motorPrimary.configAllowableClosedloopError(PID_PRIMARY, 0, TIMEOUT_MS);

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
         * Set secondary motor to follow the signal of the primary motor.
         * Used for high-power configurations where mulitple motors drive a single
         * gearbox.
         * Only the primary motor is connected to the encoder/feedback device.
         * 
         * @param followerMotor Motor to follow the primary motor
         */
        protected void configureFollower(WPI_TalonSRX followerMotor) {
                followerMotor.configFactoryDefault();
                followerMotor.follow(m_motorPrimary);
                followerMotor.setNeutralMode(NeutralMode.Brake);
                followerMotor.setInverted(InvertType.FollowMaster);
        }

        public double getCurrentAbsolutePosition() {
                // TODO: determine if this is correct
                double absolutePosition = m_motorPrimary.getSensorCollection().getPulseWidthPosition();
                return absolutePosition;
        }

        public double getCurrentRelativePosition() {
                // TODO: determine if this is correct
                double relativePosition = m_motorPrimary.getSelectedSensorPosition(PID_PRIMARY);
                return relativePosition;
        }

        /**
         * Rotates the motors to achive the desired position on the encoders.
         * TODO: determine if this should be called from a loop (like arcadeDrive)
         *
         * @param position the commanded position
         */
        public void goToPosition(double position) {
                targetRotations = position;
                m_motorPrimary.set(ControlMode.Position, targetRotations);
        }

        public double getTargetPosition() {
                return targetRotations;
        }

        /**
         * Rotates the motors at the specified speed.
         * Used for manual control and testing, ignoring encoders.
         * WARNING: be careful when switching back to goToPosition(), unless position
         * has been zeroed.
         *
         * @param output the commanded motor output, between [-1, 1]
         */
        public void setOutput(double output) {
                appliedOutput = output;
                m_motorPrimary.set(ControlMode.PercentOutput, output);
        }

        public double getOutput() {
                return appliedOutput;
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

        public void zeroSensors() {
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

        public void setPositionZero() {
                m_motorPrimary.getSensorCollection().setQuadraturePosition(0, TIMEOUT_MS);
                System.out.println("[Quadrature Encoders] All sensors are zeroed.\n");
        }
}
