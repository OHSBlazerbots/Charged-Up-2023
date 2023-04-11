package frc.robot.subsystems.motor_controllers;

import frc.robot.Gains;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

public class PositionPidDualMotorController extends PositionPidMotorController {
        private final WPI_TalonSRX m_motorSecondary;

        public PositionPidDualMotorController(Gains gains, int primaryMotorPort, int secondaryMotorPort) {
                super(gains, primaryMotorPort);

                m_motorSecondary = new WPI_TalonSRX(secondaryMotorPort);
                configureFollower(m_motorSecondary);
        }
}
