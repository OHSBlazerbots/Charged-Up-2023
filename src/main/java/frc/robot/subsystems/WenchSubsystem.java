package frc.robot.subsystems;

import frc.robot.Constants.WenchConstants;
import frc.robot.subsystems.motor_controllers.PositionPidDualMotorController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class WenchSubsystem extends SubsystemBase {
    private static final PositionPidDualMotorController m_wenchController = new PositionPidDualMotorController(
            WenchConstants.kWenchGains,
            WenchConstants.kWenchPrimaryMotorPort,
            WenchConstants.kWenchSecondaryMotorPort);

    public WenchSubsystem() {
        // TODO: determine if both calls are needed
        zeroSensors();
        setPositionZero();
    }

    public void zeroSensors() {
        m_wenchController.zeroSensors();
    }

    public void setPositionZero() {
        m_wenchController.setPositionZero();
    }

    public void setWenchPosition(double targetPosition) {
        double motorPosition = targetPosition;
        m_wenchController.goToPosition(motorPosition);
        writeMetricsToSmartDashboard();
    }

    /**
     * For manual control of motor
     * 
     * @param speed
     */
    public void setWenchSpeed(double speed) {
        m_wenchController.setOutput(speed);
        writeMetricsToSmartDashboard();
    }

    public void writeMetricsToSmartDashboard() {
        SmartDashboard.putNumber("Wench Relative Position", m_wenchController.getCurrentRelativePosition());
        SmartDashboard.putNumber("Wench Absolute Position", m_wenchController.getCurrentAbsolutePosition());
        SmartDashboard.putNumber("Wench Goal", m_wenchController.getTargetPosition());
        SmartDashboard.putNumber("Wench Motor set output", m_wenchController.getOutput());
    }

    // public void goToMaxOpenPosition() {
    // setWenchPosition(WenchConstants.kWenchEncoderRotationsAtMaxExtension);
    // }
}
