package frc.robot.subsystems;

import frc.robot.Constants.ClawConstants;
import frc.robot.subsystems.motor_controllers.PositionPidMotorController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClawSubsystem extends SubsystemBase {
    private static final PositionPidMotorController m_clawController = new PositionPidMotorController(
            ClawConstants.kClawGains,
            ClawConstants.kClawMotorPort);

    public ClawSubsystem() {
        // TODO: determine if both calls are needed
        zeroSensors();
        setPositionZero();

        // Current limiting
        int TIMEOUT_MS = 10;
        m_clawController.getMotor().configPeakCurrentLimit(20, TIMEOUT_MS); // 20 Amps
        m_clawController.getMotor().configPeakCurrentDuration(200, TIMEOUT_MS); // 200ms
        m_clawController.getMotor().configContinuousCurrentLimit(20, TIMEOUT_MS); // 20 Amps
    }

    public void zeroSensors() {
        m_clawController.zeroSensors();
    }

    public void setPositionZero() {
        m_clawController.setPositionZero();
    }

    public void setClawPosition(double targetPosition) {
        double motorPosition = targetPosition;
        m_clawController.goToPosition(motorPosition);
        writeMetricsToSmartDashboard();
    }

    /**
     * For manual control of motor
     * 
     * @param speed
     */
    public void setClawSpeed(double speed) {
        m_clawController.setOutput(speed);
        writeMetricsToSmartDashboard();
    }

    public void writeMetricsToSmartDashboard() {
        SmartDashboard.putNumber("Claw Relative Position", m_clawController.getCurrentRelativePosition());
        SmartDashboard.putNumber("Claw Absolute Position", m_clawController.getCurrentAbsolutePosition());
        SmartDashboard.putNumber("Claw Goal", m_clawController.getTargetPosition());
        SmartDashboard.putNumber("Claw Motor set output", m_clawController.getOutput());
    }

    public void goToConePosition() {
        setClawPosition(ClawConstants.kClawConePosition);
    }

    public void goToCubePosition() {
        setClawPosition(ClawConstants.kClawCubePosition);
    }

    public void goToMaxOpenPosition() {
        setClawPosition(ClawConstants.kClawEncoderRotationsAtMaxExtension);
    }
}
