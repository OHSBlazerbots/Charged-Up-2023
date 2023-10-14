package frc.robot.subsystems;

import frc.robot.Constants.UpperWenchConstants;
import frc.robot.subsystems.motor_controllers.PositionPidMotorController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class UpperWenchSubsystem extends SubsystemBase {
   private static final PositionPidMotorController m_upperController = new PositionPidMotorController(
         UpperWenchConstants.kElevGains,
         UpperWenchConstants.kElevMotorPort);

   public UpperWenchSubsystem() {
      m_upperController.zeroSensors();
   }

   public void setUpperWenchPosition(double targetPosition) {
      double motorPosition = targetPosition;
      m_upperController.goToPosition(motorPosition);
      writeMetricsToSmartDashboard();
   }

   public void setSafeWenchPosition() {
      setUpperWenchPosition(UpperWenchConstants.upperatorSafeWenchPosition);
   }

   public void setUpperWenchSpeed(double speed) {
      // TODO: test this logic!!!
      // stops elavator when it reaches the relative max height to prevent breakage
      // if (m_upperController.getCurrentAbsolutePosition() >=
      // UpperWenchConstants.kElevEncoderRotationsAtMaxHeight
      // && speed < 0) {
      // return;
      // }
      m_upperController.setOutput(speed);
      writeMetricsToSmartDashboard();
   }

   public void writeMetricsToSmartDashboard() {
      SmartDashboard.putNumber("UpperWench Relative Position", m_upperController.getCurrentRelativePosition());
      SmartDashboard.putNumber("UpperWench Absolute Position", m_upperController.getCurrentAbsolutePosition());
      SmartDashboard.putNumber("UpperWench Goal", m_upperController.getTargetPosition());
      SmartDashboard.putNumber("Evelvator Motor set output", m_upperController.getOutput());
   }

   public boolean isAtpositionA() {

      return true;
      // need to finish implementation, hook up hall effect sensors
   }

   public void zeroSensors() {
      m_upperController.zeroSensors();
   }

   public void setPositionZero() {
      m_upperController.setPositionZero();
   }

   public void setUpperWenchPosition() {
      Double targetPosition = SmartDashboard.getNumber("UpperWench Goal", 10);
      setUpperWenchPosition(targetPosition);
   }
}
