package frc.robot.subsystems;

import frc.robot.Constants.ElevatorConstants;
import frc.robot.subsystems.motor_controllers.PositionPidMotorController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ElevatorSubsystem extends SubsystemBase {
   private static final PositionPidMotorController m_elevController = new PositionPidMotorController(
         ElevatorConstants.kElevGains,
         ElevatorConstants.kElevMotorPort);

   public ElevatorSubsystem() {
      m_elevController.zeroSensors();
   }

   public void setElevatorPosition(double targetPosition) {
      double motorPosition = targetPosition;
      m_elevController.goToPosition(motorPosition);
      writeMetricsToSmartDashboard();
   }

   public void setSafeWenchPosition() {
      setElevatorPosition(ElevatorConstants.elevatorSafeWenchPosition);
   }

   public void setElevatorSpeed(double speed) {
      // TODO: test this logic!!!
      // stops elavator when it reaches the relative max height to prevent breakage
      // if (m_elevController.getCurrentAbsolutePosition() >=
      // ElevatorConstants.kElevEncoderRotationsAtMaxHeight
      // && speed < 0) {
      // return;
      // }
      m_elevController.setOutput(speed);
      writeMetricsToSmartDashboard();
   }

   public void writeMetricsToSmartDashboard() {
      SmartDashboard.putNumber("Elevator Relative Position", m_elevController.getCurrentRelativePosition());
      SmartDashboard.putNumber("Elevator Absolute Position", m_elevController.getCurrentAbsolutePosition());
      SmartDashboard.putNumber("Elevator Goal", m_elevController.getTargetPosition());
      SmartDashboard.putNumber("Evelvator Motor set output", m_elevController.getOutput());
   }

   public boolean isAtpositionA() {

      return true;
      // need to finish implementation, hook up hall effect sensors
   }

   public void zeroSensors() {
      m_elevController.zeroSensors();
   }

   public void setPositionZero() {
      m_elevController.setPositionZero();
   }

   public void setElevatorPosition() {
      Double targetPosition = SmartDashboard.getNumber("Elevator Goal", 10);
      setElevatorPosition(targetPosition);
   }
}
