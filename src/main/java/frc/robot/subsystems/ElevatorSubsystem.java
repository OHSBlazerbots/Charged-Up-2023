package frc.robot.subsystems;

import frc.robot.Constants.ElevatorConstants;
import frc.robot.subsystems.motor_controllers.PositionPidMotorController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ElevatorSubsystem extends SubsystemBase {
   private static final PositionPidMotorController m_elevController = new PositionPidMotorController(
         ElevatorConstants.kElevGains,
         7);

   public ElevatorSubsystem() {
      m_elevController.zeroSensors();
   }

   public void setElevatorPosition(double targetPosition) {
      double motorPosition = targetPosition
            * ElevatorConstants.kElevEncoderRotationsAtMaxHeight;
      m_elevController.goToPosition(motorPosition);
      writeMetricsToSmartDashboard();
   }

   public void setElevatorSpeed(double speed) {
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
      setElevatorPosition(10);
   }
}
