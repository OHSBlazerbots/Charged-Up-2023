package frc.robot.subsystems;

import frc.robot.Constants.ElevatorConstants;
import frc.robot.subsystems.motor_controllers.PositionPidDualMotorController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ElevatorSubsystem extends SubsystemBase {
   private static final PositionPidDualMotorController m_elevController = new PositionPidDualMotorController(
         ElevatorConstants.kElevGains,
         ElevatorConstants.kElevMotorPrimaryPort,
         ElevatorConstants.kElevMotorSecondaryPort);

   public ElevatorSubsystem() {
      m_elevController.zeroSensors();
   }

   public void setElevatorPosition(double targetPosition) throws Exception {
      // Validation that input position is valid
      if (targetPosition < 0 || targetPosition > 1) {
         throw new Exception("Position must be in range [0, 1]");
      }

      double motorPosition = targetPosition * ElevatorConstants.kElevEncoderRotationsAtMaxHeight;
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

}