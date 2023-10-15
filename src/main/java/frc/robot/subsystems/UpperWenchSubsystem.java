package frc.robot.subsystems;

import frc.robot.Constants.UpperWenchConstants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;


public class UpperWenchSubsystem extends SubsystemBase {
//    private static final PositionPidMotorController m_upperController = new PositionPidMotorController(
//          UpperWenchConstants.kElevGains,
//          UpperWenchConstants.kElevMotorPort);
private CANSparkMax m_leadMotor = new CANSparkMax(UpperWenchConstants.kUpperMotorPort, MotorType.kBrushless);
 

   public UpperWenchSubsystem() {
      // m_upperController.zeroSensors();
      m_leadMotor.restoreFactoryDefaults();
   }

   // public void setUpperWenchPosition(double targetPosition) {
   //    double motorPosition = targetPosition;
   //    m_upperController.goToPosition(motorPosition);
   //    writeMetricsToSmartDashboard();
   // }

   // public void setSafeWenchPosition() {
   //    setUpperWenchPosition(UpperWenchConstants.upperatorSafeWenchPosition);
   // }

   public void setUpperWenchSpeed(double speed) {
      // TODO: test this logic!!!
      // stops elavator when it reaches the relative max height to prevent breakage
      // if (m_upperController.getCurrentAbsolutePosition() >=
      // UpperWenchConstants.kElevEncoderRotationsAtMaxHeight
      // && speed < 0) {
      // return;
      // }
      m_leadMotor.set(speed);
      writeMetricsToSmartDashboard();
   }

   public void writeMetricsToSmartDashboard() {
      // SmartDashboard.putNumber("UpperWench Relative Position", m_upperController.getCurrentRelativePosition());
      // SmartDashboard.putNumber("UpperWench Absolute Position", m_upperController.getCurrentAbsolutePosition());
      // SmartDashboard.putNumber("UpperWench Goal", m_upperController.getTargetPosition());
      SmartDashboard.putNumber("UpperWench Motor set output", m_leadMotor.get());
   }

//    public boolean isAtpositionA() {

//       return true;
//       // need to finish implementation, hook up hall effect sensors
//    }

//    public void zeroSensors() {
//       m_upperControfller.zeroSensors();
//    }

//    public void setPositionZero() {
//       m_upperController.setPositionZero();
//    }

//    public void setUpperWenchPosition() {
//       Double targetPosition = SmartDashboard.getNumber("UpperWench Goal", 10);
//       setUpperWenchPosition(targetPosition);
//    }
}
