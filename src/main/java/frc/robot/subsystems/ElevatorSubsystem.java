package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import frc.robot.Constants.ElevatorConstants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ElevatorSubsystem extends SubsystemBase {
   private static final WPI_TalonSRX m_elevMotor = new WPI_TalonSRX(ElevatorConstants.kElevMotorPort);

   public ElevatorSubsystem() {
      m_elevMotor.configFactoryDefault();

      m_elevMotor.setNeutralMode(NeutralMode.Brake);
   }

   public void setElevatorSpeed(double speed) {
      m_elevMotor.set(speed);

   }

   public boolean isAtpositionA() {

      return true;
      // need to finish implementation, hook up hall effect sensors
   }

}