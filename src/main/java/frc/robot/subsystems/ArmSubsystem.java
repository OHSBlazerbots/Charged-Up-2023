package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import frc.robot.Constants.ArmConstants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmSubsystem extends SubsystemBase {
    private static final WPI_TalonSRX m_armMotor = new WPI_TalonSRX(ArmConstants.kArmMotorPort);

    public ArmSubsystem() {
        m_armMotor.configFactoryDefault();

        m_armMotor.setNeutralMode(NeutralMode.Brake); // ".Brake" is placehoder, not sure if we need brake or coast.
    }

    public void setArmSpeed(double speed) {
        m_armMotor.set(speed);
    }
}