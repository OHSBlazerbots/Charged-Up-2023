package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import frc.robot.Constants.WenchConstants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class WenchSubsystem extends SubsystemBase {
    private static final WPI_TalonSRX m_wenchMotor = new WPI_TalonSRX(WenchConstants.kWenchPrimaryMotorPort);

    public WenchSubsystem() {
        m_wenchMotor.configFactoryDefault();

        m_wenchMotor.setNeutralMode(NeutralMode.Brake); // ".Brake" is placehoder, not sure if we need brake or coast.
    }

    public void setArmSpeed(double speed) {
        m_wenchMotor.set(speed);
    }
}