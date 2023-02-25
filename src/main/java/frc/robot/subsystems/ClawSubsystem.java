package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import frc.robot.Constants.ClawConstants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClawSubsystem extends SubsystemBase {
    private static final WPI_TalonSRX m_clawMotor = new WPI_TalonSRX(ClawConstants.kClawMotorPort);

    public ClawSubsystem() {
        m_clawMotor.configFactoryDefault();

        m_clawMotor.setNeutralMode(NeutralMode.Brake);
    }

    public void setClawSpeed(double speed) {
        m_clawMotor.set(speed);
    }
}