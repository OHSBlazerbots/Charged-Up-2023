package frc.robot.subsystems;

// import frc.robot.Constants.PneumaticsConstants;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.*;

import edu.wpi.first.wpilibj.Compressor;

public class PneumaticsSubsystem extends SubsystemBase {

   DoubleSolenoid p_solenoidPCM = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 1, 0);

   Compressor pcmCompressor = new Compressor(0, PneumaticsModuleType.CTREPCM);

   public PneumaticsSubsystem() {
      p_solenoidPCM.set(kOff);
   }

   public void toggleCompressor() {
      if (pcmCompressor.isEnabled()) {
         pcmCompressor.disable();
      } else {
         pcmCompressor.enableDigital();
      }
   }

   public void forward() {
      p_solenoidPCM.set(kForward);
   }

   public void backward() {
      p_solenoidPCM.set(kReverse);
   }

   public void off() {
      p_solenoidPCM.set(kOff);
   }
}