package frc.robot.subsystems;

import java.io.File;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.Filesystem;
import swervelib.parser.SwerveParser;
import swervelib.SwerveDrive;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.SPI;



/*double maximumSpeed = Units.feetToMeters(4.5)
File swerveJsonDirectory = new File(Filesystem.getDeployDirectory(),"swerve");
SwerveDrive  swerveDrive = new SwerveParser(directory).createSwerveDrive(maximumSpeed);*/
public class SwerveSubsystem {
    AHRS gyro = new AHRS(SPI.Port.kMXP);
    public void printGyroStatus() {
        System.out.println("getPitch: " + gyro.getPitch());
        System.out.println("getYaw: " + gyro.getYaw());
        System.out.println("getRoll: " + gyro.getRoll());
        System.out.println("getAngle: " + gyro.getAngle());
    }
}
