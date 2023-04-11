package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.cscore.VideoSink;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class CameraSubsystem extends SubsystemBase {
    UsbCamera frontBumper;
    UsbCamera frontClaw;
    NetworkTableEntry cameraSelection;
    int currentCameraIndex = 0;
    VideoSink cameraServer;
    UsbCamera[] allCameras;

    public CameraSubsystem() {
        frontBumper = CameraServer.startAutomaticCapture(0); // 0 is placeholder
        frontClaw = CameraServer.startAutomaticCapture(1); // 1 is placeholder
        // cameraSelection =
        // NetworkTableInstance.getDefault().getTable("").getEntry("CameraSelection");
        cameraServer = CameraServer.getServer();
        allCameras = new UsbCamera[] { frontBumper, frontClaw };
    }

    public void nextCameraSelection() {
        System.out.println("part a:" + currentCameraIndex);
        currentCameraIndex += 1;
        System.out.println("part b:" + currentCameraIndex);
        currentCameraIndex = currentCameraIndex % allCameras.length;
        System.out.println("currently:" + currentCameraIndex);

        UsbCamera currentCamera = allCameras[currentCameraIndex];
        // cameraSelection.setString(currentCamera.getName());
        cameraServer.setSource(currentCamera);
    }
}
