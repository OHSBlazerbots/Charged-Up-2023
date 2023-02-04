// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.Constants.LightingConstants;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

// Reference docs for addressable LEDs
// https://docs.wpilib.org/en/stable/docs/software/hardware-apis/misc/addressable-leds.html
public class LightingSubsystem extends SubsystemBase {
    // hardware components
    private static final AddressableLED m_LedStrip1 = new AddressableLED(LightingConstants.kLedStrip1Port);
    private static final AddressableLEDBuffer m_LedStripBuffer1 = new AddressableLEDBuffer(
            LightingConstants.kLedStrip1Length);

    /** Creates a new LightingSubsystem. */
    public LightingSubsystem() {
        m_LedStrip1.setLength(m_LedStripBuffer1.getLength());

        m_LedStrip1.setData(m_LedStripBuffer1);
        m_LedStrip1.start();
    }

    public void setRed() {
        // set every item on buffer to red
        for (int i = 0; i < m_LedStripBuffer1.getLength(); i++) {
            m_LedStripBuffer1.setRGB(i, 255, 0, 0);
        }
        m_LedStrip1.setData(m_LedStripBuffer1);
    }

    public void setBlue() {
        for (int i = 0; i < m_LedStripBuffer1.getLength(); i++) {
            m_LedStripBuffer1.setRGB(i, 0, 0, 255);
        }
        m_LedStrip1.setData(m_LedStripBuffer1);
    }

    public void setGreen() {
        for (int i = 0; i < m_LedStripBuffer1.getLength(); i++) {
            m_LedStripBuffer1.setRGB(i, 0, 255, 255);
        }
        m_LedStrip1.setData(m_LedStripBuffer1);
    }
}
