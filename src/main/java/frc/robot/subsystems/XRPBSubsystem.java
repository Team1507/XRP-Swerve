// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
// WPI Lib
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
// Constants
import static frc.robot.Constants.XRPBConstants.*;

public class XRPBSubsystem extends SubsystemBase {

    private final SerialPort uart;

    private double m0 = 0;
    private double m1 = 0;
    private double m2 = 0;
    private double m3 = 0;

    private int enc0 = 0;
    private int enc1 = 0;
    private int enc2 = 0;
    private int enc3 = 0;

    public XRPBSubsystem() {
        uart = new SerialPort(BAUD_RATE, SerialPort.Port.kUSB);
        uart.setTimeout(TIMEOUT);
        uart.setWriteBufferMode(SerialPort.WriteBufferMode.kFlushOnAccess);
    }

    /** Send motor efforts to XRP B */
    public void sendMotorEfforts(double m0, double m1, double m2, double m3) {
        String packet = String.format(FORMAT, m0, m1, m2, m3);
        uart.writeString(packet);
    }

    /** Read encoder packet from XRP B */
    private void readEncoderPacket() {
        String line = uart.readString();

        if (line == null || line.isEmpty()) {
            return;
        }

        // Only process complete lines
        if (!line.contains("\n")) {
            return;
        }

        String[] parts = line.trim().split(",");
        if (parts.length != 4) {
            return;
        }

        try {
            enc0 = Integer.parseInt(parts[0]);
            enc1 = Integer.parseInt(parts[1]);
            enc2 = Integer.parseInt(parts[2]);
            enc3 = Integer.parseInt(parts[3]);
        } catch (Exception e) {
            // Ignore malformed packets
        }
    }

    @Override
    public void periodic() {
        readEncoderPacket();
    }

    public int getRemoteEncoder(int index) {
        return switch (index) {
            case 0 -> enc0;
            case 1 -> enc1;
            case 2 -> enc2;
            case 3 -> enc3;
            default -> 0;
        };
    }
    
    public void setRemoteMotor(int index, double effort) {
        switch (index) {
            case 0 -> m0 = effort;
            case 1 -> m1 = effort;
            case 2 -> m2 = effort;
            case 3 -> m3 = effort;
        }
    
        sendMotorEfforts(m0, m1, m2, m3);
    }    

    // Getters for swerve math
    public int getEnc0() { return enc0; }
    public int getEnc1() { return enc1; }
    public int getEnc2() { return enc2; }
    public int getEnc3() { return enc3; }
}
