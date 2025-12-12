// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import edu.wpi.first.wpilibj.I2C;
// WPI Lib
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
// Constants
import static frc.robot.Constants.XRPBConstants.*;

public class XRPBSubsystem extends SubsystemBase {

    private double m0 = 0;
    private double m1 = 0;
    private double m2 = 0;
    private double m3 = 0;

    private int enc0 = 0;
    private int enc1 = 0;
    private int enc2 = 0;
    private int enc3 = 0;

    private static final int I2C_ADDRESS = 0x20;

    private final I2C i2c;
    private int heartbeat = 0;
    private double lastHeartbeatTime = 0.0;

    public XRPBSubsystem() {
        i2c = new I2C(I2C.Port.kOnboard, I2C_ADDRESS);
        System.out.println("XRPBSubsystem: I2C device initialized at 0x20");
    }

    private void readHeartbeat() {
        byte[] buffer = new byte[4];
    
        // Combined write-then-read: set register pointer to 0x20, then read 4 bytes
        boolean fail = i2c.transaction(
            new byte[] { (byte) 0x20 }, // write: register address
            1,                          // write length
            buffer,                     // read buffer
            4                           // read length
        );
    
        if (fail) {
            System.out.println("I2C transaction failed");
            return;
        }
    
        // Log the raw bytes
        System.out.printf("HB bytes: %02X %02X %02X %02X%n",
            buffer[0], buffer[1], buffer[2], buffer[3]);
    
        // For this test, just treat heartbeat as the first byte
        int hb = buffer[0] & 0xFF;
        heartbeat = hb;
        lastHeartbeatTime = Timer.getFPGATimestamp();
    }    

    public int getHeartbeat() {
        return heartbeat;
    }

    public boolean isAlive() {
        return (Timer.getFPGATimestamp() - lastHeartbeatTime) < 0.5;
    }


    /** Send motor efforts to XRP B */
    public void sendMotorEfforts(double m0, double m1, double m2, double m3) {
        String packet = String.format(FORMAT, m0, m1, m2, m3);
        //uart.writeString(packet);
    }

    /** Read encoder packet from XRP B */
    private void readEncoderPacket() {
        String line = "null"; //uart.readString();

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

            heartbeat = Integer.parseInt(parts[4]);
            lastHeartbeatTime = Timer.getFPGATimestamp();
        } catch (Exception e) {
            // Ignore malformed packets
        }
    }

    @Override
    public void periodic() {
        readHeartbeat();

        double age = Timer.getFPGATimestamp() - lastHeartbeatTime;

        SmartDashboard.putNumber("XRPB Heartbeat", heartbeat);
        SmartDashboard.putNumber("XRPB Packet Age", age);
        SmartDashboard.putBoolean("XRPB Link OK", age < 0.25);
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
