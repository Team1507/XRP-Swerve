package frc.robot.adapters;

import frc.robot.interfaces.MotorInterface;
import frc.robot.subsystems.XRPBSubsystem;

/**
 * Motor adapter for controlling a motor on the secondary XRP board (XRP-B).
 * Commands are sent over UART to the remote controller.
 */
public class RemoteMotorAdapter implements MotorInterface {

    private final XRPBSubsystem xrpB;
    private final int motorId;

    /**
     * Creates a motor adapter for a remote motor.
     * @param xrpB reference to the XRP-B subsystem
     * @param motorId ID of the remote motor
     */
    public RemoteMotorAdapter(XRPBSubsystem xrpB, int motorId) {
        this.xrpB = xrpB;
        this.motorId = motorId;
    }

    /**
     * Sends a motor power command to the remote XRP-B controller.
     * @param power motor power (-1 to 1)
     */
    @Override
    public void set(double power) {
        xrpB.setRemoteMotor(motorId, power);
    }
}
