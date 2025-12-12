package frc.robot.adapters;

import frc.robot.interfaces.EncoderInterface;
import frc.robot.subsystems.XRPBSubsystem;

/**
 * Encoder adapter for reading encoder values from the XRP-B controller.
 * Values are received over UART.
 */
public class RemoteEncoderAdapter implements EncoderInterface {

    private final XRPBSubsystem xrpB;
    private final int encoderId;

    /**
     * Creates an encoder adapter for a remote encoder.
     * @param xrpB reference to the XRP-B subsystem
     * @param encoderId ID of the remote encoder
     */
    public RemoteEncoderAdapter(XRPBSubsystem xrpB, int encoderId) {
        this.xrpB = xrpB;
        this.encoderId = encoderId;
    }

    /**
     * Returns the current encoder count from the remote controller.
     * @return encoder tick count
     */
    @Override
    public int getCounts() {
        return xrpB.getRemoteEncoder(encoderId);
    }
}
