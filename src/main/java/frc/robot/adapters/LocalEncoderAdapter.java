package frc.robot.adapters;

import edu.wpi.first.wpilibj.Encoder;
import frc.robot.interfaces.EncoderInterface;

/**
 * Encoder adapter for reading from an encoder connected to the XRP controller.
 */
public class LocalEncoderAdapter implements EncoderInterface {

    private final Encoder encoder;

    /**
     * Creates an encoder adapter for a local encoder.
     * @param channelA encoder channel A
     * @param channelB encoder channel B
     */
    public LocalEncoderAdapter(int channelA, int channelB) {
        this.encoder = new Encoder(channelA, channelB);
    }

    /**
     * Returns the current encoder count.
     * @return encoder tick count
     */
    @Override
    public int getCounts() {
        return encoder.get();
    }
}
