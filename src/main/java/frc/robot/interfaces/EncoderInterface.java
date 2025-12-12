package frc.robot.interfaces;

/**
 * Represents a generic encoder device.
 * Implementations may read from local or remote encoders.
 */
public interface EncoderInterface {

    /**
     * Returns the current encoder count.
     * @return encoder tick count
     */
    int getCounts();
}
