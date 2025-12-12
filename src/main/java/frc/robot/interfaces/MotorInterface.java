package frc.robot.interfaces;

/**
 * Represents a generic motor output device.
 * Implementations may control local or remote motors.
 */
public interface MotorInterface {

    /**
     * Sets the motor output power.
     * @param power motor power (-1 to 1)
     */
    void set(double power);
}
