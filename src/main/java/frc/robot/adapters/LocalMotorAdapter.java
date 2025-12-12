package frc.robot.adapters;

import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import frc.robot.interfaces.MotorInterface;

/**
 * Motor adapter for controlling a motor connected directly to the XRP controller.
 */
public class LocalMotorAdapter implements MotorInterface {

    private final PWMSparkMax motor;

    /**
     * Creates a motor adapter for a local PWM motor port.
     * @param port PWM port number
     */
    public LocalMotorAdapter(int port) {
        this.motor = new PWMSparkMax(port);
    }

    /**
     * Sets the motor output power.
     * @param power motor power (-1 to 1)
     */
    @Override
    public void set(double power) {
        motor.set(power);
    }
}
