// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static final class XRPBConstants{
        public static final int BAUD_RATE = 115200;
        public static final double TIMEOUT = 0.005;
        public static final String FORMAT = "%.3f,%.3f,%.3f,%.3f\n";
    }

    public static final class SwerveConstants {

        // ============================================================
        // LOCAL (XRP A) MOTOR PORTS
        // ============================================================
        public static final int FRONT_RIGHT_UPPER_MOTOR_PORT = 0; // MotorL
        public static final int FRONT_RIGHT_LOWER_MOTOR_PORT = 1; // Motor3
    
        public static final int FRONT_LEFT_UPPER_MOTOR_PORT  = 2; // MotorR
        public static final int FRONT_LEFT_LOWER_MOTOR_PORT  = 3; // Motor4
    
        // ============================================================
        // LOCAL (XRP A) ENCODER DIO CHANNELS
        // ============================================================
        public static final int FRONT_RIGHT_UPPER_ENCODER_A = 0;
        public static final int FRONT_RIGHT_UPPER_ENCODER_B = 1;
        public static final int FRONT_RIGHT_LOWER_ENCODER_A = 2;
        public static final int FRONT_RIGHT_LOWER_ENCODER_B = 3;
    
        public static final int FRONT_LEFT_UPPER_ENCODER_A  = 4;
        public static final int FRONT_LEFT_UPPER_ENCODER_B  = 5;
        public static final int FRONT_LEFT_LOWER_ENCODER_A  = 6;
        public static final int FRONT_LEFT_LOWER_ENCODER_B  = 7;
    
        // ============================================================
        // REMOTE (XRP B) MOTOR INDICES
        // ============================================================
        public static final int BACK_RIGHT_UPPER_REMOTE_MOTOR = 0; // MotorR
        public static final int BACK_RIGHT_LOWER_REMOTE_MOTOR = 1; // Motor4
    
        public static final int BACK_LEFT_UPPER_REMOTE_MOTOR  = 2; // MotorL
        public static final int BACK_LEFT_LOWER_REMOTE_MOTOR  = 3; // Motor3
    
        // ============================================================
        // REMOTE (XRP B) ENCODER INDICES
        // ============================================================
        public static final int BACK_RIGHT_UPPER_REMOTE_ENCODER = 0;
        public static final int BACK_RIGHT_LOWER_REMOTE_ENCODER = 1;
    
        public static final int BACK_LEFT_UPPER_REMOTE_ENCODER  = 2;
        public static final int BACK_LEFT_LOWER_REMOTE_ENCODER  = 3;

        // ============================================================
        // Dimensions of Base
        // ============================================================
        public static final double WHEELBASE = 0.5;   // meters (distance front-back)
        public static final double TRACKWIDTH = 0.5;  // meters (distance left-right)

        // ============================================================
        // SERVE MODULE CONSTANTS
        // ============================================================
        public static final double TICKS_PER_MODULE_REV =
                12 * (48.0 / 1.0) * (74.0 / 36.0) * 2.0;
    
        public static final double DEGREES_PER_TICK = 360.0 / TICKS_PER_MODULE_REV;
    
        public static final double ALLOWED_MODULE_ORIENTATION_ERROR = 10.0;  // degrees
        public static final double ANGLE_OF_MAX_MODULE_ROTATION_POWER = 30.0; // degrees
        public static final double ROT_ADVANTAGE = 1.0;
        public static final double MAX_MOTOR_POWER = 1.0;
    }
    
}
