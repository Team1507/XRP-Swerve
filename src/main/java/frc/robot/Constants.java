// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

     public static final class IO {
        // Controllers
        public static final int JOYSTICK_PORT = 0;
    }

    public static final class Drive {
        public static final double TRANSLATION_SCALE    = 0.15;  // 0.15 // Input from X and Y controller to limit max speed from controller
        public static final double ROTATION_SCALE       = 0.25;  // 0.25 // Input from X of controller to limit max speed from controller

        // -----------------------------
        // Drivetrain speed constants
        // -----------------------------
        public static final double MAX_SPEED = 1.0; // XRP top speed (m/s) — tune as needed
        public static final double MAX_ANGULAR_RATE = RotationsPerSecond.of(0.75).in(RadiansPerSecond);
        public static final double DEADBAND = 0.1;
    }

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
        // Dimensions
        // ============================================================
        public static final double WHEELBASE = 0.5;   // meters (distance front-back)
        public static final double TRACKWIDTH = 0.5;  // meters (distance left-right)
        public static final double WHEEL_DIAMETER_METERS = 0.045; // 4.5 cm
        public static final double WHEEL_CIRCUMFERENCE_METERS = WHEEL_DIAMETER_METERS * Math.PI;
        public static final double ENCODER_TICKS_PER_REV = 585.0;
        public static final double METERS_PER_TICK = WHEEL_CIRCUMFERENCE_METERS / ENCODER_TICKS_PER_REV;

        public static final double MAX_SPEED_METERS_PER_SECOND = 5;

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
    
    public static final class MoveToPose {
        // --- PID gains for X, Y, and rotation ---
        public static final double X_KP     = 1.0;
        public static final double X_KI     = 0.0;
        public static final double X_KD     = 0.0;

        public static final double Y_KP     = 1.0;
        public static final double Y_KI     = 0.0;
        public static final double Y_KD     = 0.0;

        public static final double THETA_KP = 2.0;
        public static final double THETA_KI = 0.0;
        public static final double THETA_KD = 0.0;

        // --- Speed cap and deadband ---
        public static final double MAX_LINEAR_SPEED     = 0.75;  // 2 m/s cap for testing
        public static final double MAX_ANGULAR_SPEED    = 0.75;  // 3 rad/s cap for testing
        public static final double DEADBAND_ERROR       = 0.02; // meters, near target

        // --- Tolerances ---
        public static final double POSITION_TOLERANCE_METERS    = 0.05;
        public static final double ANGLE_TOLERANCE_RADIANS      = Math.toRadians(2.0);

        // --- Timeouts ---
        public static final double STALL_THRESHOLD  = 0.02; // meters
        public static final double STALL_TIMEOUT    = 1.0;  // seconds
        
        // --- Poses ---
        public static final Pose2d POSE_A = new Pose2d(3.0, 0.0, Rotation2d.fromDegrees(0));
        public static final Pose2d POSE_B = new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(0));
    }
}
