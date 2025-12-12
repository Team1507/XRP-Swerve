// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

// CTRE libraries
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

// WPI libraries
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

// Robot Subsystems
import frc.robot.subsystems.XRPBSubsystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;

// Robot Constants
import static frc.robot.Constants.Drive.*;
import static frc.robot.Constants.IO.*;

// Robot Extra
import frc.robot.util.Telemetry;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  // -----------------------------
  // Swerve Requests
  // -----------------------------
  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
          .withDeadband(MAX_SPEED * DEADBAND)
          .withRotationalDeadband(MAX_ANGULAR_RATE * DEADBAND)
          .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

  // -----------------------------
  // Subsystems
  // -----------------------------
  private final Telemetry logger = new Telemetry(MAX_SPEED);

  private final XRPBSubsystem xrpB = new XRPBSubsystem();
  private final CommandSwerveDrivetrain drivetrain = new CommandSwerveDrivetrain(xrpB);


  // -----------------------------
  // Controller
  // -----------------------------
  private final CommandXboxController joystick = new CommandXboxController(JOYSTICK_PORT);

  public RobotContainer() {
      configureBindings();
  }

  /**
   * Configures operator controls.
   */
  private void configureBindings() {

    drivetrain.setDefaultCommand(
        drivetrain.applyRequest(() ->
            drive
                .withVelocityX((-joystick.getLeftY() * TRANSLATION_SCALE) * MAX_SPEED)
                .withVelocityY((-joystick.getLeftX() * TRANSLATION_SCALE) * MAX_SPEED)
                .withRotationalRate((-joystick.getRightX() * ROTATION_SCALE) * MAX_ANGULAR_RATE)
        )
    );

    joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));

    // Run SysId routines when holding back/start and X/Y.
    // Note that each routine should be run exactly once in a single log.
    joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
    joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
    joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
    joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

    // reset the field-centric heading on left bumper press
    joystick.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

    drivetrain.registerTelemetry(logger::telemeterize);
  }

  /**
   * Returns the autonomous command.
   */
  public Command getAutonomousCommand() {
      return null;
  }
}
