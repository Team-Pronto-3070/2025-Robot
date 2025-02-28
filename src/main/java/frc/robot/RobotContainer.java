// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.OI;
// import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import static edu.wpi.first.units.Units.*;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final OI oi = new OI();
  public final SwerveSubsystem swerve = SwerveConstants.createDrivetrain();
  private final Autos autos = new Autos(swerve, null);

  private double MaxSpeed = SwerveConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top
                                                                                 // speed
  private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max
                                                                                    // angular velocity

  /* Setting up bindings for necessary control of the swerve drive platform */
  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
      .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // swerve.setDefaultCommand(swerve.run(() -> swerve.drive(
    // oi.processed_drive_x.getAsDouble(),
    // oi.processed_drive_y.getAsDouble(),
    // oi.processed_drive_rot.getAsDouble(),
    // true,
    // true)));
    swerve.setDefaultCommand(
        // Drivetrain will execute this command periodically
        swerve.applyRequest(() -> drive.withVelocityX(oi.processed_drive_x.getAsDouble() * -MaxSpeed) // Drive forward
                                                                                                      // with
            // negative Y (forward)
            .withVelocityY(oi.processed_drive_y.getAsDouble() * -MaxSpeed) // Drive left with negative X (left)
            .withRotationalRate(oi.processed_drive_rot.getAsDouble() * MaxAngularRate) // Drive counterclockwise with
                                                                                       // negative X (left)
        ));

    oi.gyroReset.onTrue(swerve.runOnce(() -> {
      swerve.resetPose(new Pose2d());
    }));
  }

  public Command getAutonomousCommand() {
    return autos.getAutonomousCommand();
  }
}
