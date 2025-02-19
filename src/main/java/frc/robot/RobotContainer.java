// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.OI;
import frc.robot.subsystems.CameraSubsystem;
import frc.robot.subsystems.DataSubsystem;
// import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import static edu.wpi.first.units.Units.*;

import java.sql.Time;

import javax.xml.crypto.Data;

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
  private final SwerveSubsystem swerve = SwerveConstants.createDrivetrain();
  private final CameraSubsystem cameraSubsystem = new CameraSubsystem();
  private final DataSubsystem dataSubsystem = new DataSubsystem();

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

    Sendable sendable = new Sendable() {
      @Override
      public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("SwerveDrive");

        var frontLeftModule = swerve.getModules()[0];
        var frontRightModule = swerve.getModules()[1];
        var backLeftModule = swerve.getModules()[2];
        var backRightModule = swerve.getModules()[3];

        builder.addDoubleProperty("Front Left Angle",
            () -> frontLeftModule.getEncoder().getAbsolutePosition().getValueAsDouble(), null);
        builder.addDoubleProperty("Front Left Velocity",
            () -> frontLeftModule.getDriveMotor().getVelocity().getValueAsDouble(), null);

        builder.addDoubleProperty("Front Right Angle",
            () -> frontRightModule.getEncoder().getAbsolutePosition().getValueAsDouble(), null);
        builder.addDoubleProperty("Front Right Velocity",
            () -> frontRightModule.getDriveMotor().getVelocity().getValueAsDouble(), null);

        builder.addDoubleProperty("Back Left Angle",
            () -> backLeftModule.getEncoder().getAbsolutePosition().getValueAsDouble(), null);
        builder.addDoubleProperty("Back Left Velocity",
            () -> backLeftModule.getDriveMotor().getVelocity().getValueAsDouble(), null);

        builder.addDoubleProperty("Back Right Angle",
            () -> backRightModule.getEncoder().getAbsolutePosition().getValueAsDouble(), null);
        builder.addDoubleProperty("Back Right Velocity",
            () -> backRightModule.getDriveMotor().getVelocity().getValueAsDouble(), null);

        builder.addDoubleProperty("Robot Angle", () -> swerve.getRotation3d().toRotation2d().getRadians(), null);
      }
    };

    SmartDashboard.putData("Swerve Drive", sendable);

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
      // swerve.resetPose(new Pose2d());
      swerve.tareEverything();
    }));

    // swerve.resetPose(cameraSubsystem.getPose().toPose2d());

    cameraSubsystem.setCallback(cameraSubsystem.run(() -> {
      // swerve.addVisionMeasurement(cameraSubsystem.getPose().toPose2d(),
      // cameraSubsystem.getPoseTime() / 1000);
      swerve.resetPose(cameraSubsystem.getPose().toPose2d());
    }));

    // cameraSubsystem.setDefaultCommand(cameraSubsystem.run(() -> {
      // System.out.println(cameraSubsystem.getPose().toPose2d());
      // System.out.println(cameraSubsystem.getPoseTime() / 1000);
      // cameraSubsystem.getPoseTime() / 1000);
      // swerve.addVisionMeasurement(cameraSubsystem.getPose().toPose2d(),
      // swerve.resetPose(cameraSubsystem.getPose().toPose2d());
    // }));

    dataSubsystem.setDefaultCommand(dataSubsystem.run(() -> {
      dataSubsystem.update(swerve.getState().Pose);
      // dataSubsystem.update(cameraSubsystem.getPose().toPose2d());
    }));
  }

  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return null;
  }
}
