// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.subsystems.CameraSubsystem;
import frc.robot.subsystems.DataSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.EndEffector;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
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
  private final SwerveSubsystem swerve = SwerveConstants.createDrivetrain();

  private final CameraSubsystem frontCamera = new CameraSubsystem(Constants.Vision.Front.name,
      Constants.Vision.Front.transform);
  private final CameraSubsystem rearCamera = new CameraSubsystem(Constants.Vision.Rear.name,
      Constants.Vision.Rear.transform);

  private final DataSubsystem dataSubsystem = new DataSubsystem();
  private final ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem();
  private final EndEffector endEffector = new EndEffector();
  private final LEDSubsystem ledSubsystem = new LEDSubsystem();

  private double MaxSpeed = SwerveConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top
                                                                                 // speed
  private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max
                                                                                    // angular velocity

  /* Setting up bindings for necessary control of the swerve drive platform */
  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
      .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors

  private double lastVisionTime = 0;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {

    ledSubsystem.breathe(Color.fromHSV(3, 255, 100), 8);
    // ledSubsystem.setPattern(ledSubsystem.scrollingRainbow);

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

    oi.interruptButton
        .onTrue(elevatorSubsystem.runOnce(elevatorSubsystem::stop))
        .onTrue(endEffector.runOnce(endEffector::stop));

    frontCamera.setDefaultCommand(frontCamera.run(() -> {
      // Make sure to only set swerve pose if vision data is new
      if (frontCamera.hasNewData())
        swerve.addVisionMeasurement(frontCamera.getPose().toPose2d(), frontCamera.getPoseTime());
    }));

    rearCamera.setDefaultCommand(rearCamera.run(() -> {
      // Make sure to only set swerve pose if vision data is new
      if (rearCamera.hasNewData())
        swerve.addVisionMeasurement(rearCamera.getPose().toPose2d(), rearCamera.getPoseTime());
    }));

    dataSubsystem.setDefaultCommand(dataSubsystem.run(() -> {
      dataSubsystem.update(swerve.getState().Pose);
      // dataSubsystem.update(cameraSubsystem.getPose().toPose2d());
    }));

    oi.elevatorUp.onTrue(elevatorSubsystem.runOnce(() -> {
      elevatorSubsystem.moveUp();
    }));

    oi.elevatorDown.onTrue(elevatorSubsystem.runOnce(() -> {
      elevatorSubsystem.moveDown();
    }));
  }

  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return null;
  }
}
