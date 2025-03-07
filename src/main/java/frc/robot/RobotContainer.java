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
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;

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

  private double MaxSpeed = SwerveConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top
                                                                                 // speed
  private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max
                                                                                    // angular velocity

  /* Setting up bindings for necessary control of the swerve drive platform */
  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
      .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors

  // The robot's subsystems and commands are defined here...
  private final OI oi = new OI();
  private final SwerveSubsystem swerve = SwerveConstants.createDrivetrain();

  private final CameraSubsystem frontCamera = new CameraSubsystem(Constants.Vision.Front.name,
      Constants.Vision.Front.transform);
  private final CameraSubsystem rearCamera = new CameraSubsystem(Constants.Vision.Rear.name,
      Constants.Vision.Rear.transform);

  public final DataSubsystem dataSubsystem = new DataSubsystem();
  private final ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem();
  private final EndEffector endEffector = new EndEffector();
  private final LEDSubsystem ledSubsystem = new LEDSubsystem();

  private int reefStalk = 0;
  private int elevatorLevel = 0;

  private final Pose2d coralStationL = new Pose2d(0, 0, Rotation2d.fromDegrees(0));
  private final Pose2d coralStationR = new Pose2d(0, 0, Rotation2d.fromDegrees(0));

  private final Pose2d[] stalkPositions = { // A-L
      new Pose2d(3.156, 4.197, Rotation2d.fromDegrees(0)),
      new Pose2d(0, 0, Rotation2d.fromDegrees(0)),
      new Pose2d(0, 0, Rotation2d.fromDegrees(0)),
      new Pose2d(0, 0, Rotation2d.fromDegrees(0)),
      new Pose2d(0, 0, Rotation2d.fromDegrees(0)),
      new Pose2d(0, 0, Rotation2d.fromDegrees(0)),
      new Pose2d(0, 0, Rotation2d.fromDegrees(0)),
      new Pose2d(0, 0, Rotation2d.fromDegrees(0)),
      new Pose2d(0, 0, Rotation2d.fromDegrees(0)),
      new Pose2d(0, 0, Rotation2d.fromDegrees(0)),
      new Pose2d(0, 0, Rotation2d.fromDegrees(0)),
      new Pose2d(0, 0, Rotation2d.fromDegrees(0)),
  };

  private Pose2d targetPose = new Pose2d(1, 1, Rotation2d.fromDegrees(0));

  // Create the constraints to use while pathfinding
  private PathConstraints constraints = new PathConstraints(
      3.0, 4.0,
      Units.degreesToRadians(540), Units.degreesToRadians(720));

  // Since AutoBuilder is configured, we can use it to build pathfinding commands
  private Command pathfindingCommand = AutoBuilder.pathfindToPose(
      targetPose,
      constraints);

  private final boolean DEBUG_CONTROLS = false;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {

    ledSubsystem.breathe(Color.fromHSV(3, 255, 100), 8);
    // ledSubsystem.setPattern(ledSubsystem.scrollingRainbow);
    // ledSubsystem.setColor(Color.fromHSV(3, 255, 100));

    ledSubsystem.setDefaultCommand(ledSubsystem.run(() -> {
      ledSubsystem.setColor(endEffector.hasCoral() ? Color.fromHSV(0, 0, 100) : Color.fromHSV(3, 255, 100));
    }));

    NamedCommands.registerCommand("Raise", elevatorSubsystem.setLevel(4));
    NamedCommands.registerCommand("Score", Commands.sequence(
        endEffector.launchCoral(),
        elevatorSubsystem.setLevel(0)));
    NamedCommands.registerCommand("Intake", endEffector.intakeCoral());

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

    oi.gyroResetButton.onTrue(swerve.runOnce(swerve::tareEverything));

    frontCamera.setDefaultCommand(frontCamera.run(() -> {
      // Make sure to only set swerve pose if vision data is new
      if (frontCamera.hasNewData())
        swerve.addVisionMeasurement(frontCamera.getPose().toPose2d(), frontCamera.getPoseTime());
    }).ignoringDisable(true));

    rearCamera.setDefaultCommand(rearCamera.run(() -> {
      // Make sure to only set swerve pose if vision data is new
      if (rearCamera.hasNewData())
        swerve.addVisionMeasurement(rearCamera.getPose().toPose2d(), rearCamera.getPoseTime());
    }).ignoringDisable(true));

    dataSubsystem.setDefaultCommand(dataSubsystem.run(() -> {
      dataSubsystem.setRobotPose(swerve.getState().Pose);
    }).ignoringDisable(true));

    if (DEBUG_CONTROLS) {
      oi.elevatorUp.onTrue(elevatorSubsystem.setLevel(4));
      oi.elevatorDown.onTrue(elevatorSubsystem.setLevel(0));

      oi.intakeButton.onTrue(endEffector.intakeCoral());
      oi.scoreButton.onTrue(endEffector.launchCoral());
    } else {
      oi.elevatorUp.onTrue(elevatorSubsystem.setLevel(elevatorLevel));

      oi.intakeButton.onTrue(Commands.sequence(
          elevatorSubsystem.setLevel(0),
          new InstantCommand(() -> targetPose = coralStationL),
          // pathfindingCommand,
          endEffector.intakeCoral()));
      oi.scoreButton.onTrue(
          Commands.sequence(
              new InstantCommand(() -> elevatorSubsystem.setLevel(elevatorLevel)),
              new InstantCommand(() -> targetPose = stalkPositions[reefStalk]),
              // pathfindingCommand,
              endEffector.launchCoral()));
    }

    Commands.run(() -> {
      int stalk = oi.reefStalk.getAsInt();
      if (stalk > 0)
        reefStalk = stalk;
    });

    oi.nextReef.onTrue(Commands.runOnce(() -> reefStalk = (((reefStalk + 1) - 1) % 12) + 1));
    oi.nextReef.onTrue(Commands.runOnce(() -> reefStalk = (((reefStalk - 1) - 1) % 12) + 1));

    oi.elevatorNext.onTrue(Commands.runOnce(() -> elevatorLevel = Math.min(elevatorLevel + 1, 4)));
    oi.elevatorPrev.onTrue(Commands.runOnce(() -> elevatorLevel = Math.max(elevatorLevel - 1, 0)));
  }

  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return null;
  }
}
