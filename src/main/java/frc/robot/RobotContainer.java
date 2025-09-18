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
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
// import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

import static edu.wpi.first.units.Units.*;

import java.util.ArrayList;
import java.util.List;

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
  public final EndEffector endEffector = new EndEffector();
  private final LEDSubsystem ledSubsystem = new LEDSubsystem();

  // private int reefStalk = 0;
  // private int elevatorLevel = 0;

  // private final Pose2d coralStationL = new Pose2d(0, 0,
  // Rotation2d.fromDegrees(0));
  // private final Pose2d coralStationR = new Pose2d(0, 0,
  // Rotation2d.fromDegrees(0));

  private final Pose2d[] stalkPositions = { // A-L
      // new Pose2d(3.63, 4.04, Rotation2d.fromDegrees(0)),
      // new Pose2d(3.47, 3.73, Rotation2d.fromDegrees(0)),
      new Pose2d(3.50, 4.05, Rotation2d.fromDegrees(0)),
      new Pose2d(3.50, 3.80, Rotation2d.fromDegrees(0)),
      new Pose2d(3.96, 3.14, Rotation2d.fromDegrees(60)),
      new Pose2d(4.36, 3.05, Rotation2d.fromDegrees(60)),
      new Pose2d(5.54, 3.46, Rotation2d.fromDegrees(120)),
      new Pose2d(4.90, 3.56, Rotation2d.fromDegrees(120)),
      new Pose2d(5.51, 3.93, Rotation2d.fromDegrees(180)),
      new Pose2d(5.29, 4.34, Rotation2d.fromDegrees(180)),
      new Pose2d(5.01, 4.83, Rotation2d.fromDegrees(-120)),
      new Pose2d(4.75, 4.68, Rotation2d.fromDegrees(-120)),
      new Pose2d(3.96, 4.93, Rotation2d.fromDegrees(-60)),
      new Pose2d(3.70, 4.73, Rotation2d.fromDegrees(-60)),
  };

  private final int[] blueTags = { 18, 17, 22, 21, 20, 19 };

  private final int[] redTags = { 7, 8, 9, 10, 11, 6 };

  // private Pose2d targetPose = new Pose2d(1.5, 1.5, Rotation2d.fromDegrees(45));

  // private Translation2d blueReef = new Translation2d(4.1,
  // Constants.FIELD_HEIGHT / 2);
  // private Translation2d redReef = new Translation2d(Constants.FIELD_WIDTH -
  // 4.1, Constants.FIELD_HEIGHT / 2);

  // Create the constraints to use while pathfinding
  private PathConstraints constraints = new PathConstraints(
      3.0, 4.0,
      Units.degreesToRadians(540), Units.degreesToRadians(720));

  // PathConstraints constraints = PathConstraints.unlimitedConstraints(12.0);
  // You can also use unlimited constraints, only limited by motor torque and
  // nominal battery voltage

  // Since AutoBuilder is configured, we can use it to build pathfinding commands
  private Command pathfindingCommand;

  private boolean useVision = true;

  // slew rate limiters are in units/second e.g 0.5 takes 2 seconds to get to 100%
  // private SlewRateLimiter xLimiter = new
  // SlewRateLimiter(Constants.Elevator.acceleration[0]);
  // private SlewRateLimiter yLimiter = new
  // SlewRateLimiter(Constants.Elevator.acceleration[0]);
  // private SlewRateLimiter rLimiter = new
  // SlewRateLimiter(Constants.Elevator.acceleration[0]);

  private PathPlannerPath path;

  private boolean hasCoral = false;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    try {
      var positions = PathPlannerPath.fromPathFile("Reef");

      for (int i = 0; i < stalkPositions.length; i++) {
        stalkPositions[i] = new Pose2d(positions.getWaypoints().get(i).anchor().getX(),
            positions.getWaypoints().get(i).anchor().getY(), stalkPositions[i].getRotation());
        System.out.println("added position: (X: " + stalkPositions[i].getX() + ", Y: " + stalkPositions[i].getY()
            + ", R: " + stalkPositions[i].getRotation().getDegrees() + ")");
      }
    } catch (Exception e) {
      System.out.println("Failed to load reef positions");
    }

    ledSubsystem.breathe(Color.fromHSV(3, 255, 100), 8);
    // ledSubsystem.setPattern(ledSubsystem.scrollingRainbow);
    // ledSubsystem.setColor(Color.fromHSV(3, 255, 100));

    ledSubsystem.setDefaultCommand(ledSubsystem.run(() -> {
      Color color = Color.fromHSV(6 / 2, 255, 100);

      if (endEffector.hasCoral()) {
        color = Color.fromHSV(0 / 2, 0, 100);
      }

      if (oi.autoAlign.getAsBoolean()) {
        color = Color.fromHSV(100 / 2, 255, 100);
      }

      ledSubsystem.setColor(color);
    }));

    NamedCommands.registerCommand("L1", elevatorSubsystem.runOnce(() -> elevatorSubsystem.setLevel(1)));
    NamedCommands.registerCommand("L2", elevatorSubsystem.runOnce(() -> elevatorSubsystem.setLevel(2)));
    NamedCommands.registerCommand("L3", elevatorSubsystem.runOnce(() -> elevatorSubsystem.setLevel(3)));
    NamedCommands.registerCommand("L4", elevatorSubsystem.runOnce(() -> elevatorSubsystem.setLevel(4)));

    NamedCommands.registerCommand("Score", Commands.sequence(
        endEffector.launchCoral(),
        elevatorSubsystem.runOnce(() -> elevatorSubsystem.setLevel(0))));

    NamedCommands.registerCommand("Intake", endEffector.intakeCoral());

    swerve.setDefaultCommand(
        // Drivetrain will execute this command periodically
        swerve.applyRequest(() -> drive
            // .withVelocityX(xLimiter.calculate(oi.processed_drive_x.getAsDouble()) *
            // -MaxSpeed
            .withVelocityX(oi.processed_drive_x.getAsDouble() * -MaxSpeed
                * Constants.Elevator.maxSpeeds[elevatorSubsystem.getLevel()]) // Drive forward
            // with
            // negative Y (forward)
            // .withVelocityY(yLimiter.calculate(oi.processed_drive_y.getAsDouble()) *
            // -MaxSpeed
            .withVelocityY(oi.processed_drive_y.getAsDouble() * -MaxSpeed
                * Constants.Elevator.maxSpeeds[elevatorSubsystem.getLevel()]) // Drive left with negative X (left)
            // .withRotationalRate(rLimiter.calculate(oi.processed_drive_rot.getAsDouble())
            // * MaxAngularRate) // Drive
            .withRotationalRate(oi.processed_drive_rot.getAsDouble() * MaxAngularRate) // Drive
        // counterclockwise
        // with
        // negative X (left)
        ));

    endEffector.setDefaultCommand(endEffector.runOnce(() -> {
      if (endEffector.hasCoral() && !hasCoral) {
        endEffector.runOnce(() -> {
          oi.driveRumble(RumbleType.kBothRumble, 1);
        }).andThen(Commands.waitSeconds(1)).andThen(endEffector.runOnce(() -> {
          oi.driveRumble(RumbleType.kBothRumble, 0);
        }));
      }
    }));

    oi.interruptButton
        .onTrue(elevatorSubsystem.runOnce(elevatorSubsystem::stop))
        .onTrue(endEffector.runOnce(endEffector::stop));

    oi.gyroResetButton.onTrue(swerve.runOnce(swerve::tareEverything));

    frontCamera.setDefaultCommand(frontCamera.run(() -> {
      // Make sure to only set swerve pose if vision data is new
      if (useVision && frontCamera.hasNewData())
        swerve.addVisionMeasurement(frontCamera.getPose().toPose2d(), frontCamera.getPoseTime());
    }).ignoringDisable(true));

    rearCamera.setDefaultCommand(rearCamera.run(() -> {
      // Make sure to only set swerve pose if vision data is new
      // if (useVision && rearCamera.hasNewData())
      // swerve.addVisionMeasurement(rearCamera.getPose().toPose2d(),
      // rearCamera.getPoseTime());
    }).ignoringDisable(true));

    dataSubsystem.setDefaultCommand(dataSubsystem.run(() -> {
      dataSubsystem.setRobotPose(swerve.getState().Pose);
    }).ignoringDisable(true));

    oi.elevatorUp.onTrue(elevatorSubsystem.runOnce(() -> {
      elevatorSubsystem.moveUp();
      // setAcceleration(Constants.Elevator.acceleration[elevatorSubsystem.getLevel()]);
    }));
    oi.elevatorDown.onTrue(elevatorSubsystem.runOnce(() -> {
      elevatorSubsystem.moveDown();
      // setAcceleration(Constants.Elevator.acceleration[elevatorSubsystem.getLevel()]);
    }));

    oi.intakeButton
        .onTrue(endEffector.intakeCoral().alongWith(elevatorSubsystem.run(() -> {
          elevatorSubsystem.setLevel(0);
          // setAcceleration(Constants.Elevator.acceleration[elevatorSubsystem.getLevel()]);
        })));
    oi.scoreButton
        .onTrue(endEffector.launchCoral().andThen(elevatorSubsystem.run(() -> {
          elevatorSubsystem.setLevel(0);
          // setAcceleration(Constants.Elevator.acceleration[elevatorSubsystem.getLevel()]);
        })));

    oi.manualElevatorUp.onTrue(elevatorSubsystem.runOnce(() -> {
      elevatorSubsystem.moveUp();
      // setAcceleration(Constants.Elevator.acceleration[elevatorSubsystem.getLevel()]);
    }));
    oi.manualElevatorDown.onTrue(elevatorSubsystem.runOnce(() -> {
      elevatorSubsystem.moveDown();
      // setAcceleration(Constants.Elevator.acceleration[elevatorSubsystem.getLevel()]);
    }));
    oi.manualIntake.onTrue(endEffector.runOnce(() -> {
      endEffector.setCoral(1);
    }));
    oi.manualOuttake.onTrue(endEffector.runOnce(() -> {
      endEffector.setCoral(-1);
    }));

    oi.coralDelayUp.onTrue(endEffector.runOnce(() -> endEffector.increaseCoralDelay()));
    oi.coralDelayDown.onTrue(endEffector.runOnce(() -> endEffector.decreaseCoralDelay()));

    oi.visionToggle.onTrue(Commands.runOnce(() -> useVision = !useVision));

    // oi.povUp.onTrue(endEffector.runOnce(() -> endEffector.setArm(0.1)));
    // oi.povUp.onFalse(endEffector.runOnce(() -> endEffector.setArm(0.0)));
    // oi.povDown.onTrue(endEffector.runOnce(() -> endEffector.setArm(-0.1)));
    // oi.povDown.onFalse(endEffector.runOnce(() -> endEffector.setArm(0.0)));

    oi.povUp.onTrue(endEffector.runOnce(() -> endEffector.armUp()));
    oi.povDown.onTrue(endEffector.runOnce(() -> endEffector.armDown()));

    // oi.povLeft.onTrue(endEffector.runOnce(() -> endEffector.setAlgae(0.1)));
    // oi.povLeft.onFalse(endEffector.runOnce(() -> endEffector.setAlgae(0.0)));
    // oi.povRight.onTrue(endEffector.runOnce(() -> endEffector.setAlgae(-0.1)));
    // oi.povRight.onFalse(endEffector.runOnce(() -> endEffector.setAlgae(0.0)));

    oi.autoAlign.onTrue(Commands.runOnce(() -> {
      pathfindingCommand.until(oi.autoAlign.negate()).schedule();
    }));

    // Commands.run(() -> {}).schedule();
  }

  public void periodic() {
    SmartDashboard.putNumber("Pose X", swerve.getState().Pose.getX());
    SmartDashboard.putNumber("Pose Y", swerve.getState().Pose.getY());
    SmartDashboard.putNumber("Pose R", swerve.getState().Pose.getRotation().getDegrees());

    useVision = SmartDashboard.getBoolean("Use Vision", true);
    SmartDashboard.putBoolean("Use Vision", useVision);

    // System.out.println("Posey");

    if (oi.autoAlign.getAsBoolean()) {
      return;
    }

    // int[] tagNums = blueTags;

    boolean redAlliance = false;
    if (DriverStation.getAlliance().isPresent()) {
      redAlliance = (DriverStation.getAlliance().get() == Alliance.Red);
      // tagNums = redTags;
    }

    // List<AprilTag> tags =
    // AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded).getTags();

    // find the closest stalk
    Pose2d closest = null;
    double closestDistance = Double.MAX_VALUE;

    Pose2d robot = swerve.getState().Pose;

    // for (int i = 0; i < tagNums.length; i++) {
    // Pose2d tag = tags.get(tagNums[i]).pose.toPose2d();

    // double dist = 0.3;
    // Pose2d left = tag.transformBy(new
    // Transform2d(Math.cos(tag.getRotation().getRadians()) * dist,
    // Math.sin(tag.getRotation().getRadians()) * dist, new Rotation2d(0)));
    // Pose2d right = tag.transformBy(new
    // Transform2d(Math.cos(tag.getRotation().getRadians()) * -dist,
    // Math.sin(tag.getRotation().getRadians()) * -dist, new Rotation2d(0)));

    // double ldx = left.getX() - robot.getX();
    // double ldy = left.getY() - robot.getY();
    // double ldistance = Math.sqrt(ldx * ldx + ldy * ldy);

    // if (ldistance < closestDistance) {
    // closestDistance = ldistance;
    // closest = left;
    // }

    // double rdx = right.getX() - robot.getX();
    // double rdy = right.getY() - robot.getY();
    // double rdistance = Math.sqrt(rdx * rdx + rdy * rdy);

    // if (rdistance < closestDistance) {
    // closestDistance = rdistance;
    // closest = right;
    // }
    // }

    if (redAlliance)
      robot = new Pose2d(Constants.FIELD_WIDTH - robot.getX(), robot.getY(),
          robot.getRotation());

    for (int i = 0; i < stalkPositions.length; i++) {
      double dx = stalkPositions[i].getX() - robot.getX();
      double dy = stalkPositions[i].getY() - robot.getY();
      double distance = Math.sqrt(dx * dx + dy * dy);
      if (distance < closestDistance) {
        closestDistance = distance;
        closest = stalkPositions[i];
      }
    }

    if (redAlliance)
      closest = new Pose2d(Constants.FIELD_WIDTH - closest.getX(), closest.getY(),
          new Rotation2d(closest.getRotation().getRadians()));

    ArrayList<Pose2d> show = new ArrayList<Pose2d>();

    show.add(closest);
    dataSubsystem.setRobotPath(show);

    // pathfindingCommand = AutoBuilder.pathfindToPose(
    // closest,
    // constraints);

    // Create a list of waypoints from poses. Each pose represents one waypoint.
    // The rotation component of the pose should be the direction of travel. Do not
    // use holonomic rotation.
    List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(robot, closest);

    // Create the path using the waypoints created above
    path = new PathPlannerPath(
        waypoints,
        constraints,
        null, // The ideal starting state, this is only relevant for pre-planned paths, so can
              // be null for on-the-fly paths.
        new GoalEndState(0.0, closest.getRotation()) // Goal end state. You can set a holonomic rotation here. If
                                                     // using a differential drivetrain, the rotation will have no
                                                     // effect.
    );

    path.preventFlipping = true;

    pathfindingCommand = AutoBuilder.followPath(path);
  }

  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return null;
  }
}
