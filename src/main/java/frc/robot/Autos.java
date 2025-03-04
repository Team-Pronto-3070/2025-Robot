package frc.robot;

import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.function.Supplier;
import java.util.stream.Collectors;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public class Autos extends SubsystemBase {
    private final SendableChooser<Command> autoChooser;
    private final SwerveSubsystem swerve;

    private SwerveRequest.ApplyRobotSpeeds drive = new SwerveRequest.ApplyRobotSpeeds();

    private String autoName, newAutoName;
    private List<Pose2d> poses = new ArrayList<>();

    public Autos(SwerveSubsystem swerve, ElevatorSubsystem elevator) {
        this.swerve = swerve;

        // Load the RobotConfig from the GUI settings. You should probably
        // store this in your Constants file
        RobotConfig config = null;
        try {
            config = RobotConfig.fromGUISettings();
        } catch (Exception e) {
            // Handle exception as needed
            e.printStackTrace();
        }

        // Configure AutoBuilder last
        AutoBuilder.configure(
                this::getPose, // Robot pose supplier
                this::resetPose, // Method to reset odometry (will be called if your auto has a starting pose)
                this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
                (speeds, feedforwards) -> driveRobotRelative(speeds), // Method that will drive the robot given ROBOT
                                                                      // RELATIVE ChassisSpeeds. Also optionally outputs
                                                                      // individual module feedforwards
                new PPHolonomicDriveController( // PPHolonomicController is the built in path following controller for
                                                // holonomic drive trains
                        new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
                        new PIDConstants(5.0, 0.0, 0.0) // Rotation PID constants
                ),
                config, // The robot configuration
                () -> {
                    // Boolean supplier that controls when the path will be mirrored for the red
                    // alliance
                    // This will flip the path being followed to the red side of the field.
                    // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

                    var alliance = DriverStation.getAlliance();
                    if (alliance.isPresent()) {
                        return alliance.get() == DriverStation.Alliance.Red;
                    }
                    return false;
                },
                this // Reference to this subsystem to set requirements
        );

        // NamedCommands.registerCommand("intake",
        // Commands.sequence(
        // new InstantCommand(() -> SmartDashboard.putBoolean("intake command running",
        // true)),
        // intake.run(() -> intake.set(1)).until(new Trigger(() -> intake.hasNote())),
        // Commands.defer(() -> intake.run(() -> intake.set(-0.1))
        // .until(new Trigger(() -> intake.hasNote()).debounce(0.04)), Set.of(intake)),
        // Commands.defer(() -> intake.run(() -> intake.set(-0.12))
        // .until(new Trigger(() -> !intake.hasNote()).debounce(0.08)), Set.of(intake)),
        // Commands.defer(() -> intake.run(() -> intake.set(0.1))
        // .until(new Trigger(() -> intake.hasNote()).debounce(0.04)), Set.of(intake)),
        // intake.runOnce(intake::stop))
        // .finallyDo(() -> SmartDashboard.putBoolean("intake command running",
        // false)));

        // NamedCommands.registerCommand("prepShooter",
        // shooter.prepSpeakerCommandOnce());

        // NamedCommands.registerCommand("launchNote", Commands.sequence(
        // shooter.prepSpeakerCommandOnce(),
        // Commands.sequence(
        // Commands.defer(() -> Commands.waitUntil(new
        // Trigger(shooter::atTarget).debounce(0.2)),
        // Set.of()),
        // intake.run(() -> intake.set(0.2)).withTimeout(0.2),
        // intake.runOnce(intake::stop),
        // shooter.runOnce(shooter::stop),
        // Commands.either(ampBar.homeCommand(), Commands.none(),
        // () -> shooter.target == ShooterSubsystem.Target.AMP),
        // new InstantCommand(() -> shooter.target = ShooterSubsystem.Target.NONE))));

        // NamedCommands.registerCommand("stopIntake", intake.runOnce(intake::stop));
        // NamedCommands.registerCommand("stopShooter", shooter.runOnce(shooter::stop));

        // NamedCommands.registerCommand("defensiveIntake", intake.run(() ->
        // intake.set(1)));
        // NamedCommands.registerCommand("defensiveShooter", shooter.run(() ->
        // shooter.set(300)));

        autoChooser = AutoBuilder.buildAutoChooser();

        SmartDashboard.putData("Auto Chooser", autoChooser);
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected()
                .andThen(new InstantCommand(() -> {
                    var alliance = DriverStation.getAlliance();
                    if (alliance.isPresent()) {
                        if (alliance.get() == DriverStation.Alliance.Red) {
                            swerve.resetPose(getPose().rotateBy(Rotation2d.fromDegrees(180)));
                        }
                    }
                }));
    }

    @Override
    public void periodic() {
        newAutoName = autoChooser.getSelected().getName();
        if (autoName != newAutoName) {
            autoName = newAutoName;
            if (AutoBuilder.getAllAutoNames().contains(autoName)) {
                // System.out.println("Displaying " + autoName);
                try {
                    List<PathPlannerPath> pathPlannerPaths = PathPlannerAuto.getPathGroupFromAutoFile(autoName);
                    poses.clear();
                    for (PathPlannerPath path : pathPlannerPaths) {
                        poses.addAll(path.getAllPathPoints().stream().map(
                                point -> new Pose2d(point.position.getX(), point.position.getY(), new Rotation2d()))
                                .collect(Collectors.toList()));
                    }
                } catch (Exception e) {
                    // TODO: handle exception
                    System.out.println("Auto Display Exception ): " + e);
                }
            }
        }
    }

    public List<Pose2d> getPath() {
        return poses;
    }

    public Pose2d getPose() {
        return swerve.getState().Pose;
    }

    public void resetPose(Pose2d newPose) {
        swerve.resetPose(newPose);
    }

    public ChassisSpeeds getRobotRelativeSpeeds() {
        return swerve.getKinematics().toChassisSpeeds(swerve.getState().ModuleStates);
    }

    public void driveRobotRelative(ChassisSpeeds speeds) {
        System.out.println(speeds.toString());
        swerve.setControl(
        drive.withSpeeds(speeds));
    }

    public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
        return run(() -> swerve.setControl(requestSupplier.get()));
    }
}