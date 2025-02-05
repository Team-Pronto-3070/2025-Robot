// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.MotorSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

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
  private final MotorSubsystem motorSubsystem = new MotorSubsystem();

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the trigger bindings

    // motorSubsystem.setDefaultCommand(motorSubsystem.run(() -> {
    // motorSubsystem.driveAnalog(oi.driveAnalog.getAsDouble());
    // }));

    // oi.rt.whileTrue(motorSubsystem.run(() -> {
    //   System.out.println(oi.driveAnalog.getAsDouble());
    //   // motorSubsystem.runOnce(() -> {
    //   //   motorSubsystem.driveStart(0.1);
    //   // });
    //   motorSubsystem.driveStart(oi.driveAnalog);
    // }));

    // oi.rt.whileTrue(motorSubsystem.run(() ->
    // {System.out.println(oi.driveAnalog.getAsDouble());}));

    oi.a.onTrue(motorSubsystem.driveStart(0.1));
    oi.a.onFalse(motorSubsystem.driveStop());
    oi.b.onTrue(motorSubsystem.driveStart(-0.1));
    oi.b.onFalse(motorSubsystem.driveStop());

    oi.x.onTrue(motorSubsystem.turnStart(0.1));
    oi.x.onFalse(motorSubsystem.turnStop());
    oi.y.onTrue(motorSubsystem.turnStart(-0.1));
    oi.y.onFalse(motorSubsystem.turnStop());
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be
   * created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
   * an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link
   * CommandXboxController
   * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or
   * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // // An example command will be run in autonomous
    // return Autos.exampleAuto(motorSubsystem);
    return null;
  }
}
