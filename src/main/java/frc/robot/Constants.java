// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public static final double FIELD_HEIGHT = 8.052;
  public static final double FIELD_WIDTH = 17.548;

  public static final class OI {
    public static final int driverPort = 0;
    public static final int operatorPort = 1;
    public static final double deadband = 0.04;
    public static final double triggerDeadband = 0.3;

    public static final double slowSpeed = 0.1;
  }

  public static final class Vision {

    public static final class Front {

      public static final String name = "Arducam_Front";
      public static final Transform3d transform = new Transform3d(
          new Translation3d(
              Units.inchesToMeters(13.4), // 13.4 inches forwards from center
              Units.inchesToMeters(3), // 3 inches Right from center
              Units.inchesToMeters(7.4)), // 7.4 inches up from center
          new Rotation3d(
              Units.degreesToRadians(0), // No Roll
              Units.degreesToRadians(20), // 20 Degrees Upwards
              Units.degreesToRadians(0))); // Facing Forwards
    }

    public static final class Rear {

      public static final String name = "Arducam_Rear";
      public static final Transform3d transform = new Transform3d(
          new Translation3d(
              Units.inchesToMeters(-13.5), // 13.5 inches back from center
              Units.inchesToMeters(-0.2), // 0.5 inches Left from center 
              Units.inchesToMeters(16.5)), // 16.5 inches up from center
          new Rotation3d(
              Units.degreesToRadians(0), // No Roll
              Units.degreesToRadians(10), // 10 Degrees Upwards
              Units.degreesToRadians(180))); // Facing Backwards
    }
  }

  public static final class Elevator {
    public static final int leftID = 14;
    public static final int rightID = 13;
    public static final double inToR = 1.67; // ~50/29.75

    public static final double minHeight = 0.0;
    public static final double maxHeight = 31.310;

    // Elevator Heights in inches
    public static final double L0 = 0.0; // intake
    public static final double L1 = 6.9;
    public static final double L2 = 11.32;
    public static final double L3 = 18.92;
    public static final double L4 = 31.0; // elevator max height

    public static final double[] maxSpeeds = {1, 1, 1, 0.7, 0.5};
    public static final double[] acceleration = {10, 10, 8, 5, 3};
  }

  public static final class EndEffector {
    public static final int coralID = 15;
    // public static final int algaeID = 17;
    public static final int algaeArmID = 16;
    public static final int beamBreakPort = 0; // DIO
  }

  public static final class Swerve {
    public static final NeutralModeValue idleMode = NeutralModeValue.Brake;

    public static final double wheelBase = Units.inchesToMeters(24.0 - 3.5); // distance between front and back //
                                                                             // wheels
    public static final double trackWidth = Units.inchesToMeters(26.0 - 3.5); // distance between left and right //
                                                                              // wheels
    public static final double wheelCircumference = Units.inchesToMeters(3) * Math.PI;

    // The MAXSwerve module can be configured with one of three pinion gears: 12T,
    // 13T, or 14T.
    // This changes the drive speed of the module (a pinion gear with more teeth
    // will result in a robot that drives faster).
    private static final int drivingMotorPinionTeeth = 14;
    public static final double gearRatio = (45.0 * 22) / (drivingMotorPinionTeeth * 15);

    public static final double maxSpeed = Units.feetToMeters(15.87 * (14.0 / 13.0)); // meters per second
    public static final double maxAcceleration = 1.19 * 9.81; // traction limited: COF*g (this COF is for blue nitrile
                                                              // on carpet)
    // public static final double maxAngularSpeed = 10.0 * maxSpeed /
    // Math.hypot(wheelBase / 2.0, trackWidth / 2.0);
    public static final double maxAngularSpeed = 1.0 * maxSpeed / Math.hypot(wheelBase / 2.0, trackWidth / 2.0);

    // offsets are in radians
    public static final class FrontLeft {
      public static final int driveID = 12;
      public static final int encoderId = 11;
      public static final int turnID = 10;
      public static final double offset = Math.toRadians(-45);
    }

    public static final class FrontRight {
      public static final int driveID = 3;
      public static final int encoderId = 2;
      public static final int turnID = 1;
      public static final double offset = Math.toRadians(-135);
    }

    public static final class RearLeft {
      public static final int driveID = 9;
      public static final int encoderId = 8;
      public static final int turnID = 7;
      public static final double offset = Math.toRadians(45);
    }

    public static final class RearRight {
      public static final int driveID = 6;
      public static final int encoderId = 5;
      public static final int turnID = 4;
      public static final double offset = Math.toRadians(135);
    }

    public static final class Drive {
      public static final class PID {
        public static final double P = 0.02;
        public static final double I = 0.0;
        public static final double D = 0.0;
        public static final double F = 0.0;
      }

      public static final class Feedforward {
        public static final double KS = 0.0;
        // public static final double KV = 12 / maxSpeed;
        public static final double KV = 0.0;
        // public static final double KA = 12 / maxAcceleration;
        public static final double KA = 0.0;
      }

      public static final int continuousCurrentLimit = 35;
      public static final int peakCurrentLimit = 60;
      public static final double peakCurrentDuration = 0.1;
      public static final boolean enableCurrentLimit = true;

      public static final double openLoopRamp = 0.25;
      public static final double closedLoopRamp = 0.0;

      public static final InvertedValue motorInvert = InvertedValue.CounterClockwise_Positive;
      public static final NeutralModeValue neutralMode = NeutralModeValue.Brake;
    }

    public static final class Turn {
      public static final double encoderPositionFactor = 2 * Math.PI; // radians
      public static final double encoderVelocityFactor = 2 * Math.PI / 60.0; // radians per second
      public static final boolean encoderInvert = true;

      public static final double maxModuleAngularSpeed = Units.rotationsPerMinuteToRadiansPerSecond( // radians per
                                                                                                     // second
          11000.0 * // NEO550 free speed (rpm)
              203.0 /
              9424.0); // gear ratio
      public static final double KV = 12.0 / maxModuleAngularSpeed; // volts * seconds / radians

      public static final class PID {
        public static final double P = 1.0;
        public static final double I = 0.0;
        public static final double D = 0.0;
        public static final double F = 0.0;
      }

      public static final class Feedforward {
        public static final double KS = 0.0;
        // public static final double KV = 12 / maxSpeed;
        public static final double KV = 0.0;
        // public static final double KA = 12 / maxAcceleration;
        public static final double KA = 0.0;
      }

      public static final int continuousCurrentLimit = 35;
      public static final int peakCurrentLimit = 60;
      public static final double peakCurrentDuration = 0.1;
      public static final boolean enableCurrentLimit = true;

      public static final double openLoopRamp = 0.25;
      public static final double closedLoopRamp = 0.0;

      public static final InvertedValue motorInvert = InvertedValue.CounterClockwise_Positive;
      public static final NeutralModeValue neutralMode = NeutralModeValue.Brake;
    }
  }

}
