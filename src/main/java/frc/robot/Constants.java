// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

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

  public static final int gyroID = 13;

  public static final class OI {
    public static final int driverPort = 0;
    public static final int operatorPort = 1;
    public static final double deadband = 0.04;
    public static final double triggerDeadband = 0.3;

    public static final double slowSpeed = 0.1;
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
