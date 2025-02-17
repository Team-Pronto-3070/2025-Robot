package frc.robot.utils;

import com.ctre.phoenix6.configs.ClosedLoopRampsConfigs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.OpenLoopRampsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants;

public class SwerveModule {

        private final TalonFX driveMotor;
        private final TalonFX turnMotor;
        private final CANcoder turningAbsoluteEncoder;

        // private final SparkPIDController turningPID;
        private final double chassisAngularOffset;
        private final SimpleMotorFeedforward driveFeedforward;
        private final SimpleMotorFeedforward turnFeedforward;

        private Rotation2d lastAngle;

        public SwerveModule(int driveMotorID, int turnMotorID, int encoderId, double encoderOffset) {

                turningAbsoluteEncoder = new CANcoder(encoderId);

                // Initialize Drive Motor
                driveMotor = new TalonFX(driveMotorID);

                var driveConfig = driveMotor.getConfigurator();

                var motorConfigs = new MotorOutputConfigs();
                motorConfigs.Inverted = Constants.Swerve.Drive.motorInvert;
                motorConfigs.NeutralMode = Constants.Swerve.Drive.neutralMode;
                driveConfig.apply(motorConfigs);

                // set slot 0 gains
                var slot0Configs = new Slot0Configs();
                slot0Configs.kV = Constants.Swerve.Drive.PID.F;
                slot0Configs.kP = Constants.Swerve.Drive.PID.P;
                slot0Configs.kI = Constants.Swerve.Drive.PID.I;
                slot0Configs.kD = Constants.Swerve.Drive.PID.D;

                // apply gains, 50 ms total timeout
                driveConfig.apply(slot0Configs, 0.050);

                var currentLimitsConfig = new CurrentLimitsConfigs();
                currentLimitsConfig.SupplyCurrentLimitEnable = Constants.Swerve.Drive.enableCurrentLimit;
                currentLimitsConfig.SupplyCurrentLimit = Constants.Swerve.Drive.continuousCurrentLimit;
                // currentLimitsConfig.SupplyTimeThreshold =
                // Constants.Swerve.Drive.peakCurrentDuration;
                // currentLimitsConfig.SupplyCurrentThreshold =
                // Constants.Swerve.Drive.peakCurrentLimit;
                driveConfig.apply(currentLimitsConfig);

                var OpenLoopRampsConfigs = new OpenLoopRampsConfigs();
                OpenLoopRampsConfigs.DutyCycleOpenLoopRampPeriod = Constants.Swerve.Drive.openLoopRamp;
                driveConfig.apply(OpenLoopRampsConfigs);

                var ClosedLoopRampsConfigs = new ClosedLoopRampsConfigs();
                ClosedLoopRampsConfigs.DutyCycleClosedLoopRampPeriod = Constants.Swerve.Drive.closedLoopRamp;
                driveConfig.apply(ClosedLoopRampsConfigs);

                // Initialize Turn Motor
                turnMotor = new TalonFX(driveMotorID);

                var turnConfig = turnMotor.getConfigurator();

                var turnConfigs = new MotorOutputConfigs();
                turnConfigs.Inverted = Constants.Swerve.Turn.motorInvert;
                turnConfigs.NeutralMode = Constants.Swerve.Turn.neutralMode;
                turnConfig.apply(turnConfigs);

                // set slot 0 gains
                var turnSlot0Configs = new Slot0Configs();
                turnSlot0Configs.kV = Constants.Swerve.Turn.PID.F;
                turnSlot0Configs.kP = Constants.Swerve.Turn.PID.P;
                turnSlot0Configs.kI = Constants.Swerve.Turn.PID.I;
                turnSlot0Configs.kD = Constants.Swerve.Turn.PID.D;

                // apply gains, 50 ms total timeout
                turnConfig.apply(turnSlot0Configs, 0.050);

                var turnCurrentLimitsConfig = new CurrentLimitsConfigs();
                turnCurrentLimitsConfig.SupplyCurrentLimitEnable = Constants.Swerve.Turn.enableCurrentLimit;
                turnCurrentLimitsConfig.SupplyCurrentLimit = Constants.Swerve.Turn.continuousCurrentLimit;
                // turnCurrentLimitsConfig.SupplyTimeThreshold =
                // Constants.Swerve.Turn.peakCurrentDuration;
                // turnCurrentLimitsConfig.SupplyCurrentThreshold =
                // Constants.Swerve.Turn.peakCurrentLimit;
                turnConfig.apply(turnCurrentLimitsConfig);

                var turnOpenLoopRampsConfigs = new OpenLoopRampsConfigs();
                turnOpenLoopRampsConfigs.DutyCycleOpenLoopRampPeriod = Constants.Swerve.Turn.openLoopRamp;
                turnConfig.apply(turnOpenLoopRampsConfigs);

                var turnClosedLoopRampsConfigs = new ClosedLoopRampsConfigs();
                turnClosedLoopRampsConfigs.DutyCycleClosedLoopRampPeriod = Constants.Swerve.Turn.closedLoopRamp;
                turnConfig.apply(turnClosedLoopRampsConfigs);

                driveFeedforward = new SimpleMotorFeedforward(
                                Constants.Swerve.Drive.Feedforward.KS,
                                Constants.Swerve.Drive.Feedforward.KV,
                                Constants.Swerve.Drive.Feedforward.KA);

                turnFeedforward = new SimpleMotorFeedforward(
                                Constants.Swerve.Turn.Feedforward.KS,
                                Constants.Swerve.Turn.Feedforward.KV,
                                Constants.Swerve.Turn.Feedforward.KA);

                chassisAngularOffset = encoderOffset;
                lastAngle = new Rotation2d(chassisAngularOffset);
        }

        public void setDesiredState(SwerveModuleState2 rawDesiredState, boolean isOpenLoop) {
                SwerveModuleState optimizedDesiredState = SwerveModuleState.optimize(
                                new SwerveModuleState(
                                                rawDesiredState.speedMetersPerSecond,
                                                rawDesiredState.angle.plus(new Rotation2d(chassisAngularOffset))),
                                new Rotation2d(turningAbsoluteEncoder.getPosition().getValue()));

                if (isOpenLoop) {
                        driveMotor.setControl(
                                        new DutyCycleOut(optimizedDesiredState.speedMetersPerSecond
                                                        / Constants.Swerve.maxSpeed));
                } else {
                        double wheelRPM = ((optimizedDesiredState.speedMetersPerSecond * 60)
                                        / Constants.Swerve.wheelCircumference);
                        double motorRPM = wheelRPM * Constants.Swerve.gearRatio;
                        double motorRPS = motorRPM / 60;
                        // double sensorCounts = motorRPM * (2048.0 / 600.0);
                        driveMotor.setControl(new VelocityDutyCycle(motorRPS)
                                        .withFeedForward(driveFeedforward
                                                        .calculate(optimizedDesiredState.speedMetersPerSecond)));
                }

                // Prevent rotating module if speed is less then 1% in order to prevent
                // jittering
                Rotation2d angle = (Math
                                .abs(optimizedDesiredState.speedMetersPerSecond) <= (Constants.Swerve.maxSpeed * 0.01))
                                                ? lastAngle
                                                : optimizedDesiredState.angle;
                lastAngle = angle;

                // turningPID.setReference(
                // angle.getRadians(),
                // CANSparkMax.ControlType.kPosition,
                // 0,
                // Constants.Swerve.Turn.KV * rawDesiredState.omegaRadPerSecond);

                // create a position closed-loop request, voltage output, slot 0 configs
                final PositionDutyCycle turnPosition = new PositionDutyCycle(angle.getRadians());

                // set position to 10 rotations
                turnMotor.setControl(turnPosition
                                .withFeedForward(
                                                turnFeedforward.calculate(optimizedDesiredState.speedMetersPerSecond)));
        }

        public SwerveModulePosition getPosition() {
                return new SwerveModulePosition(
                                (driveMotor.getRotorPosition().getValueAsDouble() // motor rotations
                                                / Constants.Swerve.gearRatio) // wheel rotations
                                                * Constants.Swerve.wheelCircumference, // wheel meters
                                new Rotation2d(turningAbsoluteEncoder.getPosition().getValueAsDouble()
                                                - chassisAngularOffset));
        }

        public SwerveModuleState2 getState() {
                return new SwerveModuleState2(
                                (driveMotor.getRotorVelocity().getValueAsDouble() // motor rot/s
                                                / Constants.Swerve.gearRatio) // wheel rot/s
                                                * Constants.Swerve.wheelCircumference, // wheel surface speed in meters
                                                                                       // per second
                                new Rotation2d(turningAbsoluteEncoder.getPosition().getValueAsDouble()
                                                - chassisAngularOffset),
                                turningAbsoluteEncoder.getVelocity().getValueAsDouble());
        }
}