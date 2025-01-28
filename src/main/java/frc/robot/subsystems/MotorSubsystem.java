// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.ctre.phoenix6.configs.ClosedLoopRampsConfigs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.OpenLoopRampsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class MotorSubsystem extends SubsystemBase {

  SparkMax turnMotor;
  TalonFX driveMotor;

  /** Creates a new ExampleSubsystem. */
  public MotorSubsystem() {
    turnMotor = new SparkMax(3, SparkMax.MotorType.kBrushless);
    driveMotor = new TalonFX(5);

    SparkMaxConfig turnConfig = new SparkMaxConfig();
    TalonFXConfigurator driveConfig = driveMotor.getConfigurator();

    turnConfig
        .smartCurrentLimit(10)
        .idleMode(IdleMode.kBrake);

    turnMotor.configure(turnConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    var motorConfigs = new MotorOutputConfigs();
    motorConfigs.Inverted = InvertedValue.Clockwise_Positive;
    motorConfigs.NeutralMode = NeutralModeValue.Brake;
    driveConfig.apply(motorConfigs);

    var currentLimitsConfig = new CurrentLimitsConfigs();
    currentLimitsConfig.SupplyCurrentLimitEnable = true;
    currentLimitsConfig.SupplyCurrentLimit = 10;
    driveConfig.apply(currentLimitsConfig);
  }

  public Command turnStart() {
    return runOnce(
        () -> {
          turnMotor.set(0.1);
        });
  }

  public Command turnStop() {
    return runOnce(
        () -> {
          turnMotor.set(0);
        });
  }

  public Command driveStart() {
    return runOnce(
        () -> {
          driveMotor.set(0.5);
        });
  }

  public Command driveStop() {
    return runOnce(
        () -> {
          driveMotor.set(0);
        });
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
