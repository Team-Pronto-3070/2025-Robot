// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import java.lang.reflect.Array;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class MotorSubsystem extends SubsystemBase {

  // SparkMax turnMotor;
  TalonFX turnMotor;
  TalonFX driveMotor;

  Double turnSpeed;
  Double driveSpeed;

  /** Creates a new ExampleSubsystem. */
  public MotorSubsystem() {
    // turnMotor = new SparkMax(3, SparkMax.MotorType.kBrushless);
    turnMotor = new TalonFX(2);
    driveMotor = new TalonFX(1);

    driveSpeed = 0.0;
    turnSpeed = 0.0;

    TalonFXConfigurator turnConfig = driveMotor.getConfigurator();
    TalonFXConfigurator driveConfig = driveMotor.getConfigurator();

    var motorConfigs = new MotorOutputConfigs();
    motorConfigs.Inverted = InvertedValue.Clockwise_Positive;
    motorConfigs.NeutralMode = NeutralModeValue.Brake;
    turnConfig.apply(motorConfigs);
    driveConfig.apply(motorConfigs);

    var currentLimitsConfig = new CurrentLimitsConfigs();
    currentLimitsConfig.SupplyCurrentLimitEnable = true;
    currentLimitsConfig.SupplyCurrentLimit = 10;
    turnConfig.apply(currentLimitsConfig);
    driveConfig.apply(currentLimitsConfig);
  }

  public void turnStart(double speed) {
    turnMotor.set(speed);
    turnSpeed = speed;
  }

  public void turnStop() {
    turnMotor.set(0);
    turnSpeed = 0.0;
  }

  public void driveStart(Double speed) {
    driveMotor.set(speed);
    driveSpeed = speed;
  }

  public void driveStop() {
    driveMotor.set(0);
    driveSpeed = 0.0;
  }

  public Command print() {
    return runOnce(() -> {
      System.out.println("Left Trigger Changed: ");
    });
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    Double[] values = {driveSpeed, turnSpeed};
    SmartDashboard.putNumberArray("Motor Speeds", values);
    
    SmartDashboard.putNumber("Drive Speed", driveSpeed);
    SmartDashboard.putNumber("Turn Speed", turnSpeed);
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
