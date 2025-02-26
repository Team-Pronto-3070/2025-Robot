package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ElevatorSubsystem extends SubsystemBase {

    public final TalonFX leftMotor;
    public final TalonFX rightMotor;

    public ElevatorSubsystem() {
        // Initialize the elevator subsystem here

        leftMotor = new TalonFX(Constants.Elevator.leftID);
        rightMotor = new TalonFX(Constants.Elevator.rightID);

        TalonFXConfigurator leftConfig = leftMotor.getConfigurator();
        TalonFXConfigurator rightConfig = rightMotor.getConfigurator();

        var leftConfigs = new MotorOutputConfigs();
        leftConfigs.Inverted = InvertedValue.Clockwise_Positive ;
        leftConfigs.NeutralMode = NeutralModeValue.Brake;
        leftConfig.apply(leftConfigs);

        var rightConfigs = new MotorOutputConfigs();
        rightConfigs.Inverted = InvertedValue.CounterClockwise_Positive;
        rightConfigs.NeutralMode = NeutralModeValue.Brake;
        rightConfig.apply(rightConfigs);

        var currentLimitsConfig = new CurrentLimitsConfigs();
        currentLimitsConfig.SupplyCurrentLimitEnable = true;
        currentLimitsConfig.SupplyCurrentLimit = 10;
        leftConfig.apply(currentLimitsConfig);
        rightConfig.apply(currentLimitsConfig);
    }

    public void set(double speed) {
        leftMotor.set(speed);
        rightMotor.set(speed);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }

}
