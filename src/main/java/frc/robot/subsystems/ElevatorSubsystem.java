package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ElevatorSubsystem extends SubsystemBase {

    public final TalonFX leftMotor;
    public final TalonFX rightMotor;

    private int level = 0;
    private final double[] levels = {
            Constants.Elevator.L0,
            Constants.Elevator.L1,
            Constants.Elevator.L2,
            Constants.Elevator.L3,
            Constants.Elevator.L4,
    };

    public ElevatorSubsystem() {
        // Initialize the elevator subsystem here
        leftMotor = new TalonFX(Constants.Elevator.leftID);
        rightMotor = new TalonFX(Constants.Elevator.rightID);

        TalonFXConfigurator leftConfig = leftMotor.getConfigurator();
        TalonFXConfigurator rightConfig = rightMotor.getConfigurator();

        var leftConfigs = new MotorOutputConfigs();
        leftConfigs.Inverted = InvertedValue.CounterClockwise_Positive;
        leftConfigs.NeutralMode = NeutralModeValue.Brake;
        leftConfig.apply(leftConfigs);

        var rightConfigs = new MotorOutputConfigs();
        rightConfigs.Inverted = InvertedValue.Clockwise_Positive;
        rightConfigs.NeutralMode = NeutralModeValue.Brake;
        rightConfig.apply(rightConfigs);

        var currentLimitsConfig = new CurrentLimitsConfigs();
        currentLimitsConfig.SupplyCurrentLimitEnable = true;
        currentLimitsConfig.SupplyCurrentLimit = 10;
        leftConfig.apply(currentLimitsConfig);
        rightConfig.apply(currentLimitsConfig);

        // in init function
        var talonFXConfigs = new TalonFXConfiguration();

        // set slot 0 gains
        var slot0Configs = talonFXConfigs.Slot0;
        slot0Configs.kS = 0.25; // Add 0.25 V output to overcome static friction
        slot0Configs.kV = 0.12; // A velocity target of 1 rps results in 0.12 V output
        slot0Configs.kA = 0.01; // An acceleration of 1 rps/s requires 0.01 V output
        slot0Configs.kP = 4.8; // A position error of 2.5 rotations results in 12 V output
        slot0Configs.kI = 0; // no output for integrated error
        slot0Configs.kD = 0.1; // A velocity error of 1 rps results in 0.1 V output

        // set Motion Magic settings
        var motionMagicConfigs = talonFXConfigs.MotionMagic;
        motionMagicConfigs.MotionMagicCruiseVelocity = 200; // Target cruise velocity of 80 rps
        motionMagicConfigs.MotionMagicAcceleration = 160; // Target acceleration of 160 rps/s (0.5 seconds)
        // motionMagicConfigs.MotionMagicCruiseVelocity = 80; // Target cruise velocity
        // of 80 rps
        // motionMagicConfigs.MotionMagicAcceleration = 160; // Target acceleration of
        // 160 rps/s (0.5 seconds)
        motionMagicConfigs.MotionMagicJerk = 1600; // Target jerk of 1600 rps/s/s (0.1 seconds)

        leftConfig.apply(talonFXConfigs);
        rightConfig.apply(talonFXConfigs);

        leftMotor.setPosition(0);
        rightMotor.setPosition(0);

        // this.runEnd(
        // () -> {
        // leftMotor.set(10);
        // },
        // () -> {
        // leftMotor.set(0);
        // }).until(() -> {
        // return leftMotor.getSupplyCurrent().getValueAsDouble() > 30;
        // }).andThen(this.runOnce(() -> {
        // leftMotor.setPosition(0);
        // rightMotor.setPosition(0);
        // }));
    }

    // Move the elevator up by a specified amount (in sensor units)
    private void goTo(double targetHeight) {
        final MotionMagicVoltage m_request = new MotionMagicVoltage(0);

        leftMotor.setControl(m_request.withPosition(
                Math.min(Math.max(targetHeight, Constants.Elevator.minHeight),
                        Constants.Elevator.maxHeight)
                        * Constants.Elevator.inToR));

        rightMotor.setControl(m_request.withPosition(
                Math.min(Math.max(targetHeight, Constants.Elevator.minHeight),
                        Constants.Elevator.maxHeight)
                        * -Constants.Elevator.inToR));
    }

    // Move the elevator up by a specified amount (in sensor units)
    public void setLevel(int lvl) {
        level = lvl;
        double targetHeight = levels[lvl];

        goTo(targetHeight);
    }

    // Move the elevator up by a specified amount (in sensor units)
    public void moveUp() {
        setLevel(Math.min(level + 1, levels.length - 1));
    }

    // Move the elevator down to the bottom
    public void moveDown() {
        setLevel(Math.max(level - 1, 0));
    }

    public void stop() {
        leftMotor.stopMotor();
        rightMotor.stopMotor();
    }

    @Override
    public void periodic() {
        // if (leftMotor.getSupplyCurrent().getValueAsDouble() > 25
        // || rightMotor.getSupplyCurrent().getValueAsDouble() > 25) {
        // leftMotor.setPosition(0);
        // rightMotor.setPosition(0);
        // }
    }
}
