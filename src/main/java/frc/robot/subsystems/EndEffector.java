package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.DigitalInput;

public class EndEffector extends SubsystemBase {

    private final TalonFX coralMotor;
    private final TalonFX algaeMotor;
    private final TalonFX armMotor;

    private final DigitalInput coralBeamBreak;

    public EndEffector() {
        coralMotor = new TalonFX(Constants.EndEffector.coralID);
        algaeMotor = new TalonFX(Constants.EndEffector.algaeID);
        armMotor = new TalonFX(Constants.EndEffector.algaeArmID);

        coralBeamBreak = new DigitalInput(Constants.EndEffector.beamBreakPort);

        var coralConfig = coralMotor.getConfigurator();
        var algaeConfig = algaeMotor.getConfigurator();
        var armConfig = armMotor.getConfigurator();

        var coralConfigs = new MotorOutputConfigs();
        coralConfigs.Inverted = InvertedValue.Clockwise_Positive;
        coralConfigs.NeutralMode = NeutralModeValue.Brake;
        coralConfig.apply(coralConfigs);

        var algaeConfigs = new MotorOutputConfigs();
        algaeConfigs.Inverted = InvertedValue.Clockwise_Positive;
        algaeConfigs.NeutralMode = NeutralModeValue.Brake;
        algaeConfig.apply(algaeConfigs);

        var armConfigs = new MotorOutputConfigs();
        armConfigs.Inverted = InvertedValue.Clockwise_Positive;
        armConfigs.NeutralMode = NeutralModeValue.Brake;
        armConfig.apply(armConfigs);

        var currentLimitsConfig = new CurrentLimitsConfigs();
        currentLimitsConfig.SupplyCurrentLimitEnable = true;
        currentLimitsConfig.SupplyCurrentLimit = 10;
        coralConfig.apply(currentLimitsConfig);
        algaeConfig.apply(currentLimitsConfig);
        armConfig.apply(currentLimitsConfig);

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
        motionMagicConfigs.MotionMagicCruiseVelocity = 80; // Target cruise velocity of 80 rps
        motionMagicConfigs.MotionMagicAcceleration = 160; // Target acceleration of 160 rps/s (0.5 seconds)
        motionMagicConfigs.MotionMagicJerk = 1600; // Target jerk of 1600 rps/s/s (0.1 seconds)

        armConfig.apply(talonFXConfigs);
    }

    public Command intakeCoral() {
        // spin motor until beam break detects coral has been loaded
        return this.runOnce(() -> coralMotor.set(1)).until(() -> coralBeamBreak.get()).andThen(() -> coralMotor.set(0));
    }

    public Command launchCoral() {
        return this.runOnce(() -> coralMotor.set(1)).until(() -> !coralBeamBreak.get()).andThen(() -> coralMotor.set(0));
    }

    public Command intakeAlgae() {
        return this.runOnce(() -> algaeMotor.set(-1)).until(() -> {
            return algaeMotor.getStatorCurrent().getValueAsDouble() > 10;
        }).andThen(() -> algaeMotor.set(0));
    }

    public Command launchAlgae() {
        return this.runOnce(() -> algaeMotor.set(1)).withTimeout(1).andThen(() -> algaeMotor.set(0));
    }

    public void stop() {
        coralMotor.stopMotor();
        algaeMotor.stopMotor();
        armMotor.stopMotor();
    }
}
