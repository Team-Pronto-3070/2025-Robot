package frc.robot.subsystems;

import java.util.Set;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class EndEffector extends SubsystemBase {

    private final TalonFX coralMotor;
    // private final TalonFX algaeMotor;
    private final TalonFX armMotor;

    private final DigitalInput coralBeamBreak;

    private double coralDelay = 0.005;

    public EndEffector() {
        coralBeamBreak = new DigitalInput(Constants.EndEffector.beamBreakPort);

        coralMotor = new TalonFX(Constants.EndEffector.coralID);
        // algaeMotor = new TalonFX(Constants.EndEffector.algaeID);
        armMotor = new TalonFX(Constants.EndEffector.algaeArmID);

        var coralConfig = coralMotor.getConfigurator();
        // var algaeConfig = algaeMotor.getConfigurator();
        var armConfig = armMotor.getConfigurator();

        var coralConfigs = new MotorOutputConfigs();
        coralConfigs.Inverted = InvertedValue.CounterClockwise_Positive;
        coralConfigs.NeutralMode = NeutralModeValue.Brake;
        coralConfig.apply(coralConfigs);

        // var algaeConfigs = new MotorOutputConfigs();
        // algaeConfigs.Inverted = InvertedValue.Clockwise_Positive;
        // algaeConfigs.NeutralMode = NeutralModeValue.Brake;
        // algaeConfig.apply(algaeConfigs);

        var armConfigs = new MotorOutputConfigs();
        armConfigs.Inverted = InvertedValue.CounterClockwise_Positive;
        armConfigs.NeutralMode = NeutralModeValue.Brake;
        armConfig.apply(armConfigs);

        var currentLimitsConfig = new CurrentLimitsConfigs();
        currentLimitsConfig.SupplyCurrentLimitEnable = true;
        currentLimitsConfig.SupplyCurrentLimit = 80;
        coralConfig.apply(currentLimitsConfig);
        // algaeConfig.apply(currentLimitsConfig);
        armConfig.apply(currentLimitsConfig);

        // in init function
        var talonFXConfigs = new TalonFXConfiguration();

        // set slot 0 gains
        var slot0Configs = talonFXConfigs.Slot0;
        slot0Configs.kS = 0.25; // Add 0.25 V output to overcome static friction
        slot0Configs.kV = 0.12; // A velocity target of 1 rps results in 0.12 V
        // output
        slot0Configs.kA = 0.01; // An acceleration of 1 rps/s requires 0.01 V output
        slot0Configs.kP = 4.8; // A position error of 2.5 rotations results in
        // 2 V output
        slot0Configs.kI = 0; // no output for integrated error
        slot0Configs.kD = 0.1; // A velocity error of 1 rps results in 0.1 V output

        // set Motion Magic settings
        var motionMagicConfigs = talonFXConfigs.MotionMagic;
        motionMagicConfigs.MotionMagicCruiseVelocity = 80; // Target cruise velocity
        // of 80 rps
        motionMagicConfigs.MotionMagicAcceleration = 100; // Target acceleration of
        // 160 rps/s (0.5 seconds)
        motionMagicConfigs.MotionMagicJerk = 1000; // Target jerk of 1600 rps/s/s (0
        // 1 seconds)

        armConfig.apply(talonFXConfigs);

        // armMotor.setPosition(0);
    }

    public Command intakeCoral() {
        return Commands.sequence(
                this.runOnce(() -> System.out.println("Intake")),
                this.runOnce(() -> coralMotor.set(0.4)),
                Commands.defer(() -> Commands.waitUntil(new Trigger(() -> !coralBeamBreak.get()).debounce(coralDelay)),
                        Set.of(this)),
                this.runOnce(() -> coralMotor.set(0)));
    }

    public Command launchCoral() {
        return Commands.sequence(
                this.runOnce(() -> System.out.println("Launch")),
                this.runOnce(() -> coralMotor.set(0.4)),
                Commands.defer(() -> Commands.waitUntil(new Trigger(() -> coralBeamBreak.get()).debounce(0.2)),
                        Set.of(this)),
                this.runOnce(() -> coralMotor.set(0)));
    }

    public void setCoral(double speed) {
        coralMotor.set(speed);
    }

    // public Command intakeAlgae() {
    // return this.runOnce(() -> algaeMotor.set(-1)).until(() -> {
    // return algaeMotor.getStatorCurrent().getValueAsDouble() > 10;
    // }).andThen(() -> algaeMotor.set(0));

    // public Command launchAlgae() {
    // return this.runOnce(() -> algaeMotor.set(1)).withTimeout(1).andThen(() ->
    // algaeMotor.set(0));
    // }

    // public void setAlgae(double speed) {
    // algaeMotor.set(speed);
    // }

    public void armUp() {
        // armUp = true;
        var request = new MotionMagicVoltage(-0.1);
        armMotor.setControl(request);

        // this.run(() -> armMotor.set(0.1));
        // armMotor.set(0.1);
        // this.runEnd(() -> armMotor.set(0.1), () -> armMotor.set(0)).until(() ->
        // armMotor.getSupplyCurrent().getValueAsDouble() > 4.5).schedule();
    }

    public void armDown() {
        // armUp = true;
        var request = new MotionMagicVoltage(-0.52);
        armMotor.setControl(request);

        // this.run(() -> armMotor.set(-0.1));
        // armMotor.set(-0.1);
        // this.runEnd(() -> armMotor.set(-0.1), () -> armMotor.set(0)).until(() ->
        // armMotor.getSupplyCurrent().getValueAsDouble() > 4.5).schedule();
    }

    public void calibrateArm() {
        this.runEnd(() -> armMotor.set( 0.1), () -> {armMotor.set(0); armMotor.setPosition(0);})
                .until(() -> armMotor.getSupplyCurrent().getValueAsDouble() > 4.0).withTimeout(1).schedule();
    }

    public Boolean hasCoral() {
        return !coralBeamBreak.get();
    }

    public void stop() {
        coralMotor.stopMotor();
        // algaeMotor.stopMotor();
        // armMotor.stopMotor();
    }

    public void increaseCoralDelay() {
        coralDelay += 0.05;
    }

    public void decreaseCoralDelay() {
        coralDelay -= 0.05;
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("Has Coral", !coralBeamBreak.get());

        coralDelay = SmartDashboard.getNumber("Coral Delay", 0.005);
        SmartDashboard.putNumber("Coral Delay", coralDelay);
    }
}
