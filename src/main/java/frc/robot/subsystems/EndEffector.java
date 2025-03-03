package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;

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

        var coralCongfig = coralMotor.getConfigurator();
        var algaeCongfig = algaeMotor.getConfigurator();
        var armCongfig = armMotor.getConfigurator();

        var config = new TalonFXConfiguration();

        // TODO: add Motor Configs
    }

    public void intakeCoral() {
        // spin motor until beam break detects coral has been loaded
        this.runOnce(() -> coralMotor.set(1)).until(() -> coralBeamBreak.get()).andThen(() -> coralMotor.set(0));
    }

    public void launchCoral() {
        this.runOnce(() -> coralMotor.set(1)).until(() -> !coralBeamBreak.get()).andThen(() -> coralMotor.set(0));
    }

    public void intakeAlgae() {
        this.runOnce(() -> algaeMotor.set(-1)).until(() -> {return algaeMotor.getStatorCurrent().getValueAsDouble() > 10;}).andThen(() -> algaeMotor.set(0));
    }

    public void launchAlgae() {
        this.runOnce(() -> algaeMotor.set(1)).withTimeout(1).andThen(() -> algaeMotor.set(0));
    }
}
