package frc.robot;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.MotorSubsystem;

public class OI {
    private final CommandXboxController driver = new CommandXboxController(
            OperatorConstants.kDriverControllerPort);

    public final Trigger a;
    public final Trigger b;
    public final Trigger x;
    public final Trigger y;

    public final Trigger rt;

    public final DoubleSupplier driveAnalog;

    public OI() {
        a = driver.a();
        b = driver.b();
        x = driver.x();
        y = driver.y();

        rt = driver.rightTrigger();
        driveAnalog = driver::getRightTriggerAxis;
    }

}
