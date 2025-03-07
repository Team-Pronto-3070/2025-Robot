package frc.robot;

import java.util.function.DoubleSupplier;
import java.util.function.IntSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class OI {

        private CommandXboxController driver;
        private CommandXboxController operator;

        public final DoubleSupplier drive_x;
        public final DoubleSupplier drive_y;
        public final DoubleSupplier drive_rot;

        public final Trigger driveFast;
        public final DoubleSupplier driveBoost;

        public final DoubleSupplier processed_drive_x;
        public final DoubleSupplier processed_drive_y;
        public final DoubleSupplier processed_drive_rot;

        public final Trigger interruptButton;
        public final Trigger gyroResetButton;

        public final Trigger elevatorUp;
        public final Trigger elevatorDown;
        public final Trigger scoreButton;
        public final Trigger intakeButton;

        public final DoubleSupplier reefX;
        public final DoubleSupplier reefY;

        public final Trigger nextReef;
        public final Trigger prevReef;

        public final IntSupplier reefStalk;

        public final Trigger elevatorNext;
        public final Trigger elevatorPrev;

        public OI() {
                driver = new CommandXboxController(Constants.OI.driverPort);
                operator = new CommandXboxController(Constants.OI.operatorPort);

                interruptButton = driver.start();
                gyroResetButton = driver.back();

                drive_x = () -> -driver.getLeftY();
                drive_y = () -> -driver.getLeftX();
                drive_rot = () -> -driver.getRightX();

                driveFast = driver.leftTrigger(Constants.OI.triggerDeadband);
                driveBoost = () -> driver.getLeftTriggerAxis();

                processed_drive_x = () -> Math.pow(MathUtil.applyDeadband(drive_x.getAsDouble(), Constants.OI.deadband),
                                3)
                                * Constants.Swerve.maxSpeed
                                * (driveFast.getAsBoolean()
                                                ? Constants.OI.slowSpeed
                                                                + MathUtil.applyDeadband(driveBoost.getAsDouble(),
                                                                                Constants.OI.triggerDeadband,
                                                                                1 - Constants.OI.slowSpeed)
                                                : Constants.OI.slowSpeed);
                processed_drive_y = () -> Math.pow(MathUtil.applyDeadband(drive_y.getAsDouble(), Constants.OI.deadband),
                                3)
                                * Constants.Swerve.maxSpeed
                                * (driveFast.getAsBoolean()
                                                ? Constants.OI.slowSpeed
                                                                + MathUtil.applyDeadband(driveBoost.getAsDouble(),
                                                                                Constants.OI.triggerDeadband,
                                                                                1 - Constants.OI.slowSpeed)
                                                : Constants.OI.slowSpeed);
                processed_drive_rot = () -> Math
                                .pow(MathUtil.applyDeadband(drive_rot.getAsDouble(), Constants.OI.deadband), 3)
                                * Constants.Swerve.maxAngularSpeed
                                * (driveFast.getAsBoolean()
                                                ? Constants.OI.slowSpeed
                                                                + MathUtil.applyDeadband(driveBoost.getAsDouble(),
                                                                                Constants.OI.triggerDeadband,
                                                                                1 - Constants.OI.slowSpeed)
                                                : Constants.OI.slowSpeed)
                                * 0.75;

                elevatorUp = driver.y();
                elevatorDown = driver.b();
                scoreButton = driver.a();
                intakeButton = driver.x();

                reefX = operator::getLeftX;
                reefY = operator::getLeftY;

                reefStalk = () -> {
                        double x = reefX.getAsDouble();
                        double y = reefY.getAsDouble();

                        if ((x > 0.1 || x < -0.1) && (y > 0.1 || y < -0.1)) {
                                double angle = Math.atan2(y, x);

                                return (int) Math.round(angle / (Math.PI * 2));
                        }

                        return 0;
                };

                nextReef = operator.povLeft().or(operator.x());
                prevReef = operator.povRight().or(operator.b());

                elevatorNext = operator.povUp().or(operator.y());
                elevatorPrev = operator.povDown().or(operator.a());
        }
}
