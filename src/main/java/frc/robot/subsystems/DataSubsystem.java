package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DataSubsystem extends SubsystemBase {

    private final PowerDistribution pdh;
    private final Field2d field;

    public DataSubsystem() {
        // Initialize the data subsystem here
        pdh = new PowerDistribution();
        field = new Field2d();
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }

    public void update(Pose2d position) {
        SmartDashboard.putNumber("Voltage", pdh.getVoltage());
        field.setRobotPose(position);
        SmartDashboard.putData("Field", field);
        SmartDashboard.putNumber("Match Timer", Timer.getMatchTime());
    }

}
