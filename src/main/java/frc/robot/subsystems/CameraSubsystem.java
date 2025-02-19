package frc.robot.subsystems;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.proto.Photon;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Unit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CameraSubsystem extends SubsystemBase {

    private PhotonCamera camera;
    private PhotonPoseEstimator photonPoseEstimator;
    private Pose3d estimatedPose = new Pose3d();
    private long poseTime = 0;
    private Command callback;

    public CameraSubsystem() {
        // Initialize the camera subsystem here

        // The field from AprilTagFields will be different depending on the game.
        AprilTagFieldLayout aprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

        // Forward Camera
        camera = new PhotonCamera("Arducam_1");

        Transform3d robotToCam = new Transform3d(new Translation3d(
                Units.inchesToMeters(-13.5), // 13.5 inches back from center
                0.0, // 0 inches left from center
                Units.inchesToMeters(5)), // 5 inches up from center
                new Rotation3d(0, Units.degreesToRadians(20), Math.PI)); // facing backward, 20 degrees up

        // Construct PhotonPoseEstimator
        photonPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout,
                PoseStrategy.CLOSEST_TO_REFERENCE_POSE, robotToCam);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run

        var images = camera.getAllUnreadResults();
        if (images.size() > 0) {
            var image = images.get(0);

            photonPoseEstimator.setReferencePose(estimatedPose);
            var optional = photonPoseEstimator.update(image);

            if (optional.isPresent()) {
                EstimatedRobotPose newEstimatedPose = optional.get();
                estimatedPose = newEstimatedPose.estimatedPose;
                poseTime = System.currentTimeMillis();
                callback.schedule();
            }
        }
    }

    public Pose3d getPose() {
        return estimatedPose;
    }

    // time of last calculated pose in milliseconds
    public long getPoseTime() {
        return poseTime;
    }

    public void setCallback(Command cb) {
        callback = cb; 
    }

}
