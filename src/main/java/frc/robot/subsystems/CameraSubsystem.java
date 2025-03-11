package frc.robot.subsystems;

import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CameraSubsystem extends SubsystemBase {

    private PhotonCamera camera;
    private PhotonPoseEstimator photonPoseEstimator;
    private Pose3d estimatedPose = new Pose3d();
    private double poseTime = 0;
    private boolean changed = false;

    public CameraSubsystem(String cameraName, Transform3d cameraPos) {
        // Initialize the camera subsystem here

        // The field from AprilTagFields will be different depending on the game.
        AprilTagFieldLayout aprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);

        // Forward Camera
        camera = new PhotonCamera(cameraName);

        // Construct PhotonPoseEstimator
        photonPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout,
                PoseStrategy.CLOSEST_TO_REFERENCE_POSE, cameraPos);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run

        Pose3d pose = new Pose3d();

        // Get the latest image from the camera
        List<PhotonPipelineResult> images = camera.getAllUnreadResults();

        if (images.size() > 0) {
            PhotonPipelineResult image = images.get(0);

            photonPoseEstimator.setLastPose(estimatedPose);
            photonPoseEstimator.setReferencePose(estimatedPose);
            Optional<EstimatedRobotPose> optional = photonPoseEstimator.update(image);

            if (optional.isPresent()) {
                EstimatedRobotPose newEstimatedPose = optional.get();
                pose = newEstimatedPose.estimatedPose;
                poseTime = image.getTimestampSeconds();

                estimatedPose = pose;
                changed = true;
            }
        }
    }

    public Pose3d getPose() {
        return estimatedPose;
    }

    // time of last calculated pose in milliseconds
    public double getPoseTime() {
        return poseTime;
    }

    // returns true if the pose has changed, and resets the flag
    public boolean hasNewData() {
        if (changed) {
            changed = false;
            return true;
        }
        return false;
    }

}
