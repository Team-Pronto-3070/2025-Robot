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
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CameraSubsystem extends SubsystemBase {

    private PhotonCamera cameraF;
    private PhotonCamera cameraB;
    private PhotonPoseEstimator photonPoseEstimatorF;
    private PhotonPoseEstimator photonPoseEstimatorB;
    private Pose3d estimatedPose = new Pose3d();
    private double poseTime = 0;

    public CameraSubsystem() {
        // Initialize the camera subsystem here

        // The field from AprilTagFields will be different depending on the game.
        AprilTagFieldLayout aprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

        // Forward Camera
        cameraF = new PhotonCamera("Arducam_Front");
        cameraB = new PhotonCamera("Arducam_Rear");

        Transform3d robotToCamF = new Transform3d(
                new Translation3d(
                        Units.inchesToMeters(13.5), // 13.5 inches forwards from center
                        Units.inchesToMeters(3), // 3 inches left from center
                        Units.inchesToMeters(6.5)), // 6.5 inches up from center
                new Rotation3d(
                        Units.degreesToRadians(0), // forwards
                        Units.degreesToRadians(20), // 20 degrees upwards
                        Units.degreesToRadians(0))); // facing backwards

        Transform3d robotToCamB = new Transform3d(
                new Translation3d(
                        Units.inchesToMeters(-13.5), // 13.5 inches back from center
                        Units.inchesToMeters(0), // 0 inches left from center
                        Units.inchesToMeters(5)), // 5 inches up from center
                new Rotation3d(
                        Units.degreesToRadians(0), // forwards
                        Units.degreesToRadians(20), // 20 degrees upwards
                        Units.degreesToRadians(180))); // facing backwards

        // Construct PhotonPoseEstimator
        photonPoseEstimatorF = new PhotonPoseEstimator(aprilTagFieldLayout,
                PoseStrategy.CLOSEST_TO_REFERENCE_POSE, robotToCamF);
        photonPoseEstimatorB = new PhotonPoseEstimator(aprilTagFieldLayout,
                PoseStrategy.CLOSEST_TO_REFERENCE_POSE, robotToCamB);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run

        boolean changed = false;
        double imageTime = 0;

        Pose3d fPose = new Pose3d();
        Pose3d bPose = new Pose3d();

        // Get the latest image from the camera
        List<PhotonPipelineResult> imagesF = cameraF.getAllUnreadResults();
        List<PhotonPipelineResult> imagesB = cameraB.getAllUnreadResults();

        if (imagesF.size() > 0) {
            PhotonPipelineResult image = imagesF.get(0);

            photonPoseEstimatorF.setReferencePose(estimatedPose);
            Optional<EstimatedRobotPose> optional = photonPoseEstimatorF.update(image);

            if (optional.isPresent()) {
                EstimatedRobotPose newEstimatedPose = optional.get();
                fPose = newEstimatedPose.estimatedPose;
                imageTime = image.getTimestampSeconds();
                changed = true;
            }
        }

        if (imagesB.size() > 0) {
            PhotonPipelineResult image = imagesB.get(0);

            photonPoseEstimatorB.setReferencePose(estimatedPose);
            Optional<EstimatedRobotPose> optional = photonPoseEstimatorB.update(image);

            if (optional.isPresent()) {
                EstimatedRobotPose newEstimatedPose = optional.get();
                bPose = newEstimatedPose.estimatedPose;
                if (changed) {
                    imageTime = (imageTime + image.getTimestampSeconds()) / 2;
                } else {
                    changed = true;
                    imageTime = image.getTimestampSeconds();
                }
            }
        }

        if (changed) {
            estimatedPose = fPose;
            poseTime = imageTime;
        }
    }

    public Pose3d getPose() {
        return estimatedPose;
    }

    // time of last calculated pose in milliseconds
    public double getPoseTime() {
        return poseTime;
    }

}
