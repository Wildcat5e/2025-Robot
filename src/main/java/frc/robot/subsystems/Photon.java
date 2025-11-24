package frc.robot.subsystems;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonTrackedTarget;

import java.util.List;
import java.util.Optional;

import static frc.robot.subsystems.AprilTag2025.*;

public class Photon extends SubsystemBase {

    private static final PhotonCamera CAMERAL = new PhotonCamera("GENERAL_WEBCAM");

    private static final List<AprilTag> APRIL_TAGS = List.of(
            TAG_1, TAG_2, TAG_3, TAG_4, TAG_5, TAG_6, TAG_7, TAG_8, TAG_9, TAG_10, TAG_11,
            TAG_12, TAG_13, TAG_14, TAG_15, TAG_16, TAG_17, TAG_18, TAG_19, TAG_20, TAG_21, TAG_22);

    private static final double FIELD_WIDTH = 8.052;
    private static final double FIELD_LENGTH = 17.548;

    private static final AprilTagFieldLayout LAYOUT = new AprilTagFieldLayout(APRIL_TAGS, FIELD_LENGTH, FIELD_WIDTH);

    // Adjust this to reflect the camera position on the robot
    private static final Transform3d ROBOT_TO_CAMERA = new Transform3d(0, 0, 0, new Rotation3d(0, 0, 0));

    private static final PhotonPoseEstimator ESTIMATOR =
            new PhotonPoseEstimator(LAYOUT, PoseStrategy.LOWEST_AMBIGUITY, ROBOT_TO_CAMERA);

    private final Drivetrain drivetrain;


    public Photon(Drivetrain drivetrain) {
        this.drivetrain = drivetrain;
    }

    @Override
    public void periodic() {
        Matrix<N3, N1> stddev = VecBuilder.fill(0.5, 0.5, 1);

        for (var change : CAMERAL.getAllUnreadResults()) {
            var optionalVisionEst = ESTIMATOR.update(change);
            if (optionalVisionEst.isEmpty()) {
                continue;
            }
            var visionEst = optionalVisionEst.get();
            var estimatedPose2d = visionEst.estimatedPose.toPose2d();
            stddev = stddev.plus(standardDeviationOfDistance(estimatedPose2d, change.getTargets()));
            drivetrain.addVisionMeasurement(estimatedPose2d, visionEst.timestampSeconds, stddev);
        }
    }

    // Calculate the standard deviation, how much on average the visual of the april
    // tag deviates in meters from its actual location in real life
    private double standardDeviationOfDistance(Pose2d estimatedPose, List<PhotonTrackedTarget> targets) {
        // For each photon tracked target, grab the april tag's pose, and find the distance from the estimated robot
        // pose and april tag to calculate and estimate a standard deviation
        double sumOfDistanceToAprilTag = 0;
        int numberOfAprilTagsFound = 0;
        for (var target : targets) {
            Optional<Pose3d> pose3d = LAYOUT.getTagPose(target.getFiducialId());
            if (pose3d.isPresent()) {
                ++numberOfAprilTagsFound;
                Pose2d tagPose = pose3d.get().toPose2d();
                sumOfDistanceToAprilTag += tagPose.getTranslation().getDistance(estimatedPose.getTranslation());
            }
        }
        double averageAprilTagDistance = sumOfDistanceToAprilTag / numberOfAprilTagsFound;

        // If there is only one pose, square the standard deviation to account for higher uncertainty
        if (numberOfAprilTagsFound == 1) {
            return averageAprilTagDistance * averageAprilTagDistance;
        }
        return averageAprilTagDistance;
    }
}

