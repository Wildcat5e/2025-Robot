// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonTrackedTarget;

import java.util.List;
import java.util.Optional;
import java.util.stream.Collectors;

import static frc.robot.subsystems.ListAprilTag2025.*;

public class Photon extends SubsystemBase {

    private static final double FIELD_WIDTH = 8.052;
    private static final double FIELD_LENGTH = 17.548;
    private static final Transform3d CAMERA_TO_ROBOT = new Transform3d(-0.114, 0, 0, new Rotation3d(0, 0, 0));
    private static final PhotonCamera CAMERAL = new PhotonCamera("C922_Pro_Stream_Webcam");

    private final Drivetrain drivetrain;

    private AprilTagFieldLayout layout;
    private PhotonPoseEstimator estimator;

    private int counter = 0;

    public List<AprilTag> TAG_LIST = List.of(TAG_25, TAG_26);


    public Photon(Drivetrain drivetrain) {
        this.drivetrain = drivetrain;
    }

    @Override
    public void periodic() {
        counter++;


        if (!initializeLayoutAndEstimator()) {
            return;
        }

        Matrix<N3, N1> stddev = VecBuilder.fill(10, 10, 10);

        for (var change : CAMERAL.getAllUnreadResults()) {
            var optionalVisionEst = estimator.update(change);
            if (optionalVisionEst.isEmpty()) {
                continue;
            }
            var visionEst = optionalVisionEst.get();
            Pose2d estimatedPose2d = visionEst.estimatedPose.toPose2d();
            // System.out.println(estimatedPose2d);
            // stddev = stddev.plus(averageDistanceOfTag(estimatedPose2d, change.getTargets()));
            drivetrain.addVisionMeasurement(estimatedPose2d, visionEst.timestampSeconds, stddev);
            if (counter % 100 == 0){
                System.out.println("estimated pose: " + estimatedPose2d);
                System.out.println("actual robot pose: " + drivetrain.getState().Pose);
            }
        }
    }


    // Calculate the standard deviation, how much on average the visual of the april
    // tag deviates in meters from its actual location in real life
    private double averageDistanceOfTag(Pose2d estimatedPose, List<PhotonTrackedTarget> targets) {
        // For each photon tracked target, grab the april tag's pose, and find the distance from the estimated robot
        // pose and april tag to calculate and estimate a standard deviation
        double distanceSum = 0;
        int numOfTags = 0;
        for (var target : targets) {
            Optional<Pose3d> pose3d = layout.getTagPose(target.getFiducialId());
            if (pose3d.isPresent()) {
                ++numOfTags;
                Pose2d tagPose = pose3d.get().toPose2d();
                double distance = tagPose.getTranslation().getDistance(estimatedPose.getTranslation());
                distanceSum += distance;
            }
        }

        double avgDistance = distanceSum / numOfTags;

        if (numOfTags == 1) {
            return avgDistance * avgDistance;
        }
        return avgDistance;
    }

    private boolean initializeLayoutAndEstimator() {
        if (layout != null)
            // already initialized
            return true;
        var alliance = DriverStation.getAlliance().orElse(null);
        if (alliance == null)
            // not yet determined
            return false;
        layout = new AprilTagFieldLayout(TAG_LIST, FIELD_LENGTH, FIELD_WIDTH);
        estimator = new PhotonPoseEstimator(layout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, CAMERA_TO_ROBOT);
        return true;
    }

}
