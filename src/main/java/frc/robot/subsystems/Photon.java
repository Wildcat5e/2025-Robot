// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Photon extends SubsystemBase {
    private static final AprilTagFieldLayout LAYOUT = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
    private static final Transform3d CAMERA_TO_ROBOT = new Transform3d(0, 0, 0, new Rotation3d(0, 0, 0));

    private static final PhotonPoseEstimator PHOTON_ESTIMATOR = new PhotonPoseEstimator(
            LAYOUT,
            PoseStrategy.LOWEST_AMBIGUITY,
            CAMERA_TO_ROBOT);

    // create camera object for phton camera
    PhotonCamera camera = new PhotonCamera("GENERAL_WEBCAM");
    Optional<EstimatedRobotPose> visionEst = Optional.empty();

    Drivetrain drivetrain;
    double distanceSum;
    double numOfTags;
    Runtime runtime = Runtime.getRuntime();

    /** Creates a new Photon. */
    public Photon(Drivetrain drivetrain) {

        this.drivetrain = drivetrain;

    }

    int counter = 0;

    @Override
    public void periodic() {
        if (counter % 50 != 0) { // only run every once a second
            return;
        }
        counter += 1;
        Matrix<N3, N1> stddev = VecBuilder.fill(0.5, 0.5, 1);

        // for every pipeline in the list of unread pipeline results
        for (var change : camera.getAllUnreadResults()) {

            // update visionEst using photon estimator, visionEst contains info about
            // estimated pose and timestamp, visionEst is Optional, meaning it can be empty
            // Update visionEst using photon estimator
            // If the visionEst object has an estimated pose, update the drivetrain pose
            // and calculate the stddev
            var optionalVisionEst = PHOTON_ESTIMATOR.update(change);
            if (optionalVisionEst.isEmpty()) {
                continue;
            }
            var visionEst = optionalVisionEst.get();
            Pose2d estimatedPose = visionEst.estimatedPose.toPose2d();
            double distance = averageDistanceOfTag(estimatedPose, change.getTargets());
            stddev = stddev.plus(distance);
            drivetrain.addVisionMeasurement(estimatedPose, visionEst.timestampSeconds, stddev);
        }
    }

    // Calculate the standard deviation, how much on average the visual of the april
    // tag deviates in meters
    // from its actual location in real life

    private double averageDistanceOfTag(Pose2d estimatedPose, List<PhotonTrackedTarget> targets) {
        int numOfTags = 0;
        double distanceSum = 0;
        double avgDistance = 0;

        // For each photon tracked target, grab the april tag's pose, and find the
        // distance from the
        // estimated robot pose and april tag to calculate and estimate a standard
        // deviation
        for (var target : targets) {
            Pose2d tagPose = LAYOUT.getTagPose(target.getFiducialId()).get().toPose2d();
            double distance = tagPose.getTranslation().getDistance(estimatedPose.getTranslation());
            numOfTags++;
            distanceSum += distance;
        }

        avgDistance = distanceSum / numOfTags;
        

        if (numOfTags == 1) {
            return avgDistance * avgDistance;
        }
        return avgDistance;
    }

}
