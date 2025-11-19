// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Quaternion;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Photon extends SubsystemBase {
    // LINK OF ALL THE APRIL TAG POSES FROM JSON FILE
    // https://github.com/wpilibsuite/allwpilib/blob/main/apriltag/src/main/native/resources/edu/wpi/first/apriltag/2025-reefscape-welded.json
    private static final double FIELD_WIDTH = 8.052;
    private static final double FIELD_LENGTH = 17.548;
    AprilTagFieldLayout betterLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeAndyMark);
    private static final Transform3d CAMERA_TO_ROBOT = new Transform3d(0, 0, 0, new Rotation3d(0, 0, 0));

    private PhotonPoseEstimator photonEstimator;

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
        if (!initializeFieldLayout()) {
            return;
        }

        // if (++counter % 5 != 0) { // only run every once a second
        // return;
        // }

        Matrix<N3, N1> stddev = VecBuilder.fill(0.5, 0.5, 1);

        // for every pipeline in the list of unread pipeline results
        for (var change : camera.getAllUnreadResults()) {

            var optionalVisionEst = photonEstimator.update(change);
            if (optionalVisionEst.isEmpty()) {
                continue;
            }
            var visionEst = optionalVisionEst.get();
            Pose2d estimatedPose = visionEst.estimatedPose.toPose2d();
            double distance = averageDistanceOfTag(estimatedPose, change.getTargets());
            stddev = stddev.plus(distance);
            drivetrain.addVisionMeasurement(estimatedPose, visionEst.timestampSeconds, stddev);
            System.out.println("estimated pose: " + estimatedPose);
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
            Pose2d tagPose = betterLayout.getTagPose(target.getFiducialId()).get().toPose2d();
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

    private boolean initializeFieldLayout() {
        // check if alliance is present even without connection to fms
        if (betterLayout == null && DriverStation.getAlliance().isPresent()) {
            var aprilTagList = new ArrayList<AprilTag>();

            if (DriverStation.getAlliance().get() == DriverStation.Alliance.Blue) {
                // all april tags needed for blue side
                aprilTagList.add(
                        new AprilTag(20, new Pose3d(4.90474, 4.745482, 0.308102,
                                new Rotation3d(new Quaternion(0.8660254037844387, 0.0, 0.0, 0.5)))));
                aprilTagList.add(
                        new AprilTag(19, new Pose3d(4.90474, 4.745482, 0.308102,
                                new Rotation3d(new Quaternion(0.8660254037844387, 0.0, 0.0, 0.5)))));
            } else {
                // all april tags needed for red side
                aprilTagList.add(new AprilTag(20, new Pose3d(4.90474, 4.745482, 0.308102,
                        new Rotation3d(new Quaternion(0.8660254037844387, 0.0, 0.0, 0.5)))));
                aprilTagList.add(new AprilTag(19, new Pose3d(4.90474, 4.745482, 0.308102,
                        new Rotation3d(new Quaternion(0.8660254037844387, 0.0, 0.0, 0.5)))));
            }

            betterLayout = new AprilTagFieldLayout(aprilTagList, FIELD_LENGTH, FIELD_WIDTH);
            photonEstimator = new PhotonPoseEstimator(
                    betterLayout,
                    PoseStrategy.LOWEST_AMBIGUITY,
                    CAMERA_TO_ROBOT);
            return true;
        }
        return false;
    }

}
