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
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;

public class Photon extends SubsystemBase {
  private static final AprilTagFieldLayout layout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
  private static final List<Pose2d> blueAprilTagPoses = new ArrayList<Pose2d>();
  private static final List<Pose2d> redAprilTagPoses = new ArrayList<Pose2d>();
  private static final Transform3d cameraToRobot = new Transform3d(0, 0, 0, new Rotation3d(0, 0, 0));

  PhotonPoseEstimator photonEstimator = new PhotonPoseEstimator(layout, PoseStrategy.LOWEST_AMBIGUITY,
      cameraToRobot);
  // create camera object for phton camera
  PhotonCamera camera = new PhotonCamera("GENERAL_WEBCAM");
  Matrix<N3, N1> stddev = VecBuilder.fill(0.5, 0.5, 1);
  Optional<EstimatedRobotPose> visionEst = Optional.empty();

  Drivetrain drivetrain;
  double distanceSum;
  double numOfTags;
  Runtime runtime = Runtime.getRuntime();

  /** Creates a new Photon. */
  public Photon(Drivetrain drivetrain) {

    this.drivetrain = drivetrain;

    blueAprilTagPoses.add(layout.getTagPose(17).get().toPose2d());
    blueAprilTagPoses.add(layout.getTagPose(18).get().toPose2d());
    blueAprilTagPoses.add(layout.getTagPose(19).get().toPose2d());
    blueAprilTagPoses.add(layout.getTagPose(20).get().toPose2d());
    blueAprilTagPoses.add(layout.getTagPose(21).get().toPose2d());
    blueAprilTagPoses.add(layout.getTagPose(22).get().toPose2d());

    redAprilTagPoses.add(layout.getTagPose(6).get().toPose2d());
    redAprilTagPoses.add(layout.getTagPose(7).get().toPose2d());
    redAprilTagPoses.add(layout.getTagPose(8).get().toPose2d());
    redAprilTagPoses.add(layout.getTagPose(9).get().toPose2d());
    redAprilTagPoses.add(layout.getTagPose(10).get().toPose2d());
    redAprilTagPoses.add(layout.getTagPose(11).get().toPose2d());
  }

  @Override
  public void periodic() {

    

    //for every pipeline in the list of unread pipeline results
    for (var change : camera.getAllUnreadResults()) {

      //update visionEst using photon estimator, visionEst contains info about
      //estimated pose and timestamp, visionEst is Optional, meaning it can be empty
      visionEst = photonEstimator.update(change);
      System.out.println(change.getTimestampSeconds());
      //if the visionEst object has an estimated pose, update the drivetrain pose
      //and calculate the stddev
      if (!visionEst.isEmpty()) {
        System.out.println("TAG DETECTED");
        Pose2d estimatedPose = visionEst.get().estimatedPose.toPose2d();
        double timestamp = visionEst.get().timestampSeconds;
        updateEstimationStdDevs(estimatedPose, change.getTargets());
        drivetrain.addVisionMeasurement(estimatedPose, timestamp, stddev);
        System.out.println("estimated pose: " + estimatedPose);
      }
    }
  }


  // Calculate the standard deviation, how much on average the visual of the april tag deviates in meters
  // from its actual location in real life

  private void updateEstimationStdDevs(Pose2d estimatedPose, List<PhotonTrackedTarget> targets) {
      int numOfTags = 0;
      double distanceSum = 0;
      double avgDistance = 0;

    // For each photon tracked target, grab the april tag's pose, and find the distance from the
    // estimated robot pose and april tag to calculate and estimate a standard deviation
      for (var target : targets){
        Pose2d tagPose = layout.getTagPose(target.getFiducialId()).get().toPose2d();
        double distance = tagPose.getTranslation().getDistance(estimatedPose.getTranslation());
        numOfTags++;
        distanceSum += distance;
      }
    
      avgDistance = distanceSum/numOfTags;

      if (numOfTags == 1){
        stddev = stddev.plus(avgDistance * avgDistance);
      } else {
        stddev = stddev.plus(avgDistance);
      }

  }

}
