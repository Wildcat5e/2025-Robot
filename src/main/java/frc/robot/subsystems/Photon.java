// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Photon extends SubsystemBase {
  private static final AprilTagFieldLayout layout = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);
  private static final List<Pose2d> blueAprilTagPoses = new ArrayList<Pose2d>();
  private static final List<Pose2d> redAprilTagPoses = new ArrayList<Pose2d>();
  private static final Transform3d cameraToRobot = new Transform3d(new Pose3d(1, 1, 1, new Rotation3d(0, 0, 0)), new Pose3d());

  PhotonCamera camera = new PhotonCamera("a");

  /** Creates a new Photon. */
  public Photon() {
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
    var result = camera.getLatestResult();
    boolean hasTarget = result.hasTargets();

    if (hasTarget){
      PhotonTrackedTarget aprilTag = result.getBestTarget();
      Pose3d robotPose = PhotonUtils.estimateFieldToRobotAprilTag(aprilTag.getBestCameraToTarget(), layout.getTagPose(aprilTag.getFiducialId()).get(), cameraToRobot);
    }

  }
}
