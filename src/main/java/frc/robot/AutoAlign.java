// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.ArrayList;
import java.util.List;

import com.pathplanner.lib.trajectory.PathPlannerTrajectoryState;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Extractor;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Photon;

import static frc.robot.subsystems.ListPose2d.*;
// might need to schedule this command !!
/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AutoAlign extends Command {
  Drivetrain drivetrain;

private static final double POSITION_TOLERANCE = 0.005;
  private static final double ROTATION_TOLERANCE = 0.02;
  private static final double ALGAE_POSITION_TOLERANCE = 0.03;
  private static final double ALGAE_ROTATION_TOLERANCE = 0.03;
    double MAX_DISTANCE = 1.5;
    Extractor extractor;
    Pose2d targetPose;
    boolean emergencyStop = false;
    long startTime;
    long endTime;
  boolean tooLong = false;
  boolean withinTolerance;
  Transform2d alignment;
  PathPlannerTrajectoryState goalState = new PathPlannerTrajectoryState();

    public List<Pose2d> TAG_POSE_LIST = List.of(POSE_1, POSE_2, POSE_3, POSE_4, POSE_5, POSE_6, POSE_7, POSE_8, POSE_9, POSE_10, POSE_11, POSE_12, POSE_13, POSE_14, POSE_15, POSE_16, POSE_17, POSE_18, POSE_19, POSE_20, POSE_21, POSE_22);

  public AutoAlign(Drivetrain drivetrain, Transform2d alignment) {
    this.alignment = alignment;
    this.drivetrain = drivetrain;
    addRequirements(drivetrain);


  }

  @Override
  public void initialize() {
    startTime = System.currentTimeMillis();
    Pose2d currentPose = drivetrain.getState().Pose;
    Pose2d nearestTagPose = currentPose.nearest(TAG_POSE_LIST);
    nearestTagPose = nearestTagPose.transformBy(alignment);
    double distance = currentPose.getTranslation().getDistance(nearestTagPose.getTranslation());
    goalState.pose = nearestTagPose;
    System.out.println("CLOSEST POSE ALERT !!!!!!!!!!!!!" + nearestTagPose);
    if (distance > MAX_DISTANCE){
      CommandScheduler.getInstance().cancel(this);
      System.out.println("TOO FAR DUMMY");
    }
  }

  @Override
  public void execute() {
    Pose2d currentPose = drivetrain.getState().Pose;
    ChassisSpeeds outputSpeeds = drivetrain.holonomicDriveController.calculateRobotRelativeSpeeds(currentPose, goalState);
    drivetrain.setControl(drivetrain.m_pathApplyRobotSpeeds
       .withSpeeds(outputSpeeds));
  }

  @Override
  public void end(boolean interrupted) {
    tooLong = false;
    emergencyStop = false;
  }

  @Override
  public boolean isFinished() {
    endTime = System.currentTimeMillis();
    if (endTime - startTime >= 1000){
      tooLong = true;
    }

    Pose2d currentPose = drivetrain.getState().Pose;
    double positionDistance = currentPose.getTranslation().getDistance(goalState.pose.getTranslation());
    double rotationDistance = Math.abs(currentPose.getRotation().minus(goalState.pose.getRotation()).getRadians());
    withinTolerance = (positionDistance < POSITION_TOLERANCE && rotationDistance < ROTATION_TOLERANCE);
    return (withinTolerance || emergencyStop || tooLong);
  }
}
