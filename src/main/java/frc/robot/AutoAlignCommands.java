// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;
import java.util.Set;

import com.pathplanner.lib.trajectory.PathPlannerTrajectoryState;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Extractor;
import frc.robot.subsystems.Photon;

import static frc.robot.subsystems.ListPose2d.*;
/** Add your docs here. */
public class AutoAlignCommands {

private static final double POSITION_TOLERANCE = 0.02;
  private static final double ROTATION_TOLERANCE = 0.02;
  private static final double ALGAE_POSITION_TOLERANCE = 0.03;
  private static final double ALGAE_ROTATION_TOLERANCE = 0.03;

  private static final Transform2d CENTER_ALGAE_ARM = new Transform2d(
    new Translation2d(1, .25), 
    Rotation2d.fromDegrees(180));

  private static final Transform2d EXTRACT_ALGAE_ARM = new Transform2d(
    new Translation2d(0.375, .2), 
    Rotation2d.fromDegrees(180));
    
  private static final Transform2d LEFT_ALIGN_DISTANCE = new Transform2d(
      new Translation2d(0.375, -0.175),
      Rotation2d.fromDegrees(180));

  private static final Transform2d RIGHT_ALIGN_DISTANCE = new Transform2d(
      new Translation2d(0.375, 0.175),
      Rotation2d.fromDegrees(180));

  // public List<Pose2d> TAG_POSE_LIST = List.of(POSE_1, POSE_2, POSE_3, POSE_4, POSE_5, POSE_6, POSE_7, POSE_8, POSE_9, POSE_10, POSE_11, POSE_12, POSE_13, POSE_14, POSE_15, POSE_16, POSE_17, POSE_18, POSE_19, POSE_20, POSE_21, POSE_22);


    double MIN_DISTANCE = 1.5;
    Drivetrain drivetrain;
    Extractor extractor;
    Pose2d targetPose;
    boolean emergencyStop = false;
    long startTime;
    long endTime;
  boolean tooLong = false;
  boolean withinTolerance;

    public AutoAlignCommands(Drivetrain drivetrain, Extractor extractor, Photon photon){
        this.drivetrain = drivetrain;
        this.extractor = extractor;
    }

    public Command leftAutoAlign() {
      
        return new AutoAlign(drivetrain, LEFT_ALIGN_DISTANCE);
      }

      // public Command rightAutoAlign() {
      //   return Commands.defer(() -> {
        
      //   //robotpose and currentpose are same in this command
      //   Pose2d robotPose = drivetrain.getState().Pose;
      //   Pose2d nearestTagPose = robotPose.nearest(TAG_POSE_LIST);
      //   // System.out.println("closest tag pose (before transform): "+ nearestTagPose);
      //   nearestTagPose = nearestTagPose.transformBy(RIGHT_ALIGN_DISTANCE);
      //   // System.out.println("closest tag pose (after transform): "+ nearestTagPose);
      //   double distance = robotPose.getTranslation().getDistance(nearestTagPose.getTranslation());
      //   // System.out.println("robot pose: " + robotPose);
      //   // System.out.println("distance:" + distance + "min distancne: " + MIN_DISTANCE);
      //   PathPlannerTrajectoryState goalState = new PathPlannerTrajectoryState();
      //   goalState.pose = nearestTagPose;
      

      //   if (distance < MIN_DISTANCE){
      //     return new FunctionalCommand(
      //       () -> {
      //         startTime = System.currentTimeMillis();
      //       },
      //       () -> {
      
      //         Pose2d currentPose = drivetrain.getState().Pose;
      //         ChassisSpeeds outputSpeeds = drivetrain.holonomicDriveController.calculateRobotRelativeSpeeds(currentPose, goalState);

      //         drivetrain.setControl(drivetrain.m_pathApplyRobotSpeeds
      //             .withSpeeds(outputSpeeds));

      //           // System.out.println(outputSpeeds);
      //       },
      //       (interrupted) -> {
      //         tooLong = false;
      //         emergencyStop = false;

      //         // System.out.println("POSE REACHED");
      //         //   System.out.println("ROBOT POSE: " + drivetrain.getState().Pose);
      //         //   System.out.println("TARGET POSE: " + goalState.pose);
      //       },
      //       () -> {
      //         endTime = System.currentTimeMillis();
      //         if (endTime - startTime >= 1000){
      //           tooLong = true;
      //         }

      //         Pose2d currentPose = drivetrain.getState().Pose;
      //         double positionDistance = currentPose.getTranslation().getDistance(goalState.pose.getTranslation());
      //         double rotationDistance = Math.abs(currentPose.getRotation().minus(goalState.pose.getRotation()).getRadians());
      //         // System.out.println("position distance: " + positionDistance + " rotation distance:" + rotationDistance);
      //         withinTolerance = (positionDistance < POSITION_TOLERANCE && rotationDistance < ROTATION_TOLERANCE);
      //         return (withinTolerance || emergencyStop || tooLong);
      //       },
      //       drivetrain);
      //   } else {
      //     return Commands.none();
      //   }
      
      
      // }
      // , Set.of(drivetrain));
      // }


  // public Command alignArmToAlgae() {
  //   return Commands.defer(() -> {
    
  //   //robotpose and currentpose are same in this command
  //   Pose2d robotPose = drivetrain.getState().Pose;
  //   Pose2d nearestTagPose = robotPose.nearest(limelight.aprilTagPoses);
  //   // System.out.println("closest tag pose (before transform): "+ nearestTagPose);
  //   nearestTagPose = nearestTagPose.transformBy(CENTER_ALGAE_ARM);
  //   // System.out.println("closest tag pose (after transform): "+ nearestTagPose);
  //   double distance = robotPose.getTranslation().getDistance(nearestTagPose.getTranslation());
  //   // System.out.println("robot pose: " + robotPose);
  //   // System.out.println("distance:" + distance + "min distancne: " + MIN_DISTANCE);
  //   PathPlannerTrajectoryState goalState = new PathPlannerTrajectoryState();
  //   goalState.pose = nearestTagPose;
  
  
  //   if (distance < MIN_DISTANCE && limelight.calibrate()){
  //     return new FunctionalCommand(
  //       () -> {
  //         startTime = System.currentTimeMillis();
  //       },
  //       () -> {
  
  //         Pose2d currentPose = drivetrain.getState().Pose;
  
  //         drivetrain.setControl(drivetrain.m_pathApplyRobotSpeeds
  //             .withSpeeds(drivetrain.algaeDriveController.calculateRobotRelativeSpeeds(currentPose, goalState)));
  //       },
  //       (interrupted) -> {
  //         tooLong = false;
  //         emergencyStop = false;

  //         // System.out.println("ALIGN ARM END");

  //       },
  //       () -> {
  //         endTime = System.currentTimeMillis();
  //         if (endTime - startTime >= 1500){
  //           tooLong = true;
  //         }

  //         Pose2d currentPose = drivetrain.getState().Pose;
  //         double positionDistance = currentPose.getTranslation().getDistance(goalState.pose.getTranslation());
  //         double rotationDistance = Math.abs(currentPose.getRotation().minus(goalState.pose.getRotation()).getRadians());
  //         withinTolerance = positionDistance < ALGAE_POSITION_TOLERANCE && rotationDistance < ALGAE_ROTATION_TOLERANCE;
  //         return (withinTolerance || emergencyStop || tooLong);
  //       },
  //       drivetrain);
  //   } else {
  //     return Commands.none();
  //   }
  
  
  // }
  // , Set.of(drivetrain));
  // }

  // public Command driveToAlgae() {
  //   return Commands.defer(() -> {
    
  //   //robotpose and currentpose are same in this command
  //   Pose2d robotPose = drivetrain.getState().Pose;
  //   Pose2d nearestTagPose = robotPose.nearest();
  //   nearestTagPose = nearestTagPose.transformBy(EXTRACT_ALGAE_ARM);
  //   double distance = robotPose.getTranslation().getDistance(nearestTagPose.getTranslation());
  //   PathPlannerTrajectoryState goalState = new PathPlannerTrajectoryState();
  //   goalState.pose = nearestTagPose;
  

  //   if (distance < MIN_DISTANCE && limelight.calibrate()){
  //     return new FunctionalCommand(
  //       () -> {
  //         startTime = System.currentTimeMillis();
  //       },
  //       () -> {
  
  //         Pose2d currentPose = drivetrain.getState().Pose;
  
  //         drivetrain.setControl(drivetrain.m_pathApplyRobotSpeeds
  //             .withSpeeds(drivetrain.algaeDriveController.calculateRobotRelativeSpeeds(currentPose, goalState)));
  //       },
  //       (interrupted) -> {
  //         tooLong = false;
  //         emergencyStop = false;

  //       },
  //       () -> {
  //         endTime = System.currentTimeMillis();
  //         if (endTime - startTime >= 1500){
  //           tooLong = true;
  //         }

  //         Pose2d currentPose = drivetrain.getState().Pose;
  //         double positionDistance = currentPose.getTranslation().getDistance(goalState.pose.getTranslation());
  //         double rotationDistance = Math.abs(currentPose.getRotation().minus(goalState.pose.getRotation()).getRadians());
  //         withinTolerance = positionDistance < ALGAE_POSITION_TOLERANCE && rotationDistance < ALGAE_ROTATION_TOLERANCE;
  //         return (withinTolerance || emergencyStop || tooLong);
  //       },
  //       drivetrain);
  //   } else {
  //     return Commands.none();
  //   }
  
  
  // }
  // , Set.of(drivetrain));
  // }

  // public Command printDistances() {
  //   return Commands.defer(() -> {
  //     Pose2d currentPose = drivetrain.getState().Pose;

  //     ChassisSpeeds speeds = drivetrain.getState().Speeds;

  //     Pose2d targetPose = null;
  //     int counter = 1;

  //     for (Pose2d pose : limelight.aprilTagPoses) {
  //       double distance = currentPose.getTranslation().getDistance(pose.getTranslation());
  //       // System.out.println("AprilTag counter: " + counter + " Tag Pose: " + pose + "Robot Pose: " + currentPose
  //       //     + "Distance: " + distance);
  //       if (distance < MIN_DISTANCE) {
  //         // System.out.println("AprilTag counter: " + counter + " is close enough!!");
  //       }
  //       counter++;
  //     }

  //     return Commands.none();
  //   }, Set.of());
  // }

}
