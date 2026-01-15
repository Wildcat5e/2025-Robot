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
      
        return new AutoAlign(drivetrain);
      }
}
