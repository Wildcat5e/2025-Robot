// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot;

// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.math.geometry.Transform2d;
// import edu.wpi.first.math.geometry.Translation2d;
// import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
// import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
// import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
// import edu.wpi.first.wpilibj2.command.WaitCommand;
// import frc.robot.subsystems.Drivetrain;
// import frc.robot.subsystems.Elevator;
// import frc.robot.subsystems.Extractor;
// import frc.robot.subsystems.Limelight;
// import frc.robot.subsystems.Outtake;

// /** Add your docs here. */
// public class ShootingCommands {
//     Drivetrain drivetrain;
//     Elevator elevator;
//     Outtake outtake;
//     Extractor extractor;
//     Limelight limelight;

//   private static final Transform2d CENTER_ALGAE_ARM = new Transform2d(
//     new Translation2d(1, .25), 
//     Rotation2d.fromDegrees(180));

//   private static final Transform2d EXTRACT_ALGAE_ARM = new Transform2d(
//     new Translation2d(0.375, .2), 
//     Rotation2d.fromDegrees(180));
    

//   private static final Transform2d LEFT_ALIGN_DISTANCE = new Transform2d(
//       new Translation2d(0.375, -0.175),
//       Rotation2d.fromDegrees(180));

//   private static final Transform2d RIGHT_ALIGN_DISTANCE = new Transform2d(
//       new Translation2d(0.375, 0.175),
//       Rotation2d.fromDegrees(180));


//     public ShootingCommands(Drivetrain drivetrain, Limelight limelight, Elevator elevator, Outtake outtake, Extractor extractor) {
//         this.elevator = elevator;
//         this.outtake = outtake;
//         this.extractor = extractor;
//     }

//     public Command leftAutoAlign (){
//         return new AutoAlign(drivetrain, limelight, LEFT_ALIGN_DISTANCE);
//     }

//     public Command rightAutoAlign(){
//         return new AutoAlign(drivetrain, limelight, RIGHT_ALIGN_DISTANCE);
//     }

//     public Command algaeAlignOver() {
//         return new SequentialCommandGroup(
//         new ParallelCommandGroup(
//             new AutoAlign(null, null, null),
//             elevator.moveToLevelTwo()
//         ),
//         extractor.moveArmOverAlgae());
//     }

//     public Command algaeExtractOver() {
//         return new SequentialCommandGroup(
//             new AutoAlign(drivetrain, limelight, EXTRACT_ALGAE_ARM),
//             new ParallelCommandGroup(
//                 extractor.removeAlgaeDown(),
//                 new SequentialCommandGroup(
//                     new WaitCommand(0.5),
//                     new AutoAlign(drivetrain, limelight, CENTER_ALGAE_ARM))
//             )
//         );
//     }

//     public Command algaeAlignUnder() {
//         return new SequentialCommandGroup(
//         new ParallelCommandGroup(
//             new AutoAlign(drivetrain, limelight, CENTER_ALGAE_ARM),
//             elevator.moveToLevelTwo()
//         ),
//         extractor.moveArmUnderAlgae());
//     }


//     public Command algaeExtractUnder() {
//         return new SequentialCommandGroup(
//             new AutoAlign(drivetrain, limelight, EXTRACT_ALGAE_ARM),
//             new ParallelCommandGroup(
//                 extractor.removeAlgaeUp(),
//                 new SequentialCommandGroup(
//                     new WaitCommand(0.5),
//                     new AutoAlign(drivetrain, limelight, CENTER_ALGAE_ARM))
//             )
//         );
//     }


//     // public Command safetyStopExtractorPID() {
//     //     return extractor.runEnd(() -> {
//     //         autoAlignCommands.emergencyStop = true;
//     //         extractor.stop();
//     //     }, () -> {
//     //         autoAlignCommands.emergencyStop = false;
//     //     });
//     // }

// }
