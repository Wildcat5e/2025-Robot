// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Extractor;
import frc.robot.subsystems.Outtake;
import frc.robot.AutoAlignCommands;

/** Add your docs here. */
public class ShootingCommands {
    Elevator elevator;
    Outtake outtake;
    AutoAlignCommands autoAlignCommands;
    Extractor extractor;

    public ShootingCommands(Elevator elevator, Outtake outtake, Extractor extractor,
            AutoAlignCommands autoAlignCommands) {
        this.elevator = elevator;
        this.outtake = outtake;
        this.autoAlignCommands = autoAlignCommands;
        this.extractor = extractor;
    }

    public Command moveToLevelThreeShoot() {
        return new SequentialCommandGroup(
                elevator.moveToLevelThree(),
                new WaitCommand(0.1),
                outtake.shoot());
    }

    public Command moveToLevelTwoShoot() {
        return new SequentialCommandGroup(
                elevator.moveToLevelTwo(),
                new WaitCommand(0.1),
                outtake.shoot());
    }

    public Command moveToPositionZeroShoot() {
        return new SequentialCommandGroup(
                elevator.moveToPositionZero(),
                new WaitCommand(0.1),
                outtake.shoot());
    }

    public Command algaeAlignOver() {
        return new SequentialCommandGroup(
        new ParallelCommandGroup(
            autoAlignCommands.alignArmToAlgae(),
            elevator.moveToLevelTwo()
        ),
        extractor.moveArmOverAlgae());
    }

    public Command algaeExtractOver() {
        return new SequentialCommandGroup(
            autoAlignCommands.driveToAlgae(),
            new ParallelCommandGroup(
                extractor.removeAlgaeDown(),
                autoAlignCommands.alignArmToAlgae()
            )
        );
    }

    public Command algaeAlignUnder() {
        return new SequentialCommandGroup(
        new ParallelCommandGroup(
            autoAlignCommands.alignArmToAlgae(),
            elevator.moveToLevelTwo()
        ),
        extractor.moveArmUnderAlgae());
    }

    public Command algaeExtractUnder() {
        return new SequentialCommandGroup(
            autoAlignCommands.driveToAlgae(),
            new ParallelCommandGroup(
                extractor.removeAlgaeUp(),
                autoAlignCommands.alignArmToAlgae()
            )
        );
    }


    public Command safetyStopExtractorPID() {
        return extractor.runEnd(() -> {
            autoAlignCommands.emergencyStop = true;
            extractor.stop();
        }, () -> {
            autoAlignCommands.emergencyStop = false;
        });
    }

}
