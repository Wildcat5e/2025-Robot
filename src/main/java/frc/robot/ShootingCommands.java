// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Outtake;

/** Add your docs here. */
public class ShootingCommands {
    Elevator elevator;
    Outtake outtake;
    public ShootingCommands(Elevator elevator, Outtake outtake){
        this.elevator = elevator;
        this.outtake = outtake;
    }

    public Command moveToLevelThreeShoot(){
        return new SequentialCommandGroup(
            elevator.moveToLevelThree(),
            outtake.shoot()
        );
    }

    public Command moveToLevelTwoShoot(){
        return new SequentialCommandGroup(
            elevator.moveToLevelTwo(),
            outtake.shoot()
        );
    }

    public Command moveToPositionZeroShoot(){
        return new SequentialCommandGroup(
            elevator.moveToPositionZero(),
            outtake.shoot()
        );
    }

}
