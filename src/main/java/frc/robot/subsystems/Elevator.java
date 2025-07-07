package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;

public interface Elevator {
    Command moveToPositionZero();

    Command moveToLevelTwo();

    Command moveToLevelThree();

    Command manualUp();

    Command manualDown();
}
