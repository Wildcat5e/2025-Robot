package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;

public interface Elevator {
    double WHEEL_CIRCUMFERENCE = 2.7 * Math.PI;
    double GEAR_RATIO = 48.0;
    double ENCODER_TICS_PER_INCH = GEAR_RATIO / WHEEL_CIRCUMFERENCE;
    double LEVEL_ZERO_POSITION = 0.0;
    double LEVEL_ONE_POSITION = 3.0;
    double LEVEL_TWO_POSITION = 6.0;
    double LEVEL_THREE_POSITION = 9.0;
    double LEVEL_FOUR_POSITION = 12.0;
    double CORAL_STATION_HEIGHT = 5.0;

    Command jogUpCommand();

    Command jogDownCommand();

    Command moveToCoralStationHeightCommand();

    Command moveToPositionZeroCommand();

    Command moveToLevelOneCommand();

    Command moveToLevelTwoCommand();

    Command moveToLevelThreeCommand();

    Command moveToLevelFourCommand();

    Command setCurrentPositionAsHomeCommand();

    Command updateConfigCommand();
}
