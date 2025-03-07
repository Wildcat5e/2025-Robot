package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

public interface Elevator {
    double WHEEL_CIRCUMFERENCE = 2.7 * Math.PI;
    double GEAR_RATIO = 48.0;
    double ENCODER_TICS_PER_INCH = GEAR_RATIO / WHEEL_CIRCUMFERENCE;
    double LEVEL_ZERO_POSITION = 0.0;
    double LEVEL_ONE_POSITION = 0.0;
    double LEVEL_TWO_POSITION = 50.0;
    double LEVEL_THREE_POSITION = 370.0;

    Command jogUpCommand();

    Command jogDownCommand();

    Command moveToHomePositionCommand();

    Command moveToLevelOneCommand();

    Command moveToLevelTwoCommand();

    Command moveToLevelThreeCommand();

    Command updateConfigCommand();

    boolean isElevatorNotMoving();
    
    Command sysIdQuasistaticCommand(SysIdRoutine.Direction direction);

    Command sysIdDynamicCommand(SysIdRoutine.Direction direction);

    // boolean isElevatorAtHomePosition();
}