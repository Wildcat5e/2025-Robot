package frc.robot;

import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.ElevatorPID;
import frc.robot.subsystems.Outtake;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class Superstructure {
    private final Elevator elevator = new ElevatorPID();
    private final Outtake outtake = new Outtake();

    public SequentialCommandGroup moveElevatorAndOuttake() {
        return new SequentialCommandGroup(elevator.moveToLevelOneCommand(), new ParallelDeadlineGroup(new WaitCommand(1.0), outtake.outtakeCommand()));
    }
}
