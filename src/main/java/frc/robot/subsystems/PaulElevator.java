package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Example ONLY:  Elevator as a State Machine.
 */
public class PaulElevator extends SubsystemBase {
    public enum State {
        NOT_MOVING,
        MOVING_UP,
        MOVING_DOWN;
    }

    private double desiredHeight;
    State currentState = State.NOT_MOVING;
    private double tolerance = .25;
    private double currentHeight = 0.0;

    public void moveTo(double location) {
        this.desiredHeight = location;
        currentState = moveElevator();
    }

    private double getCurrentHeight() {
        switch (currentState) {
            case MOVING_DOWN:
                currentHeight = -.1;
                break;
            case MOVING_UP:
                currentHeight = +.2;
                break;
            case NOT_MOVING:
                break;
        }
        return currentHeight;
    }

    @Override
    public void periodic() {
        switch (currentState) {
            case NOT_MOVING:
                break;
            case MOVING_DOWN:
                currentState = moveElevator();
                break;
            case MOVING_UP:
                currentState = moveElevator();
                break;
        }
    }

    private State moveElevator() {
        double heightDelta = desiredHeight - getCurrentHeight();
        State newState;
        if (Math.abs(heightDelta) < tolerance) {
            // TODO: Stop Motor
            newState = State.NOT_MOVING;
        } else if (heightDelta < 0) {
            // TODO: Calculate motor voltage to lower elevator
            newState = State.MOVING_DOWN;
        } else {
            // TODO: Calculate moter voltage to move elevator up
            newState = State.MOVING_UP;
        }
        System.out.println(String.format("State: %s, heightDelta: %d", newState, heightDelta ));
        return newState;
    }

    public void stop() {
        // TODO: Stop Motor
        currentState = State.NOT_MOVING;
    }

    public State getState() {
        return currentState;
    }

}
