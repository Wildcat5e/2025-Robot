# Code Structure

## Elevator

- Define an enum class at the top of the subsystem and explicitly define all of its states. Each state represents one of the movements that the subsystem can physically do.

```java
public enum State {
    NOT_MOVING,
    MOVING_UP,
    MOVING_DOWN,
    HOLDING_POSITION
}
```

- Add a switch statement to periodic to check current state and apply motor voltage based on that.

```java
switch (currentState) {
    case NOT_MOVING:
        motor.setVoltage(0.0);
        break;
    case MOVING_UP:
        motor.setVoltage(6.0);
        break;
    case MOVING_DOWN:
        motor.setVoltage(-6.0);
        break;
    case HOLDING_POSITION:
        motor.setVoltage(0.5);
        break;
}
```

- Create a void handleStateTransition method that contains an if statement that checks target height, current height, and tolerance to determine next state.

```java
private void handleStateTransition() {
    if (Math.abs(targetHeight - currentHeight) <= TOLERANCE && targetHeight > 0.0) {
        currentState = State.HOLDING_POSITION;
    } else if (targetHeight > currentHeight + TOLERANCE) {
        currentState = State.MOVING_UP;
    } else if (targetHeight < currentHeight - TOLERANCE) {
        currentState = State.MOVING_DOWN;
    } else {
        currentState = State.NOT_MOVING;
    }
}
```

- Commands should be methods in the subsystem that set a target height

```java
public Command moveToLevelTwo() {
    return run(() -> setTargetHeight(LEVEL_TWO)).until(() -> withinTolerance());
}
```

## LED

- Elevator states: fifth priority IDLE, fourth priority MOVING_UP, fourth priority MOVING_DOWN, first priority HOLDING
- Outtake  states: fifth priority IDLE, third priority LOADING, second priority priority SHOOTING