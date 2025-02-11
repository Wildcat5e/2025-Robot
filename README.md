This is WildCat5e code for the 2025 competition

Use Cases:
-----------
When the operator hits button 1 on the numberpad, the elevator will jog down.

When the operator hits button 2 on the numberpad, the elevator will jog up.

When the operator hits button 3 on the numberpad, the elevator will move to the height of the station and the arm will move to the intake position.

When the operator hits button 4 on the numberpad, the elevator will move to position 0 and the arm will move to the outtake position.

When the operator hits button 5 on the numberpad, the elevator will move to level 1 and the arm will move to the outtake position.

When the operator hits button 6 on the numberpad, the elevator will move to level 2 and the arm will move to the outtake position.

When the operator hits button 7 on the numberpad, the elevator will move to level 3 and the arm will move to the outtake position.

When the operator hits button 8 on the numberpad, the elevator will move to level 4 and the arm will move to the outtake position.

### Button Mapping

| Device                 | Button   | Subsystem  | Usage                                                             |
|------------------------|----------|------------|-------------------------------------------------------------------|
| Driver Xbox Controller | Left X   | Drivetrain | Move left joystick horizontally to control the robot translation  |
| Driver Xbox Controller | Left Y   | Drivetrain | Move left joystick vertically to control the robot translation    |
| Driver Xbox Controller | Right X  | Drivetrain | Move right joystick horizonally to control the robot rotation     |
| Operator Numberpad     | 1        | Elevator   | jog down                                                          |
| Operator Numberpad     | 2        | Elevator   | jog up                                                            |
| Operator Numberpad     | 3        | Elevator   | move to the height of the coral station.                          |
| Operator Numberpad     | 4        | Elevator   | move to position 0 and the arm will move to the outtake position. |
| Operator Numberpad     | 5        | Elevator   | move to level 1.                                                  |
| Operator Numberpad     | 6        | Elevator   | move to level 2.                                                  |
| Operator Numberpad     | 7        | Elevator   | move to level 3.                                                  |
| Operator Numberpad     | 8        | Elevator   | move to level 4.                                                  |
|                        |          | Algae      | Extract from reef                                                 |
|                        |          | Algae      | Eject into processor                                              |

### Can Devices

| CAN Bus  | Type     | ID | Usage               |
|----------|----------|----|---------------------|
| Rio      | Talon    | 0  | Elevator Motor One  |
| Rio      | Talon    | 1  | Elevator Motor Two  |
| Rio      | Talon    | 2  | Arm Motor One       |
| Rio      | Talon    | 3  | Arm Motor Two       |
| Canivore | Falcon   |    | Front Left Drive    |
| Canivore | Falcon   |    | Front Left Steer    |
| Canivore | CanCoder |    | Front Left Encoder  |
| Canivore | Falcon   |    | Front Right Drive   |
| Canivore | Falcon   |    | Front Right Steer   |
| Canivore | CanCoder |    | Front Right Encoder |
| Canivore | Falcon   |    | Rear Left Drive     |
| Canivore | Falcon   |    | Rear Left Steer     |
| Canivore | CanCoder |    | Rear Left Encoder   |
| Canivore | Falcon   |    | Rear Right Drive    |
| Canivore | Falcon   |    | Rear Right Steer    |
| Canivore | CanCoder |    | Rear Right Encoder  |