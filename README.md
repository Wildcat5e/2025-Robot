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

| Device                | Button   | Usage                                                                                                  |
|-----------------------|----------|--------------------------------------------------------------------------------------------------------|
| Driver Xbox Controller| Joysticks| Move drivetrain                                                                                        |
| Operator Numberpad    | 1        | The elevator will jog down                                                                             |
| Operator Numberpad    | 2        | The elevator will jog up                                                                               |
| Operator Numberpad    | 3        | The elevator will move to the height of the coral station and the arm will move to the intake position.|
| Operator Numberpad    | 4        | The elevator will move to position 0 and the arm will move to the outtake position.                    |
| Operator Numberpad    | 5        | The elevator will move to level 1 and the arm will move to the outtake position.                       |
| Operator Numberpad    | 6        | The elevator will move to level 2 and the arm will move to the outtake position.                       |
| Operator Numberpad    | 7        | The elevator will move to level 3 and the arm will move to the outtake position.                       |
| Operator Numberpad    | 8        | The elevator will move to level 4 and the arm will move to the outtake position.                       |

### Can Devices

| CAN Bus | Type     | ID | Usage                  |
|---------|----------|----|------------------------|
| Rio     | Talon    | 0  | Elevator Motor One     |
| Rio     | Talon    | 1  | Elevator Motor Two     |
| Canivore| Falcon   |    | Front Left Drive       |
| Canivore| Falcon   |    | Front Left Steer       |
| Canivore| CanCoder |    | Front Left Encoder     |
| Canivore| Falcon   |    | Front Right Drive      |
| Canivore| Falcon   |    | Front Right Steer      |
| Canivore| CanCoder |    | Front Right Encoder    |
| Canivore| Falcon   |    | Rear Left Drive        |
| Canivore| Falcon   |    | Rear Left Steer        |
| Canivore| CanCoder |    | Rear Left Encoder      |
| Canivore| Falcon   |    | Rear Right Drive       |
| Canivore| Falcon   |    | Rear Right Steer       |
| Canivore| CanCoder |    | Rear Right Encoder     |