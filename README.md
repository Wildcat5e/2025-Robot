This is WildCat5e code for the 2025 competition

### Button Mapping

| Device                 | Button   | Subsystem            | Usage                                                                       |
|------------------------|----------|----------------------|-----------------------------------------------------------------------------|
| Driver Xbox Controller | Left X   | Drivetrain           | Move left joystick horizontally to control the robot translation            |
| Driver Xbox Controller | Left Y   | Drivetrain           | Move left joystick vertically to control the robot translation              |
| Driver Xbox Controller | Right X  | Drivetrain           | Move right joystick horizonally to control the robot rotation               |
| Operator Numberpad     | 1        | Limelight            | Limelight aligns drivetrain to left reef branch                             |
| Operator Numberpad     | 3        | Limelight            | Limelight aligns drivetrain to right reef branch                            |
| Operator Numberpad     | 11       | Elevator and Outtake | Elevator moves to level 1, outtakes, and then moves back down to position 0 |
| Operator Numberpad     | 8        | Elevator and Outtake | Elevator moves to level 2, outtakes, and then moves back down to position 0 |
| Operator Numberpad     | 5        | Elevator and Outtake | Elevator moves to level 3, outtakes, and then moves back down to position 0 |
| Operator Numberpad     | 2        | Elevator and Outtake | Elevator moves to level 4, outtakes, and then moves back down to position 0 |
| Operator Numberpad     | 4        | Elevator             | Jog up                                                                      |
| Operator Numberpad     | 6        | Elevator             | Jog down                                                                    |
| Operator Numberpad     | 7        | Outtake              | Jog up                                                                      |
| Operator Numberpad     | 9        | Outtake              | Jog down                                                                    |
| Operator Numberpad     | 10       | Intake               | Jog up                                                                      |
| Operator Numberpad     | 12       | Intake               | Jog down                                                                    |

### Can Devices

| CAN Bus  | Type     | ID | Usage               |
|----------|----------|----|---------------------|
| Canivore | Falcon   | 1  | Front Left Drive    |
| Canivore | Falcon   | 2  | Front Left Steer    |
| Canivore | CanCoder | 3  | Front Left Encoder  |
| Canivore | Falcon   | 4  | Front Right Drive   |
| Canivore | Falcon   | 5  | Front Right Steer   |
| Canivore | CanCoder | 6  | Front Right Encoder |
| Canivore | Falcon   | 7  | Rear Left Drive     |
| Canivore | Falcon   | 8  | Rear Left Steer     |
| Canivore | CanCoder | 9  | Rear Left Encoder   |
| Canivore | Falcon   | 10 | Rear Right Drive    |
| Canivore | Falcon   | 11 | Rear Right Steer    |
| Canivore | CanCoder | 12 | Rear Right Encoder  |
| Rio      | Talon    | 13 | Elevator Motor One  |
| Rio      | Talon    | 14 | Elevator Motor Two  |
| Rio      | Talon    | 15 | Outtake Motor One   |
| Rio      | Talon    | 16 | Outtake Motor Two   |