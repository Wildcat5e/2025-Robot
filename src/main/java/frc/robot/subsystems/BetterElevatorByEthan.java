// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.subsystems;

// import com.ctre.phoenix6.hardware.TalonFX;

// import edu.wpi.first.wpilibj2.command.SubsystemBase;

// public class BetterElevatorByEthan extends SubsystemBase {
//   private final TalonFX motor = new TalonFX(16);
//   private States currentState;

//   public enum States {
//     MOVING_UP,
//     MOVING_DOWN,
//     STATIONARY
//   }

//   /** Creates a new BetterElevatorByEthan. */
//   public BetterElevatorByEthan() {
//     currentState = States.STATIONARY;
//   }

//   @Override
//   public void periodic() {
//     switch (currentState){
//       case STATIONARY:
//         motor.setVoltage(0);
//         break;
//       case MOVING_DOWN:
//         motor.setVoltage(-3);
//         break;
//       case MOVING_UP:
//         motor.setVoltage(3);
//         break;
//     }
//     // This method will be called once per scheduler run
//   }

//   private void SuperAwesomeStateHandlerTransitionByEthan(){
//     motor.getPosition()
//   }
// }
