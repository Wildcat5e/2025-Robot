// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Elevator.Level;

public class Elevator extends SubsystemBase {
  TalonFX motorOne = new TalonFX(0);
  TalonFX motorTwo = new TalonFX(1);

  public enum Level {
    CORAL1(1.0),
    CORAL2(2.0),
    CORAL3(3.0),
    CORAL4(4.0);

    public final double height;

    Level(double height) {
      this.height = height;
    }
  }

  /** Creates a new Elevator. */
  public Elevator() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void move(Level level) {
  }
 }