
package frc.robot.elevator;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase {
  private static final double TOLERANCE = 0.25;
  TalonFX motorOne = new TalonFX(0);
  public static final double INCHES_PER_ENCODER_TIC = 1.0; // determine through experiment
  private double desiredHeight;
  
    /** Creates a new Elevator. */
    public Elevator() {
  
    }
  
    @Override
    public void periodic() {
      if(atDesiredHeight()) { // within range
        motorOne.stopMotor();
      }
      else if(distanceRemaining() < 0) { // moving down
        motorOne.set(-0.25);
      }
      else { // moving up
        motorOne.set(0.25);
      }
    }

    private double distanceRemaining() {
      return desiredHeight - height();
    }

  
    public double height() {
      return motorOne.getPosition().getValueAsDouble() * INCHES_PER_ENCODER_TIC;
    }
  
    public void move(double desiredHeight) {
      this.desiredHeight = desiredHeight;
    }

    public boolean atDesiredHeight() {
      return Math.abs(distanceRemaining()) < TOLERANCE;
    } 
 }