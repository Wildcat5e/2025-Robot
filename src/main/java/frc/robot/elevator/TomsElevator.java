
package frc.robot.elevator;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class TomsElevator extends SubsystemBase {
    public static final double INCHES_PER_ENCODER_TIC = 1.0; // determine through experiment
    public static final double TOLERANCE_INCHES = 0.20;
    public static final double MAX_ELEVATOR_SPEED = 1.0;

    private final PIDController pidController;
    private final TalonFX motor;

    public TomsElevator(PIDController pidController, TalonFX motor) {
        this.pidController = pidController;
        this.motor = motor;
        this.pidController.setTolerance(TOLERANCE_INCHES);
    }

    public TomsElevator() {
        this(new PIDController(0.1, 0.0, 0.0), new TalonFX(0)); // https://en.wikipedia.org/wiki/Ziegler%E2%80%93Nichols_method
    }

    @Override
    public void periodic() {
        double currentHeight = motor.getPosition().getValueAsDouble() * INCHES_PER_ENCODER_TIC;
        double speed = pidController.calculate(currentHeight);
        double clamped_speed = Math.max(-MAX_ELEVATOR_SPEED, Math.min(MAX_ELEVATOR_SPEED, speed));
        motor.set(clamped_speed);
    }

    public void moveTo(double desiredHeightInches) {
        pidController.setSetpoint(desiredHeightInches);
    }

}