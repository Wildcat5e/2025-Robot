package frc.robot.elevator;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.measure.Angle;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.mockito.Mock;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.mockito.Mockito.when;
import static org.mockito.MockitoAnnotations.openMocks;

class TomsElevatorTest {

    AutoCloseable mocks;

    @Mock TalonFX motor;
    @Mock StatusSignal<Angle> position;

    PIDController pidController = new PIDController(1.0, 0.0, 0.0);

    TomsElevator elevator;

    @BeforeEach void setUp() {
        mocks = openMocks(this);
        when(motor.getPosition()).thenReturn(position);

        elevator = new TomsElevator(pidController, motor);
    }

    @AfterEach void tearDown() throws Exception {
        mocks.close();
    }

    @Test void testMove() {
        when(position.getValueAsDouble())
                .thenReturn(0.0, 0.5, 1.0, 1.5, 1.75, 2.0);


        elevator.moveTo(2.0);

        assertEquals(2.0, pidController.getSetpoint());

        elevator.periodic();
        elevator.periodic();
        elevator.periodic();
        elevator.periodic();
        elevator.periodic();

    }
}