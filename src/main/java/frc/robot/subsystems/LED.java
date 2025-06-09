package frc.robot.subsystems;

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.led.CANdleConfiguration;

public class LED extends SubsystemBase {
  public static final int[] BLACK = {0, 0, 0};
  public static final int[] RED = {255, 0, 0};
  public static final int[] GREEN = {0, 255, 0};
  public static final int[] YELLOW = {255, 127, 0};
  public static final int[] BLUE = {0, 0, 255};
  public static final int[] ORANGE = {255, 165, 0};
  public static final int[] PURPLE = {200, 0, 255};
  public static final int[] WHITE = {255, 255, 255};
  
  private final CANdle candle = new CANdle(18);
  private final Elevator elevator;
  private final Outtake outtake;


  public LED(Elevator elevator, Outtake outtake) {
    CANdleConfiguration config = new CANdleConfiguration();
    config.stripType = LEDStripType.RGB;
    config.brightnessScalar = 0.5;
    candle.configAllSettings(config);

    setLEDs(LED.BLACK);
    this.elevator = elevator;
    this.outtake = outtake;
  }

  @Override
  public void periodic() {
    // if ((elevator.getCurrentState() == Elevator.State.NOT_MOVING || elevator.getCurrentState() == Elevator.State.HOLDING_POSITION) && (outtake.getCurrentState() == Outtake.State.IDLE)) {
    //   setLEDs(YELLOW); //Idle
    // }
    // else if (elevator.getCurrentState() == Elevator.State.HOLDING_POSITION && outtake.getCurrentState() == Outtake.State.LOADING) {
    //   setLEDs(BLUE); //Loading
    // }
    // else if (elevator.getCurrentState() == Elevator.State.HOLDING_POSITION && outtake.getCurrentState() == Outtake.State.SHOOTING) {
    //   setLEDs(PURPLE); //Shooting    
    // }
    // else if (elevator.getCurrentState() == Elevator.State.MOVING_UP && outtake.getCurrentState() == Outtake.State.IDLE) {
    //   setLEDs(GREEN); //Elevator Up  
    // }
    // else if (elevator.getCurrentState() == Elevator.State.MOVING_DOWN && outtake.getCurrentState() == Outtake.State.IDLE) {
    //   setLEDs(RED); //Elevator Down
    // }
    // else {
    //   setLEDs(WHITE); //Moving both Elevator and Outake (bad)
    // }

    if (outtake.getCurrentState() == Outtake.State.SHOOTING) {
      setLEDs(ORANGE);
    } else if (outtake.getCurrentState() == Outtake.State.LOADING) {
      setLEDs(BLUE);
    } else if(elevator.getCurrentState() == Elevator.State.MOVING_UP) {
      setLEDs(GREEN);
    } else if(elevator.getCurrentState() == Elevator.State.MOVING_DOWN) {
      setLEDs(RED);
    } else if(elevator.getCurrentState() == Elevator.State.HOLDING_POSITION) {
      setLEDs(YELLOW);
    } else if(elevator.getCurrentState() == Elevator.State.NOT_MOVING) {
      setLEDs(BLACK);
    } else if(outtake.getCurrentState() == Outtake.State.IDLE) {
      setLEDs(BLACK);
    }
  }

  public void setLEDs(int[] rgb) {
    candle.setLEDs(rgb[0], rgb[1], rgb[2]);
  }
}
