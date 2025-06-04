package frc.robot;

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdleConfiguration;

public class LED {
  public static final int[] BLACK = {0, 0, 0};
  public static final int[] RED = {255, 0, 0};
  public static final int[] GREEN = {0, 255, 0};
  public static final int[] YELLOW = {255, 127, 0};
  public static final int[] BLUE = {0, 0, 255};
  public static final int[] ORANGE = {255, 165, 0};
  public static final int[] BLOCK_1 = {0, 149};
  public static final int[] BLOCK_2 = {150, 299};
  private final CANdle candle = new CANdle(18);

  public LED() {
    CANdleConfiguration config = new CANdleConfiguration();
    config.stripType = LEDStripType.RGB;
    config.brightnessScalar = 0.5;
    candle.configAllSettings(config);

    setLEDs(LED.BLACK);
  }

  public void setLEDs(int[] rgb) {
    candle.setLEDs(rgb[0], rgb[1], rgb[2]);
  }

  public void setLEDs(int[] rgb, int start, int end) {
    candle.setLEDs(rgb[0], rgb[1], rgb[2], 0, start, end);
  }
}
