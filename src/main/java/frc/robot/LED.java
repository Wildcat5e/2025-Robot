package frc.robot;

import com.ctre.phoenix.led.Animation;
import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdleConfiguration;

public class LED {
  public static final int[] YELLOW = {255, 255, 0};
  public static final int[] GREEN = {0, 255, 0};
  public static final int[] RED = {255, 0, 0};
  public static final int[] BLACK = {0, 0, 0};
  private final CANdle candle = new CANdle(18);

  public LED() {
    CANdleConfiguration config = new CANdleConfiguration();
    config.stripType = LEDStripType.RGB;
    config.brightnessScalar = 1.0;
    candle.configAllSettings(config);

    setLEDs(LED.BLACK);
  }

  public void setLEDs(int[] rgb) {
    candle.clearAnimation(0);
    candle.setLEDs(rgb[0], rgb[1], rgb[2]);
  }

  public void setLEDs(Animation animation) {
    candle.clearAnimation(0);
    candle.animate(animation);
  }
}
