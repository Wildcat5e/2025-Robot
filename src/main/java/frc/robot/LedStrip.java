package frc.robot;

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdleConfiguration;
import com.ctre.phoenix.led.FireAnimation;
import com.ctre.phoenix.led.RainbowAnimation;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class LedStrip {
  private final CANdle candle = new CANdle(18);
  private final RainbowAnimation rainbow = new RainbowAnimation();
  private final FireAnimation fire = new FireAnimation();

  public enum Color {
    NONE(0, 0, 0),
    BLUE(0,0,255),
    RED(255, 0, 0);
    public final int r, g, b;
    Color(int r, int g, int b) {
      this.r = r;
      this.g = g;
      this.b = b;
    }
  }

  public enum Block {
    A(10, 10),
    B(20, 20),
    C(40, 30);
    public final int start, count;
    Block(int start, int count) {
      this.start=start;
      this.count=count;
    }
  }
  public LedStrip() {
    CANdleConfiguration config = new CANdleConfiguration();
    config.stripType = LEDStripType.RGB;
    config.brightnessScalar = 1.0;
    candle.configAllSettings(config);
    candle.animate(null);
    setColor(Color.NONE);
      }
    
      public void setColor(Color ledColor) {
        candle.setLEDs(ledColor.r, ledColor.g, ledColor.b);
      }
      public void setColor(Color ledColor, Block block) {
        candle.setLEDs(ledColor.r, ledColor.g, ledColor.b, 0, block.start, block.count);
      }
    
      public Command setRainbow() {
    return Commands.runOnce(() -> candle.animate(rainbow));
  }

  public void setFire() {
    candle.animate(fire);
  }

  public Command setRed() {
    return Commands.runOnce(() -> candle.setLEDs(255, 0, 0));
  }

  public Command setBlue() {
    return Commands.runOnce(() -> candle.setLEDs(0, 0, 255));
  }

  public Command setWildCat5eColors() {
    return Commands.repeatingSequence(setRed(), Commands.waitSeconds(0.5), setBlue(), Commands.waitSeconds(0.5));
  }
}
