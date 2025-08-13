package frc.robot.subsystems;

import java.util.TreeMap;

import com.ctre.phoenix.led.CANdle;

public class LED {
    private final CANdle candle = new CANdle(18); // check param
    private final TreeMap<Integer, Color> request = new TreeMap<>();

    public enum Color {
        RED(255, 0, 0),
        GREEN(0, 255, 0),
        BLUE(0, 0, 255); // add more colors

        private final int red;
        private final int green;
        private final int blue;

        Color(int red, int green, int blue) {
            this.red = red;
            this.green = green;
            this.blue = blue;
        }
    }

    private LED() {
    }

    public void setColor(int priority, Color color) {
        request.put(priority, color);
        candle.setLEDs(request.firstEntry().getValue().red, request.firstEntry().getValue().green, request.firstEntry().getValue().blue);
    }

    // for testing
    public static void main(String[] args) {
        LED led = new LED();

        led.setColor(1, Color.RED);
        led.setColor(2, Color.GREEN);
        led.setColor(3, Color.BLUE);

        System.out.println();
    }
}
