package frc.robot;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.AddressableLEDBufferView;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;

public class Leds {
    private final AddressableLED leds;
    private final AddressableLEDBuffer ledsBuffer;
    private final AddressableLEDBufferView ledsBufferViewRight;
    private final AddressableLEDBufferView ledsBufferViewLeft;

    public Leds() {
        leds = new AddressableLED(RobotMap.LEDS_PORT);

        ledsBuffer = new AddressableLEDBuffer(RobotMap.LEDS_LENGTH);
        leds.setLength(RobotMap.LEDS_LENGTH);

        ledsBufferViewRight = ledsBuffer.createView(
                RobotMap.LEDS_RIGHT_SIDE_INDEX[0],
                RobotMap.LEDS_RIGHT_SIDE_INDEX[1]);
        ledsBufferViewLeft = ledsBuffer.createView(
                RobotMap.LEDS_LEFT_SIDE_INDEX[0],
                RobotMap.LEDS_LEFT_SIDE_INDEX[1]).reversed();

        leds.start();
    }

    public void setRightSideLeds(LEDPattern ledPattern) {
        ledPattern.applyTo(ledsBufferViewRight);
    }

    public void setLeftSideLeds(LEDPattern ledPattern) {
        ledPattern.applyTo(ledsBufferViewLeft);
    }

    public void onShooting() {
        LEDPattern base = LEDPattern.solid(Color.kRed);
        //base.blink().overlayOn();
        //setLeftSideLeds();
    }

    public void update() {
        leds.setData(ledsBuffer);
    }
}
