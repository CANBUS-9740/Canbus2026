package frc.robot;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;

import java.sql.Time;
import java.util.Map;

import static edu.wpi.first.units.Units.*;

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


        leds.setData(ledsBuffer);
        leds.start();
    }

    private void setRightSideLeds(LEDPattern ledPattern) {
        ledPattern.applyTo(ledsBufferViewRight);
    }

    private void setLeftSideLeds(LEDPattern ledPattern) {
        ledPattern.applyTo(ledsBufferViewLeft);
    }

    public void setLedsss(){
        LEDPattern pattern = LEDPattern.solid(Color.kRed);
        pattern.applyTo(ledsBuffer);
    }

    private Command setBothPattern(LEDPattern pattern) {
        return new ParallelCommandGroup(
                Commands.runOnce(() -> setLeftSideLeds(pattern)),
                Commands.runOnce(() -> setRightSideLeds(pattern))
        );
    }

    public Command onShooting() {
        LEDPattern base = LEDPattern.solid(Color.kYellow);
        LEDPattern overlay = LEDPattern.solid(Color.kRed);
        base.blink(Seconds.of(0.5)).overlayOn(overlay);

        return setBothPattern(base);
    }

    public Command defaultLeds() {
        Map<Double, Color> maskSteps = Map.of(0.0, Color.kYellow, 0.5, Color.kGreen);

        Distance ledSpacing = Meters.of((double) (RobotMap.LEDS_LENGTH / 2) / (double) RobotMap.LEDS_LENGTH);
        LinearVelocity velocity = MetersPerSecond.of(0.5);

        LEDPattern base = LEDPattern.solid(Color.kYellow);
        LEDPattern mask = LEDPattern.steps(maskSteps).scrollAtAbsoluteSpeed(velocity, ledSpacing);

        LEDPattern pattern = base.mask(mask);

        return setBothPattern(pattern);
    }

    private Command setFuelStatusLeds(Color color, double time){
        LEDPattern base = LEDPattern.solid(color);
        LEDPattern pattern = base.breathe(Seconds.of(time));

        return setBothPattern(pattern);
    }

    public Command emptyLeds() {
        return setFuelStatusLeds(Color.kRed, 2);
    }

    public Command halfFuelLeds() {
        return setFuelStatusLeds(Color.kOrange, 1);
    }

    public Command fullFuelLeds() {
        return setFuelStatusLeds(Color.kGreen, 0.4);
    }

    public void update() {
        leds.setData(ledsBuffer);
    }
}
