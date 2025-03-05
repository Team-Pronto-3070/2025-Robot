package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.AddressableLED.ColorOrder;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDSubsystem extends SubsystemBase {

    private final AddressableLED led;
    private AddressableLEDBuffer buffer;

    // all hues at maximum saturation and half brightness
    private final LEDPattern rainbow = LEDPattern.rainbow(255, 64);

    // Our LED strip has a density of 120 LEDs per meter
    private static final Distance kLedSpacing = Meters.of(1 / 14.0);

    // Create a new pattern that scrolls the rainbow pattern across the LED strip,
    // moving at a speed
    // of 1 meter per second.
    public final LEDPattern scrollingRainbow = rainbow.scrollAtAbsoluteSpeed(MetersPerSecond.of(1), kLedSpacing);

    private LEDPattern pattern;

    public LEDSubsystem() {
        // PWM port 0
        // Must be a PWM header, not MXP or DIO
        led = new AddressableLED(0);
        led.setColorOrder(ColorOrder.kRGB);

        // Reuse buffer
        // Default to a length of 60, start empty output
        // Length is expensive to set, so only set it once, then just update data
        buffer = new AddressableLEDBuffer(12);
        led.setLength(buffer.getLength());

        // Set the data
        led.setData(buffer);
        led.start();

        pattern = LEDPattern.solid(Color.kRed);
    }

    public void setColor(Color c) {
        // Create an LED pattern that sets the entire strip to solid red
        pattern = LEDPattern.solid(c);
        // pattern.applyTo(buffer);
        // led.setData(buffer);
    } 

    public void breathe(Color c, double seconds) {
        // Create an LED pattern that sets the entire strip to solid red
        pattern = LEDPattern.solid(c).breathe(Time.ofBaseUnits(seconds, Units.Seconds));
        // pattern.applyTo(buffer);
        // led.setData(buffer);
    }   

    public void setPattern(LEDPattern p) {
        // Update the buffer with the rainbow animation
        pattern = p;
    }

    @Override
    public void periodic() {
        pattern.applyTo(buffer);
        // // Periodically send the latest LED color data to the LED strip for it to
        // // display
        led.setData(buffer);
    }
}
