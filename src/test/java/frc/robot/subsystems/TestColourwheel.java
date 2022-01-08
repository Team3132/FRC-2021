package frc.robot.subsystems;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import frc.robot.Config;
import frc.robot.interfaces.ColourWheel;
import frc.robot.interfaces.ColourWheel.ColourAction;
import frc.robot.interfaces.ColourWheel.ColourAction.ColourWheelType;
import frc.robot.interfaces.LEDStrip;
import frc.robot.lib.WheelColour;
import frc.robot.mock.MockLEDStrip;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.strongback.mock.Mock;
import org.strongback.mock.MockClock;
import org.strongback.mock.MockMotor;
import org.strongback.mock.MockSolenoid;

public class TestColourwheel {

    WheelColour colour;
    MockMotor motor;
    MockClock clock;
    MockSolenoid solenoid;
    ColourWheel colourWheel;
    LEDStrip ledStrip;

    @BeforeEach
    public void setup() {
        colour = WheelColour.UNKNOWN;
        motor = Mock.stoppedMotor();
        solenoid = Mock.Solenoids.singleSolenoid(0);
        clock = Mock.clock();
        ledStrip = new MockLEDStrip();
        colourWheel = new ColourWheelImpl(motor, solenoid, () -> colour, ledStrip, clock);
    }


    @Test
    public void testEnableDisable() {
        // Should start with no action and no output
        colourWheel.execute(0);
        assertEquals(0, motor.get(), 0.01);
        assertEquals(new ColourAction(ColourWheelType.NONE, WheelColour.UNKNOWN),
                colourWheel.getDesiredAction());
        assertTrue(colourWheel.isFinished());

        colourWheel.enable();
        colourWheel.execute(0);
        assertEquals(0, motor.get(), 0.01);
        assertEquals(new ColourAction(ColourWheelType.NONE, WheelColour.UNKNOWN),
                colourWheel.getDesiredAction());
        assertTrue(colourWheel.isFinished());

        colourWheel.setDesiredAction(
                new ColourAction(ColourWheelType.ADJUST_WHEEL_CLOCKWISE, WheelColour.UNKNOWN));
        colourWheel.execute(0);
        assertEquals(-Config.colourWheel.motor.adjust, motor.get(), 0.01);
        assertFalse(colourWheel.isFinished());

        colourWheel.disable();
        colourWheel.execute(0);
        assertEquals(0, motor.get(), 0.01);
        assertEquals(new ColourAction(ColourWheelType.NONE, WheelColour.UNKNOWN),
                colourWheel.getDesiredAction());
        assertTrue(colourWheel.isFinished());
    }

    @Test
    public void testAdjustClockwise() {
        colourWheel.enable();
        colourWheel.setDesiredAction(
                new ColourAction(ColourWheelType.ADJUST_WHEEL_CLOCKWISE, WheelColour.UNKNOWN));
        colourWheel.execute(0);
        assertEquals(-Config.colourWheel.motor.adjust, motor.get(), 0.01);
        assertFalse(colourWheel.isFinished());
    }

    @Test
    public void testAdjustAnticlockwise() {
        colourWheel.enable();
        colourWheel.setDesiredAction(
                new ColourAction(ColourWheelType.ADJUST_WHEEL_ANTICLOCKWISE, WheelColour.UNKNOWN));
        colourWheel.execute(0);
        assertEquals(Config.colourWheel.motor.adjust, motor.get(), 0.01);
        assertFalse(colourWheel.isFinished());
    }

    @Test
    public void testRotational() {
        for (int i = 0; i < 4; i++) {
            doRotational(i);
        }
    }

    @Test
    public void testPositional() {
        for (int i = 0; i < WheelColour.NUM_COLOURS; i++) {
            for (int x = 0; x < WheelColour.NUM_COLOURS; x++) {
                doPositional(WheelColour.of(i), WheelColour.of(x));
            }
        }
    }

    public void doRotational(int x) {
        colourWheel.enable();
        colourWheel
                .setDesiredAction(new ColourAction(ColourWheelType.ROTATION, WheelColour.UNKNOWN));
        for (int i = 0; i < Config.colourWheel.rotationTarget; i++) {
            colour = WheelColour.of(3 - ((i + x) % 4));
            colourWheel.execute(0);
            assertEquals(Config.colourWheel.motor.full, motor.get(), 0.01);
            assertFalse(colourWheel.isFinished());
        }
        colour = WheelColour.of(3 - ((Config.colourWheel.rotationTarget + x) % 4));
        colourWheel.execute(0);
        assertEquals(Config.colourWheel.motor.off, motor.get(), 0.01);
        assertTrue(colourWheel.isFinished());
    }

    public void doPositional(WheelColour desired, WheelColour start) {
        colourWheel.enable();
        colourWheel.setDesiredAction(new ColourAction(ColourWheelType.POSITION, desired));
        colour = start;
        int rotations = 0;
        int amount = (desired.id - start.id + WheelColour.NUM_COLOURS) % WheelColour.NUM_COLOURS;
        if (amount == 3)
            amount = 1;
        colourWheel.execute(0);
        if (!desired.equals(start)) {
            assertEquals(Math.signum(motor.get()) * Config.colourWheel.motor.full, motor.get(),
                    0.01);
            assertFalse(colourWheel.isFinished());
            for (int i = 0; i < amount; i++) {
                colour = WheelColour
                        .of((colour.id + WheelColour.NUM_COLOURS + -(int) Math.signum(motor.get()))
                                % WheelColour.NUM_COLOURS);
                colourWheel.execute(0);
                assertEquals(Math.signum(motor.get()) * Config.colourWheel.motor.full, motor.get(),
                        0.01);
                assertFalse(colourWheel.isFinished());
                rotations++;
                if (desired.equals(colour))
                    break;
            }
        }
        clock.incrementByMilliseconds(50);
        colourWheel.execute(0);
        assertEquals(Config.colourWheel.motor.off, motor.get(), 0.01);
        assertTrue(colourWheel.isFinished());
        assertEquals(amount, rotations);
    }
}
