package frc.robot.subsystems;

import static org.junit.jupiter.api.Assertions.assertEquals;

import frc.robot.interfaces.LEDStrip;
import frc.robot.mock.MockLEDStrip;
import java.util.ArrayList;
import java.util.function.BooleanSupplier;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.strongback.mock.Mock;
import org.strongback.mock.MockClock;
import org.strongback.mock.MockMotor;
import org.strongback.mock.MockSolenoid;

public class TestLoader {
    MockMotor spinner, passthrough;
    MockSolenoid blocker;
    LoaderImpl loader;
    BooleanSupplier inSensor = () -> false;
    BooleanSupplier outSensor = () -> false;
    BooleanSupplier passthroughInnerSensor = () -> false;
    BooleanSupplier passthroughOuterSensor = () -> false;
    BooleanSupplier loaderSensor = () -> false;
    ArrayList<Boolean> inSensorCounts = new ArrayList<Boolean>(10);
    int outSensorCounts = 0;
    LEDStrip led;
    MockClock clock;

    @BeforeEach
    public void setup() {
        spinner = Mock.stoppedMotor();
        passthrough = Mock.stoppedMotor();
        led = new MockLEDStrip();
        clock = Mock.clock();
        inSensor = () -> {
            if (inSensorCounts.size() == 0)
                return false;
            return inSensorCounts.get(0);
        };
        loader = new LoaderImpl(spinner, passthrough, blocker, inSensor, outSensor,
                passthroughInnerSensor, passthroughOuterSensor, loaderSensor, led);
    }

    @Test
    public void testSpinnerMotor() {
        loader.setTargetSpinnerRPS(10);
        assertEquals(10, loader.getTargetSpinnerRPS(), 0.01);
    }

    @Test
    public void testPassthroughMotor() {
        loader.setTargetPassthroughDutyCycle(0.5);
        assertEquals(0.5, loader.getTargetPassthroughDutyCycle(), 0.01);
    }


    @Test
    public void testInitialBalls() {
        loader.setInitBallCount(3);
        assertEquals(3, loader.getCurrentBallCount());
    }

    @Test
    public void testCounting() {
        // TODO: Finishing Implementing functionality
    }
}
