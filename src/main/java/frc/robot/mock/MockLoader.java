package frc.robot.mock;



import frc.robot.interfaces.Loader;
import frc.robot.lib.Subsystem;
import org.strongback.components.FeatureSolenoid.Mode;

public class MockLoader extends Subsystem implements Loader {
    private double spinnerRPS = 0;
    private double passthroughPower = 0;

    public MockLoader() {
        super("MockLoader");
    }

    @Override
    public Loader setBlocker(Mode mode) {
        return this;
    }

    @Override
    public boolean blockerIsInPosition() {
        return true;
    }

    @Override
    public void setTargetSpinnerRPS(double rps) {
        spinnerRPS = rps;
    }

    @Override
    public void setTargetPassthroughDutyCycle(double percent) {
        passthroughPower = percent;
    }

    @Override
    public double getTargetSpinnerRPS() {
        return spinnerRPS;
    }

    @Override
    public double getTargetPassthroughDutyCycle() {
        return passthroughPower;
    }

    @Override
    public int getCurrentBallCount() {
        return 0;
    }

    @Override
    public void setInitBallCount(int initBallCount) {}
}
