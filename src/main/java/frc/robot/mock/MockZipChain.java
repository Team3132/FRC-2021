package frc.robot.mock;



import frc.robot.interfaces.ZipChain;

public class MockZipChain implements ZipChain {

    @Override
    public String getName() {
        return "MockZipChain";
    }

    @Override
    public void enable() {}

    @Override
    public void disable() {}

    @Override
    public void execute(long timeInMillis) {}

    @Override
    public boolean isEnabled() {
        return false;
    }

    @Override
    public void cleanup() {}

    @Override
    public ZipChain setDutyCycle(double dutyCycle) {
        return this;
    }

    @Override
    public double getDutyCycle() {
        return 0;
    }
}
