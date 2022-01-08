package frc.robot.mock;



import frc.robot.interfaces.BuddyClimb;
import org.strongback.components.FeatureSolenoid.Mode;

public class MockBuddyClimb implements BuddyClimb {
    private Mode mode = Mode.DISABLED;

    public MockBuddyClimb() {}

    @Override
    public BuddyClimb setEnabled(Mode mode) {
        this.mode = mode;
        return this;
    }

    @Override
    public boolean isInPosition() {
        return true;
    }

    @Override
    public boolean isEnabled() {
        return mode == Mode.ENABLED;
    }

    @Override
    public String getName() {
        return "MockBuddyClimb";
    }

    @Override
    public void enable() {}

    @Override
    public void disable() {}

    @Override
    public void execute(long timeInMillis) {}

    @Override
    public void cleanup() {}
}
