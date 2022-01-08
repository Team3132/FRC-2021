package frc.robot.interfaces;



import org.strongback.Executable;
import org.strongback.components.FeatureSolenoid.Mode;

/*
 * The buddy climb is a bar that another robot can latch onto in an effort to bring them up
 * with us when we climb. It has a single solenoid that is extended when the subsystem is
 * deployed.
 */
public interface BuddyClimb extends Subsystem, Executable, DashboardUpdater {
    /**
     * @mode enable or disable the buddy climb.
     */
    public BuddyClimb setEnabled(Mode mode);

    /**
     * The buddy climb mechanism is in the correct position.
     * 
     * @return true if in position
     */
    public boolean isInPosition();
}
