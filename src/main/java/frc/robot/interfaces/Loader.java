package frc.robot.interfaces;



import org.strongback.Executable;
import org.strongback.components.FeatureSolenoid.Mode;

/**
 * This is a system to store balls from the intake and then pass them to the shooter.
 */
public interface Loader extends Subsystem, Executable, DashboardUpdater {

    public double getTargetSpinnerRPS();

    public double getTargetPassthroughDutyCycle();

    public int getCurrentBallCount();

    public void setInitBallCount(int initBallCount);

    public void setTargetSpinnerRPS(double current);

    public void setTargetPassthroughDutyCycle(double percentoutput);

    /**
     * Set the blocker mode
     * 
     * @param mode the blocker should enabled or disabled
     * @return
     */
    public Loader setBlocker(Mode mode);

    /**
     * @return if the blocker has moved into position.
     */
    public boolean blockerIsInPosition();
}
