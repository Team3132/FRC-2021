package frc.robot.interfaces;



import org.strongback.Executable;
import org.strongback.components.Solenoid.Position;

/**
 * Single wheel hooded shooter driven by three motors.
 * 
 * The hood can be extended and retracted for use directly against the goal and
 * shooting across the field respectively.
 */
public interface Shooter extends Subsystem, Executable, DashboardUpdater {

    /**
     * setTargetSpeed() sets the speed on the shooter wheels.
     * 
     * @param rps is the target speed that is being given to the shooter.
     */
    public Shooter setTargetRPS(double rps);

    public double getTargetRPS();

    public boolean isAtTargetSpeed();

    /**
     * Set the position of the hood.
     */
    public Shooter setHoodPosition(Position position);

    /**
     * @return true if the hood is in position.
     */
    public boolean hoodIsInPosition();
}
