package frc.robot.interfaces;



import org.strongback.Executable;
import org.strongback.components.FeatureSolenoid.Mode;

/*
 * The climber deployer is an alternative to the zip chain, it consits of an arm which can be
 * released once using a solenoid and a hook which can raised and lowered with a winch.
 */
public interface ClimberDeployer extends Subsystem, Executable, DashboardUpdater {

    public ClimberDeployer setHolder(Mode mode);

    public boolean isInPosition();

    public ClimberDeployer setDutyCycle(double dutyCycle);

    public double getDutyCycle();
}
