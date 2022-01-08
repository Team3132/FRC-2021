package frc.robot.interfaces;



import org.strongback.Executable;

/*
 * The zip chain is made of 2 separate chains that are spooled up for storage. When deployed the 2
 * chains mesh together to form a solid column used to deploy the hook for climbing.
 */
public interface ZipChain extends Subsystem, Executable, DashboardUpdater {

    public ZipChain setDutyCycle(double dutyCycle);

    public double getDutyCycle();
}
