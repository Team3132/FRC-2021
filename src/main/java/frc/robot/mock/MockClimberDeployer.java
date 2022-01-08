package frc.robot.mock;



import frc.robot.interfaces.ClimberDeployer;
import frc.robot.lib.Subsystem;
import org.strongback.components.FeatureSolenoid.Mode;

public class MockClimberDeployer extends Subsystem implements ClimberDeployer {

    public MockClimberDeployer() {
        super("MockClimberDeployer");
    }

    @Override
    public double getDutyCycle() {
        return 0;
    }

    @Override
    public ClimberDeployer setHolder(Mode mode) {
        return this;
    }

    @Override
    public ClimberDeployer setDutyCycle(double dutyCycle) {
        return this;
    }

    @Override
    public boolean isInPosition() {
        return true;
    }
}
