package frc.robot.subsystems;



import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.interfaces.ClimberDeployer;
import frc.robot.lib.Subsystem;
import frc.robot.lib.chart.Chart;
import frc.robot.lib.log.Log;
import org.strongback.components.FeatureSolenoid;
import org.strongback.components.FeatureSolenoid.Mode;
import org.strongback.components.Motor;
import org.strongback.components.Motor.ControlMode;
import org.strongback.components.Solenoid;

/**
 * Climber Deployment Subsystem 2021:
 * Alternative to the Zip Chain subsystem, has an arm which can be released using a solenoid and
 * then a winch which raises and lowers a hook.
 */
public class ClimberDeployerImpl extends Subsystem implements ClimberDeployer {
    private Motor motor;
    private FeatureSolenoid solenoid;

    public ClimberDeployerImpl(Motor motor, Solenoid solenoid) {
        super("Climber Deployer");
        this.motor = motor;
        this.solenoid = new FeatureSolenoid(solenoid);
        Chart.register(motor::getOutputVoltage, "%s/outputVoltage", name);
        Chart.register(motor::getOutputPercent, "%s/outputPercent", name);
        Chart.register(motor::getSupplyCurrent, "%s/outputCurrent", name);
        Chart.register(() -> getDutyCycle(), "%s/dutyCycle", name);
        Chart.register(() -> this.solenoid.isDisabled(), "%s/isReleased", name);
        Chart.register(motor::getPosition, "%s/position", name);
    }

    @Override
    public void enable() {
        motor.set(ControlMode.DutyCycle, 0);
    }

    @Override
    public void disable() {
        motor.set(ControlMode.DutyCycle, 0);
    }

    @Override
    public ClimberDeployer setDutyCycle(double dutyCycle) {
        Log.debug("climberdeployer", "duty cycle %f", dutyCycle);
        motor.set(ControlMode.DutyCycle, dutyCycle);
        return this;
    }

    @Override
    public double getDutyCycle() {
        return motor.get();
    }

    @Override
    public ClimberDeployer setHolder(Mode mode) {
        solenoid.setMode(mode);
        return this;
    }

    @Override
    public boolean isInPosition() {
        return solenoid.isInPosition();
    }

    /**
     * Update the operator console with the status of the climber deployment subsystem.
     */
    @Override
    public void updateDashboard() {
        SmartDashboard.putNumber("Climberdeployer motor current", motor.getOutputCurrent());
        SmartDashboard.putNumber("Climberdeployer motor duty cycle", getDutyCycle());
        SmartDashboard.putBoolean("Climberdeployer released", solenoid.isDisabled());
        SmartDashboard.putNumber("Climberdeployer encoder position", motor.getPosition());
    }
}
