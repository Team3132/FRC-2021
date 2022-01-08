package frc.robot.subsystems;



import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.interfaces.ZipChain;
import frc.robot.lib.Subsystem;
import frc.robot.lib.chart.Chart;
import frc.robot.lib.log.Log;
import org.strongback.components.Motor;
import org.strongback.components.Motor.ControlMode;

/**
 * Zip Chain Subsystem 2021:
 * On the 2021 robot the zip chain is used to attach hooks to the generator switch. It is driven by
 * one motor.
 */
public class ZipChainImpl extends Subsystem implements ZipChain {
    private Motor motor;

    public ZipChainImpl(Motor motor) {
        super("ZipChain");
        this.motor = motor;
        Chart.register(motor::getOutputVoltage, "%s/outputVoltage", name);
        Chart.register(motor::getOutputPercent, "%s/outputPercent", name);
        Chart.register(motor::getSupplyCurrent, "%s/outputCurrent", name);
        Chart.register(() -> getDutyCycle(), "%s/dutyCycle", name);
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
    public ZipChain setDutyCycle(double dutyCycle) {
        Log.debug("zipchain", "duty cycle %f", dutyCycle);
        motor.set(ControlMode.DutyCycle, dutyCycle);
        return this;
    }

    @Override
    public double getDutyCycle() {
        return motor.get();
    }

    /**
     * Update the operator console with the status of the zip chain subsystem.
     */
    @Override
    public void updateDashboard() {
        SmartDashboard.putNumber("Zipchain motor current", motor.getOutputCurrent());
        SmartDashboard.putNumber("Zipchain motor duty cycle", this.getDutyCycle());
    }
}
