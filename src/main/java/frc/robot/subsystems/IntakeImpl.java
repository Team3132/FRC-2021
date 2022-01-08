package frc.robot.subsystems;



import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.interfaces.Intake;
import frc.robot.lib.Subsystem;
import frc.robot.lib.chart.Chart;
import org.strongback.components.Motor;
import org.strongback.components.Motor.ControlMode;
import org.strongback.components.Solenoid;
import org.strongback.components.Solenoid.Position;

/**
 * Intake Subsystem 2019:
 * On the 2019 robot the intake is pneumatically driven and using one motor to intake game objects
 */
public class IntakeImpl extends Subsystem implements Intake {
    private Motor motor;
    private Solenoid solenoid;
    private IntakeWheel intakeWheel;

    public IntakeImpl(Motor motor, Solenoid solenoid) {
        super("MecanumIntake");
        this.motor = motor;
        this.solenoid = solenoid;
        intakeWheel = new IntakeWheel(motor);
        Chart.register(() -> solenoid.isExtended(), "%s/extended", name);
        Chart.register(() -> solenoid.isRetracted(), "%s/retracted", name);
        Chart.register(motor::getOutputVoltage, "%s/outputVoltage", name);
        Chart.register(motor::getOutputPercent, "%s/outputPercent", name);
        Chart.register(motor::getSupplyCurrent, "%s/outputCurrent", name);
        Chart.register(() -> intakeWheel.getTargetRPS(), "%s/targetRPS", name);
        Chart.register(() -> intakeWheel.getRPS(), "%s/rps", name);

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
    public boolean isInPosition() {
        return solenoid.isInPosition();
    }

    @Override
    public IntakeImpl setPosition(Position position) {
        solenoid.setPosition(position);
        return this;
    }

    @Override
    public boolean isExtended() {
        return solenoid.isExtended();
    }

    @Override
    public boolean isRetracted() {
        return solenoid.isRetracted();
    }

    /**
     * Set the speed on the intake wheels.
     */
    @Override
    public Intake setTargetRPS(double rps) {
        intakeWheel.setTargetRPS(rps);
        return this;
    }

    @Override
    public double getTargetRPS() {
        return intakeWheel.getTargetRPS();
    }

    protected class IntakeWheel {

        private final Motor motor;
        private double targetRPS;

        public IntakeWheel(Motor motor) {
            this.motor = motor;
        }

        public void setTargetRPS(double rps) {
            if (rps == targetRPS) {
                return;
            }
            targetRPS = rps;
            // Note that if velocity mode is used and the speed is ever set to 0,
            // change the control mode from percent output, to avoid putting
            // unnecessary load on the battery and motor.
            if (rps == 0) {
                debug("Turning intake wheel off.");
                motor.set(ControlMode.DutyCycle, 0);
            } else {
                motor.set(ControlMode.Speed, rps);
            }
            debug("Setting intake target speed to %f", targetRPS);
        }

        public double getTargetRPS() {
            return targetRPS;
        }

        public double getRPS() {
            return motor.getSpeed();
        }
    }

    /**
     * Update the operator console with the status of the intake subsystem.
     */
    @Override
    public void updateDashboard() {
        SmartDashboard.putString("Intake position", solenoid.toString());
        SmartDashboard.putNumber("Intake motor current", motor.getOutputCurrent());
        SmartDashboard.putNumber("Intake motor target RPS", intakeWheel.getTargetRPS());
        SmartDashboard.putNumber("Intake motor actual RPS", intakeWheel.getRPS());
    }
}
