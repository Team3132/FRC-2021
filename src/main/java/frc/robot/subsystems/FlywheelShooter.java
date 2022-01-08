package frc.robot.subsystems;



import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Config;
import frc.robot.interfaces.Shooter;
import frc.robot.lib.Subsystem;
import frc.robot.lib.chart.Chart;
import org.strongback.components.Motor;
import org.strongback.components.Motor.ControlMode;
import org.strongback.components.Solenoid;
import org.strongback.components.Solenoid.Position;

/**
 * On the 2020 robot, there are three shooter motors.
 * One with an encoder and the rest without, that are under PID control for speed control.
 */
public class FlywheelShooter extends Subsystem implements Shooter {

    private final ShooterWheel flyWheel;
    private final Solenoid hood;

    public FlywheelShooter(Motor shooterMotor, Solenoid solenoid) {
        super("FlywheelShooter");
        this.hood = solenoid;
        flyWheel = new ShooterWheel(shooterMotor);
        Chart.register(() -> hood.isExtended(), "%s/extended", name);
        Chart.register(() -> hood.isRetracted(), "%s/retracted", name);
    }

    @Override
    public void disable() {
        super.disable();
        flyWheel.setTargetRPS(0);
    }

    /**
     * Set the speed on the shooter wheels.
     */
    @Override
    public Shooter setTargetRPS(double rps) {
        flyWheel.setTargetRPS(rps);
        return this;
    }

    @Override
    public double getTargetRPS() {
        return flyWheel.getTargetRPS();
    }

    @Override
    public boolean isAtTargetSpeed() {
        return Math.abs(
                flyWheel.getRPS() - flyWheel.getTargetRPS()) < Config.shooter.speed.toleranceRPS;
    }

    @Override
    public Shooter setHoodPosition(Position position) {
        hood.setPosition(position);
        return this;
    }

    @Override
    public boolean hoodIsInPosition() {
        return hood.isInPosition();
    }

    protected class ShooterWheel {

        private final Motor motor;
        private double targetRPS;

        public ShooterWheel(Motor motor) {
            this.motor = motor;

            Chart.register(() -> getTargetRPS(), "%s/targetSpeed", name);
            Chart.register(() -> getRPS(), "%s/rps", name);
            Chart.register(motor::getOutputVoltage, "%s/outputVoltage", name);
            Chart.register(motor::getOutputPercent, "%s/outputPercent", name);
            Chart.register(motor::getSupplyCurrent, "%s/outputCurrent", name);
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
                debug("Turning shooter wheel off.");
                motor.set(ControlMode.DutyCycle, 0);
            } else {
                motor.set(ControlMode.Speed, rps);
            }
            debug("Setting shooter target speed to %f", targetRPS);
        }

        public double getTargetRPS() {
            return targetRPS;
        }

        public double getRPS() {
            return motor.getSpeed();
        }
    }

    @Override
    public void updateDashboard() {
        SmartDashboard.putNumber("Shooter target rps", flyWheel.getTargetRPS());
        SmartDashboard.putNumber("Shooter actual rps", flyWheel.getRPS());
        SmartDashboard.putString("Shooter status",
                isAtTargetSpeed() ? "At target" : "Not at target");
        SmartDashboard.putString("Shooter hood", hood.toString());
    }
}
