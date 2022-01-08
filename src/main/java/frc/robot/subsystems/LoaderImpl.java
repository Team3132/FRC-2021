package frc.robot.subsystems;



import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.interfaces.DashboardUpdater;
import frc.robot.interfaces.LEDStrip;
import frc.robot.interfaces.Loader;
import frc.robot.lib.LEDColour;
import frc.robot.lib.Subsystem;
import frc.robot.lib.chart.Chart;
import java.util.function.BooleanSupplier;
import org.strongback.Executable;
import org.strongback.components.FeatureSolenoid;
import org.strongback.components.FeatureSolenoid.Mode;
import org.strongback.components.Motor;
import org.strongback.components.Motor.ControlMode;
import org.strongback.components.Solenoid;

public class LoaderImpl extends Subsystem implements Loader {
    private final Motor spinner, passthrough;
    private final FeatureSolenoid blockerSolenoid;
    private double spinnerRPS = 0;
    private final Counter inSensorCount;
    private final Counter outSensorCount;
    private BooleanSupplier passthroughInnerBallSensor;
    private BooleanSupplier passthroughOuterBallSensor;
    private BooleanSupplier loaderBallSensor;
    private int initBallCount = 0;
    private boolean loaderFull = false;
    final private LEDStrip led;
    private double targetPassthroughDutyCycle = 0;

    public LoaderImpl(final Motor loaderSpinnerMotor, final Motor loaderPassthroughMotor,
            final Solenoid blockerSolenoid,
            final BooleanSupplier inSensor, final BooleanSupplier outSensor,
            final BooleanSupplier passthroughInnerBallSensor,
            final BooleanSupplier passthroughOuterBallSensor,
            final BooleanSupplier loaderBallSensor,
            LEDStrip led) {
        super("Loader");
        this.spinner = loaderSpinnerMotor;
        this.passthrough = loaderPassthroughMotor;
        this.blockerSolenoid = new FeatureSolenoid(blockerSolenoid);
        this.led = led;
        this.passthroughInnerBallSensor = passthroughInnerBallSensor;
        this.passthroughOuterBallSensor = passthroughOuterBallSensor;
        this.loaderBallSensor = loaderBallSensor;
        inSensorCount = new Counter("loader:inSensor", inSensor);
        outSensorCount = new Counter("loader:outSensor", outSensor);

        Chart.register(() -> passthrough.getOutputCurrent(), "%s/passthrough/Current", name);
        Chart.register(() -> passthrough.getOutputPercent(), "%s/passthrough/PercentOut", name);
        Chart.register(() -> getSpinnerMotorRPS(), "%s/spinner/rps", name);
        Chart.register(() -> getTargetSpinnerRPS(), "%s/spinner/targetRPS", name);
        Chart.register(() -> spinner.getOutputCurrent(), "%s/spinner/Current", name);
        Chart.register(() -> spinner.getOutputPercent(), "%s/spinner/PercentOut", name);
        Chart.register(() -> (double) getCurrentBallCount(), "%s/spinner/CurrentBallCount", name);
        Chart.register(() -> (double) inSensorCount.count, "%s/spinner/totalBallsIn", name);
        Chart.register(() -> (double) outSensorCount.count, "%s/spinner/totalBallsOut", name);
        Chart.register(() -> (double) initBallCount, "%s/spinner/initialBallCount", name);
        Chart.register(() -> blockerSolenoid.isRetracted(), "%s/blockerRetracted", name);
        Chart.register(() -> inSensor.getAsBoolean(), "%s/spinner/inSensorState", name);
        Chart.register(() -> outSensor.getAsBoolean(), "%s/spinner/outSensorState", name);
        Chart.register(() -> passthroughInnerBallSensor.getAsBoolean(),
                "%s/passthrough/innerBallSensor",
                name);
        Chart.register(() -> passthroughOuterBallSensor.getAsBoolean(),
                "%s/passthrough/outerBallSensor",
                name);
        Chart.register(() -> loaderBallSensor.getAsBoolean(), "%s/spinner/ballSensor", name);
    }

    @Override
    public void setTargetSpinnerRPS(final double rps) {

        spinnerRPS = rps;
        debug("Setting loader motor rps to: %f", rps);

        // If motor is zero in velocity the PID will try and reverse the motor in order
        // to slow down
        if (rps == 0) {
            spinner.set(ControlMode.DutyCycle, 0);
        } else {
            spinner.set(ControlMode.Speed, rps);
        }
    }

    @Override
    public double getTargetSpinnerRPS() {
        return spinnerRPS;
    }

    public double getSpinnerMotorRPS() {
        return spinner.getSpeed();
    }

    private boolean passthroughHasBall() {
        return passthroughInnerBallSensor.getAsBoolean()
                || passthroughOuterBallSensor.getAsBoolean();
    }

    private boolean loaderIsFull() {
        return loaderBallSensor.getAsBoolean();
    }

    @Override
    public void setTargetPassthroughDutyCycle(double percent) {
        debug("Setting loader in motor percent output to: %f", percent);
        // If motor is zero in velocity the PID will try and reverse the motor in order
        // to slow down
        targetPassthroughDutyCycle = percent;
    }

    @Override
    public double getTargetPassthroughDutyCycle() {
        return targetPassthroughDutyCycle;
    }

    @Override
    public void setInitBallCount(int initBallCount) {
        this.initBallCount = initBallCount;
    }

    @Override
    public Loader setBlocker(Mode mode) {
        blockerSolenoid.setMode(mode);
        return this;
    }

    @Override
    public boolean blockerIsInPosition() {
        return blockerSolenoid.isInPosition();
    }

    @Override
    public void execute(final long timeInMillis) {

        inSensorCount.execute(0);
        outSensorCount.execute(0);

        // Don't update all the time because the colour wheel may want to use the LEDs
        if (spinner.getSpeed() > 1) {
            led.setProgressColour(LEDColour.PURPLE, LEDColour.WHITE,
                    getCurrentBallCount() / 5 * 100);
        }

        /**
         * The loader can only hold 4 balls at a time so if there's no more space and the
         * passthrough is holding the 5th ball, stop passthrough.
         * 
         * The passthrough can run in reverse so only stop the passthrough if balls going into the
         * robot.
         */
        if (passthroughHasBall() && loaderIsFull() && passthrough.get() > 0) {
            if (!loaderFull) {
                debug("Stopping passthrough, loader & passthrough full");
                loaderFull = true;
            }
            passthrough.set(ControlMode.DutyCycle, 0);
        } else {
            if (loaderFull) {
                debug("Loader has room");
                loaderFull = false;
            }
            passthrough.set(ControlMode.DutyCycle, targetPassthroughDutyCycle);
        }
    }

    /**
     * Update the operator console with the status of the hatch subsystem.
     */
    @Override
    public void updateDashboard() {
        SmartDashboard.putString("Loader blocker", blockerSolenoid.toString());
        SmartDashboard.putNumber("Loader spinner rps", getSpinnerMotorRPS());
        SmartDashboard.putNumber("Loader spinner target rps", getTargetSpinnerRPS());
        SmartDashboard.putNumber("Loader passthrough percent output",
                passthrough.getOutputPercent());
        SmartDashboard.putNumber("Current number of balls", getCurrentBallCount());
        inSensorCount.updateDashboard();
        outSensorCount.updateDashboard();
        SmartDashboard.putString("Loader Ball ",
                loaderBallSensor.getAsBoolean() ? "Detected" : "Not detected");
        SmartDashboard.putString("Inner Passthrough Ball ",
                passthroughInnerBallSensor.getAsBoolean() ? "Detected" : "Not detected");
        SmartDashboard.putString("Outer Passthrough Ball ",
                passthroughOuterBallSensor.getAsBoolean() ? "Detected" : "Not detected");
    }

    @Override
    public void enable() {
        spinner.set(ControlMode.DutyCycle, 0);
        passthrough.set(ControlMode.DutyCycle, 0);
    }

    @Override
    public void disable() {
        spinner.set(ControlMode.DutyCycle, 0);
        passthrough.set(ControlMode.DutyCycle, 0);
    }

    @Override
    public int getCurrentBallCount() {
        return inSensorCount.getCount() - outSensorCount.getCount() + initBallCount;
    }

    private class Counter implements DashboardUpdater, Executable {
        final private String name;
        final private BooleanSupplier sensor;
        private int count = 0;
        private boolean lastSensorReading = false;

        public Counter(final String name, final BooleanSupplier sensor) {
            this.name = name;
            this.sensor = sensor;
            Chart.register(() -> (double) getCount(), "%s/count", name);
        }

        public int getCount() {
            return count;
        }

        @Override
        public void execute(final long timeInMillis) {
            final boolean sensorReading = sensor.getAsBoolean();
            if (sensorReading && !lastSensorReading)
                count++;
            lastSensorReading = sensorReading;
        }

        @Override
        public void updateDashboard() {
            SmartDashboard.putNumber(name + " ball count", getCount());
            SmartDashboard.putBoolean(name + " sensor state", sensor.getAsBoolean());
        }
    }


}
