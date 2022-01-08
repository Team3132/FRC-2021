package frc.robot.subsystems;



import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Config;
import frc.robot.interfaces.ColourWheel;
import frc.robot.interfaces.ColourWheel.ColourAction.ColourWheelType;
import frc.robot.interfaces.LEDStrip;
import frc.robot.lib.LEDColour;
import frc.robot.lib.Subsystem;
import frc.robot.lib.WheelColour;
import frc.robot.lib.chart.Chart;
import java.util.function.Supplier;
import org.strongback.components.Clock;
import org.strongback.components.Motor;
import org.strongback.components.Motor.ControlMode;
import org.strongback.components.Solenoid;
import org.strongback.components.Solenoid.Position;

/**
 * This subsystem is made to spin the Colour Wheel on the control panel in the
 * 2020 game. It 5 seperate actions:
 * 
 * 1) Rotational control, spins the colour wheel 3.25 full rotations, or 26
 * eighth turns. It uses the colour wheel as an encoder, checking for every
 * colour.
 * 
 * 2) Positional control, spins the colour wheel to the selected colour,
 * choosing clockwise or anticlockwise depending on what is faster.
 * 
 * 3) Manual adjustment clockwise, moves the colour wheel clockwise at a slow
 * speed incase it is off by a bit.
 * 
 * 4) Manual adjustment anticlockwise, same as above, in the opposite direction.
 * 
 * 5) None, stops the motor. This is the default action.
 * 
 * This class expects to be given one motor and a RevRobotics Colour Sensor V3.
 */

public class ColourWheelImpl extends Subsystem implements ColourWheel {
    // Used in doubleCheck to check for mistakes with colour detection.
    private WheelColour colourPrev = WheelColour.UNKNOWN;
    private WheelColour colour = WheelColour.UNKNOWN; // Variable for what the colour sensor
                                                      // currently sees.
    private WheelColour nextColour = WheelColour.UNKNOWN;
    private int rotCount = 0; // Roation counter for rotation controls.
    // Variable to check if this is the first time the colour sensor saw the desired colour.
    private boolean firstLoop = true;
    private long spinTime; // Variable to store the time when the colour sensor sees the desired
                           // colour.
    private double speed = 0;
    // Default action for colour wheel subsystem.
    private ColourAction action = new ColourAction(ColourWheelType.NONE, WheelColour.UNKNOWN);

    private Clock clock;
    private final Motor motor;
    private Solenoid solenoid;
    private final LEDStrip ledStrip;
    private final Supplier<WheelColour> colourSensor;

    public ColourWheelImpl(Motor motor, Solenoid solenoid, Supplier<WheelColour> colourSensor,
            LEDStrip ledStrip,
            Clock clock) {
        super("ColourWheel");
        info("Creating Colour Wheel Subsystem");
        this.motor = motor;
        this.clock = clock;
        this.colourSensor = colourSensor;
        this.ledStrip = ledStrip;
        this.solenoid = solenoid;
        Chart.register(() -> (double) colour.id, "%s/colour", name);
        Chart.register(() -> (double) rotCount, "%s/rotCount", name);
        Chart.register(() -> (double) motor.getOutputPercent(), "%s/motorspeed", name);
        Chart.register(() -> (double) nextColour.id, "%s/nextColour", name);
        Chart.register(() -> (double) spinTime, "%s/spinTime", name);
    }

    public void update() {
        double newSpeed = 0;
        updateColour();
        switch (action.type) {
            case ROTATION:
                newSpeed = rotationalControl();
                ledStrip.setProgressColour(LEDColour.GREEN, LEDColour.WHITE,
                        ((double) rotCount) / Config.colourWheel.rotationTarget);
                break;
            case POSITION:
                newSpeed = positionalControl(action.colour);
                ledStrip.setAlternatingColour(LEDColour.WHITE, colour.convert());
                break;
            case ADJUST_WHEEL_ANTICLOCKWISE:
                newSpeed = Config.colourWheel.motor.adjust;
                ledStrip.setAlternatingColour(LEDColour.WHITE, colour.convert());
                break;
            case ADJUST_WHEEL_CLOCKWISE:
                newSpeed = -Config.colourWheel.motor.adjust;
                ledStrip.setAlternatingColour(LEDColour.WHITE, colour.convert());
                break;
            case NONE:
                newSpeed = Config.colourWheel.motor.off;
                break;
            default:
                error("%s: Unknown Type %s", name, action.type);
                break;
        }
        if (newSpeed != speed) {
            motor.set(ControlMode.DutyCycle, newSpeed);
            speed = newSpeed;
        }
    }

    /**
     * _________________
     * /\ | /\
     * / \ | / \
     * / \ R | Y / \
     * / \ | / \
     * / G \ | / B \
     * / \ | / \
     * | \ | / |
     * |_____________\|/____________|
     * | /|\ |
     * \ / | \ /
     * \ B / | \ G /
     * \ / | \ /
     * \ / | \ /
     * \ / Y | R \ /
     * \ / | \ /
     * \/_______|______\/
     * 
     * If colour wheel is on G, spin for 26 eighth turns by checking each colour
     * 
     * If unknown, turn at slow speed and start again.
     * 
     */
    public double rotationalControl() {
        if (colour.equals(nextColour)) {
            info("Found %s.", colour);
            info("Added one to rotations. %d", rotCount);
            rotCount += 1;
            firstLoop = true;
        }
        if (firstLoop) {
            nextColour = colour.next(speed);
            info("Next Colour is %s.", nextColour);
            firstLoop = false;
        }
        if (rotCount < Config.colourWheel.rotationTarget) {
            return Config.colourWheel.motor.full;
        } else {
            action = new ColourAction(ColourWheelType.NONE, WheelColour.UNKNOWN);
            return Config.colourWheel.motor.off;
        }
    }

    /**
     * _________________
     * /\ | /\
     * / \ | / \
     * / \ R | Y / \
     * / \ | / \
     * / G \ | / B \
     * / \ | / \
     * | \ | / |
     * |_____________\|/____________|
     * | /|\ |
     * \ / | \ /
     * \ B / | \ G /
     * \ / | \ /
     * \ / | \ /
     * \ / Y | R \ /
     * \ / | \ /
     * \/_______|______\/
     * 
     * If colour wheel is on G, turn anticlockwise for R, and clockwise at full
     * speed for anything else.
     * 
     * If unknown, turn at current speed and direction until correct colour is
     * found.
     * 
     */
    public double positionalControl(WheelColour desired) {
        double newSpeed = (colour.id - desired.id) % 4; // Calculate new speed.
        if (newSpeed == 3) {
            newSpeed -= 4; // If above calculation is 3, set speed to -1.
        }
        if (newSpeed == -3) {
            newSpeed += 4; // If above calculation is -3, set speed to 1.
        }
        newSpeed = Config.colourWheel.motor.full * Math.signum(newSpeed);
        if (colour.equals(WheelColour.UNKNOWN)) { // Colour is unknown, move in current direcion
                                                  // until colour identified.
            if (speed != 0) {
                return speed;
            } else {
                return Config.colourWheel.motor.full;
            }
        }
        if (desired.equals(colour)) { // If correct colour found, move slowly to line up better and
                                      // then stop.
            if (firstLoop) {
                spinTime = clock.currentTimeInMillis(); // Check time when correct colour found.
                firstLoop = false;
            }
            if (clock.currentTimeInMillis() - spinTime < Config.colourWheel.delayMS) {
                return motor.get();
            } else {
                action = new ColourAction(ColourWheelType.NONE, WheelColour.UNKNOWN);
                info("Desired colour found.");
                return Config.colourWheel.motor.off;
            }
        }
        return newSpeed;
    }

    public void updateColour() {
        colour = doubleCheck(colourSensor.get());
    }

    /**
     * 
     * Method to check if the current detected colour was a possible colour after
     * seeing the previous colour. Example: The colour sensor cannot see blue
     * directly after red, so it sets the detected colour back to blue and checks
     * again after the wheel has moved.
     * 
     */
    private WheelColour doubleCheck(WheelColour sensedColour) {
        if (speed == 0 || colourPrev.equals(WheelColour.UNKNOWN)) {
            colourPrev = sensedColour;
            return sensedColour;
        }
        if (sensedColour.equals(colourPrev.next(speed))) {
            colourPrev = sensedColour;
        }
        return colourPrev;
    }

    @Override
    public ColourWheel setDesiredAction(ColourAction action) {
        this.action = action;
        if (action == new ColourAction(ColourWheelType.POSITION, WheelColour.UNKNOWN)) {
            error("No colour found in FMS!");
            setDesiredAction(new ColourAction(ColourWheelType.NONE, WheelColour.UNKNOWN));
        }
        firstLoop = true;
        rotCount = 0;
        nextColour = WheelColour.UNKNOWN;
        return this;
    }

    @Override
    public ColourAction getDesiredAction() {
        return action;
    }

    @Override
    public boolean isFinished() {
        return action.type == ColourWheelType.NONE;
    }

    @Override
    public void disable() {
        motor.set(ControlMode.DutyCycle, 0);
        action = new ColourAction(ColourWheelType.NONE, WheelColour.UNKNOWN);
    }

    @Override
    public void updateDashboard() {
        SmartDashboard.putString("Colourwheel Desired colour", action.colour.toString());
        SmartDashboard.putString("Colourwheel Current colour", colour.toString());
        SmartDashboard.putNumber("Colourwheel Motor", motor.get());
        SmartDashboard.putString("Colourwheel Action", action.type.toString());
        SmartDashboard.putNumber("Colourwheel Rotations", rotCount);
        SmartDashboard.putNumber("Colourwheel spinTime", spinTime);
        SmartDashboard.putNumber("Colourwheel realTime", clock.currentTimeInMillis());
    }

    @Override
    public void setArmPosition(Position position) {
        solenoid.setPosition(position);
    }

    @Override
    public boolean armIsInPosition() {
        return solenoid.isInPosition();
    }

    @Override
    public boolean isArmExtended() {
        return solenoid.isExtended();
    }

    @Override
    public boolean isArmRetracted() {
        return solenoid.isRetracted();
    }
}
