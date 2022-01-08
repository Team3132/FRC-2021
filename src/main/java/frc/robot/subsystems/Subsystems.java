package frc.robot.subsystems;



import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorSensorV3;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import frc.robot.Config;
import frc.robot.interfaces.*;
import frc.robot.lib.JevoisImpl;
import frc.robot.lib.LEDColour;
import frc.robot.lib.MotorFactory;
import frc.robot.lib.NavXGyroscope;
import frc.robot.lib.WheelColour;
import frc.robot.mock.*;
import frc.robot.simulator.IntakeSimulator;
import java.io.IOException;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;
import org.strongback.Executor.Priority;
import org.strongback.Strongback;
import org.strongback.components.Clock;
import org.strongback.components.Gyroscope;
import org.strongback.components.Motor;
import org.strongback.components.PneumaticsModule;
import org.strongback.components.Solenoid;
import org.strongback.hardware.Hardware;
import org.strongback.mock.Mock;

/**
 * Contains the subsystems for the robot.
 * 
 * Makes it easy to pass all subsystems around.
 */
public class Subsystems implements DashboardUpdater, LogHelper {
    // Not really a subsystem, but used by all subsystems.
    public Clock clock;
    public LEDStrip ledStrip;
    public Location location;
    public Drivebase drivebase;
    public Intake intake;
    public Intake hwIntake;
    public ZipChain zipChain;
    public ClimberDeployer climberDeployer;
    public BuddyClimb buddyClimb;
    public OverridableSubsystem<Intake> intakeOverride;
    public Loader loader;
    public Loader hwLoader; // Keep track of the real hardware for dashboard update
    public OverridableSubsystem<Loader> loaderOverride;
    public Shooter shooter;
    public Shooter hwShooter;
    public OverridableSubsystem<Shooter> shooterOverride;
    public ColourWheel colourWheel;
    public PneumaticsModule pcm;
    public Vision vision;
    public Jevois jevois;
    public Monitor monitor;

    private final I2C.Port i2cPort = I2C.Port.kOnboard;

    public Subsystems(Clock clock) {
        this.clock = clock;
    }

    public void createOverrides() {
        createIntakeOverride();
        createLoaderOverride();
        createShooterOverride();
    }

    public void enable() {
        info("Enabling subsystems");
        // location is always enabled.
        drivebase.enable();
        intake.enable();
        shooter.enable();
        loader.enable();
        colourWheel.enable();
    }

    public void disable() {
        info("Disabling Subsystems");
        drivebase.disable();
        intake.disable();
        shooter.disable();
        loader.disable();
        colourWheel.disable();
    }

    @Override
    public void updateDashboard() {
        drivebase.updateDashboard();
        hwIntake.updateDashboard();
        location.updateDashboard();
        hwLoader.updateDashboard();
        hwShooter.updateDashboard();
        vision.updateDashboard();
        colourWheel.updateDashboard();
    }

    /**
     * Create the drivebase subsystem. Creates the motors.
     */
    public void createDrivebase() {
        if (!Config.drivebase.present) {
            debug("Using mock drivebase");
            drivebase = new MockDrivebase();
            return;
        }
        Motor leftMotor = MotorFactory.getDriveMotor(true, clock);
        Motor rightMotor = MotorFactory.getDriveMotor(false, clock);
        Solenoid ptoSolenoid = pcm.singleSolenoid(Config.climber.ptoPort, 0.1, 0.1);
        Solenoid ratchetSolenoid = pcm.singleSolenoid(Config.climber.ratchetPort, 0.1, 0.1);

        leftMotor.setPosition(0);
        rightMotor.setPosition(0);
        try {
            // Let the encoders get the message and have time to send it back to us.
            Thread.sleep(100);
        } catch (InterruptedException e) {
        }
        error("Reset drive encoders to zero, currently are: %f, %f", leftMotor.getPosition(),
                rightMotor.getPosition());
        // metres.
        drivebase =
                new DrivebaseImpl(leftMotor, rightMotor, ptoSolenoid, ratchetSolenoid);
        Strongback.executor().register(drivebase, Priority.HIGH);
    }

    /**
     * Create the location subsystem. Creates the gyro.
     */
    public void createLocation() {
        if (!Config.drivebase.present) {
            debug("No drivebase, using mock location");
            location = new MockLocation();
            return;
        }
        Gyroscope gyro = new NavXGyroscope("NavX", Config.navx.present);
        gyro.zero();
        // Encoders must return metres.
        location = new LocationImpl(drivebase, gyro, clock);
        Strongback.executor().register(location, Priority.HIGH);
    }

    public void createIntake() {
        if (!Config.intake.present) {
            intake = hwIntake = new MockIntake();
            debug("Intake not present, using a mock intake instead");
            return;
        }

        Solenoid intakeSolenoid = pcm.singleSolenoid(Config.intake.solenoidPort, 0.2, 0.2);
        Motor intakeMotor = MotorFactory.getIntakeMotor();
        intake = hwIntake = new IntakeImpl(intakeMotor, intakeSolenoid);
    }

    public void createIntakeOverride() {
        // Setup the diagBox so that it can take control.
        IntakeSimulator simulator = new IntakeSimulator();
        MockIntake mock = new MockIntake();
        intakeOverride = new OverridableSubsystem<Intake>("intake", Intake.class, intake, simulator,
                mock);
        // Plumb accessing the intake through the override.
        intake = intakeOverride.getNormalInterface();
        Strongback.executor().register(simulator, Priority.HIGH);
    }

    public void createBuddyClimb() {
        if (!Config.buddyClimb.present) {
            buddyClimb = new MockBuddyClimb();
            debug("Buddy climb not present, using a mock buddy climb instead");
            return;
        }

        Solenoid buddyClimbSolenoid = pcm.singleSolenoid(Config.buddyClimb.solenoidPort, 0.1, 0.1);
        buddyClimb = new BuddyClimbImpl(buddyClimbSolenoid);
    }

    public void createZipChain() {
        if (!Config.zipChain.present) {
            zipChain = new MockZipChain();
            debug("Zip chain not present, using a mock zip chain instead");
            return;
        }
        Motor motor = MotorFactory.getZipChainMotor();
        zipChain = new ZipChainImpl(motor);
    }

    public void createClimberDeployer() {
        if (!Config.climberDeployer.present) {
            climberDeployer = new MockClimberDeployer();
            debug("Climber deployer not present, using a mock climber deployer instead");
            return;
        }
        Motor motor = MotorFactory.getClimberDeployerMotor();
        motor.setPosition(0);
        // TODO: Test and work out correct timings.
        Solenoid solenoid = pcm.singleSolenoid(Config.climberDeployer.solenoidPort, 0.1, 0.1);
        climberDeployer = new ClimberDeployerImpl(motor, solenoid);
    }

    public void createColourWheel() {
        if (!Config.colourWheel.present) {
            colourWheel = new MockColourWheel();
            debug("Colour Sensor not present, using a mock colour sensor instead");
            return;
        }
        Motor motor = MotorFactory.getColourWheelMotor();
        Solenoid colourWheelSolenoid =
                pcm.singleSolenoid(Config.colourWheel.solenoidPort, 0.1, 0.1);
        ColorSensorV3 colourSensor = new ColorSensorV3(i2cPort);
        ColorMatch colourMatcher = new ColorMatch();
        // Adding colours to the colourMatcher
        colourMatcher.addColorMatch(Config.colourWheel.target.blue);
        colourMatcher.addColorMatch(Config.colourWheel.target.green);
        colourMatcher.addColorMatch(Config.colourWheel.target.red);
        colourMatcher.addColorMatch(Config.colourWheel.target.yellow);
        colourMatcher.addColorMatch(Config.colourWheel.target.white);

        colourWheel = new ColourWheelImpl(motor, colourWheelSolenoid, new Supplier<WheelColour>() {
            @Override
            public WheelColour get() {
                ColorMatchResult match = colourMatcher.matchClosestColor(colourSensor.getColor());
                WheelColour sensedColour = WheelColour.UNKNOWN;
                if (match.color == Config.colourWheel.target.blue) {
                    sensedColour = WheelColour.BLUE;
                } else if (match.color == Config.colourWheel.target.red) {
                    sensedColour = WheelColour.RED;
                } else if (match.color == Config.colourWheel.target.green) {
                    sensedColour = WheelColour.GREEN;
                } else if (match.color == Config.colourWheel.target.yellow) {
                    sensedColour = WheelColour.YELLOW;
                }
                return sensedColour;
            }
        }, ledStrip, clock);
        Strongback.executor().register(colourWheel, Priority.HIGH);
    }

    public void createLEDStrip() {
        if (!Config.ledStrip.present) {
            ledStrip = new MockLEDStrip();
            debug("LED Strip not present, using a mock LED Strip instead.");
            return;
        }
        ledStrip = new LEDStripImpl(Config.ledStrip.pwmPort, Config.ledStrip.numLEDs);
    }

    public void createMonitor() {
        monitor = new MonitorImpl();
    }

    public void updateIdleLED() {
        ledStrip.setIdle();
    }

    public void setLEDColour(LEDColour c) {
        ledStrip.setColour(c);
    }

    public void setLEDFinalCountdown(double time) {
        ledStrip.setProgressColour(LEDColour.GREEN, LEDColour.RED,
                time / Config.ledStrip.countdown);
    }

    @SuppressWarnings("resource")
    public void createLoader() {
        if (!Config.loader.present) {
            loader = hwLoader = new MockLoader();
            debug("Created a mock loader!");
            return;
        }

        Motor spinnerMotor = MotorFactory.getLoaderSpinnerMotor();
        Motor loaderPassthroughMotor = MotorFactory.getLoaderPassthroughMotor();
        Solenoid blockerSolenoid = pcm.singleSolenoid(Config.loader.solenoidPort, 0.1, 0.1);
        // The ball sensors are connected to the DIO ports on the rio.
        DigitalInput inBallSensor = new DigitalInput(Config.loader.ballDetector.inPort);
        DigitalInput outBallSensor = new DigitalInput(Config.loader.ballDetector.outPort);
        DigitalInput passthroughInnerSensor =
                new DigitalInput(Config.loader.ballDetector.passthroughInnerPort);
        DigitalInput passthroughOuterSensor =
                new DigitalInput(Config.loader.ballDetector.passthroughOuterPort);
        DigitalInput loaderSensor = new DigitalInput(Config.loader.ballDetector.loaderPort);
        BooleanSupplier loaderInSensor = () -> !inBallSensor.get();
        BooleanSupplier loaderOutSensor = () -> !outBallSensor.get();
        BooleanSupplier loaderFullSensor = () -> loaderSensor.get();
        BooleanSupplier passthroughInnerBallSensor = () -> passthroughInnerSensor.get();
        BooleanSupplier passthroughOuterBallSensor = () -> passthroughOuterSensor.get();
        loader = hwLoader =
                new LoaderImpl(spinnerMotor, loaderPassthroughMotor, blockerSolenoid,
                        loaderInSensor,
                        loaderOutSensor, passthroughInnerBallSensor, passthroughOuterBallSensor,
                        loaderFullSensor, ledStrip);
        Strongback.executor().register(loader, Priority.LOW);

    }

    public void createLoaderOverride() {
        // Setup the diagBox so that it can take control.
        MockLoader simulator = new MockLoader(); // Nothing to simulate, use the mock
        MockLoader mock = new MockLoader();
        loaderOverride = new OverridableSubsystem<Loader>("loader", Loader.class, loader, simulator,
                mock);
        // Plumb accessing the lift through the override.
        loader = loaderOverride.getNormalInterface();
    }

    public void createShooter() {
        if (!Config.shooter.present) {
            shooter = hwShooter = new MockFlywheelShooter();
            debug("Created a mock shooter!");
            return;
        }

        Solenoid hoodSolenoid = pcm.singleSolenoid(Config.shooter.solenoidPort, 0.1, 0.1);
        Motor motor = MotorFactory.getShooterMotor(clock);

        shooter = hwShooter = new FlywheelShooter(motor, hoodSolenoid);
    }

    public void createShooterOverride() {
        // Setup the diagBox so that it can take control.
        MockFlywheelShooter simulator = new MockFlywheelShooter(); // Nothing to simulate, use a
                                                                   // mock instead.
        MockFlywheelShooter mock = new MockFlywheelShooter();
        shooterOverride = new OverridableSubsystem<Shooter>("shooter", Shooter.class, shooter,
                simulator, mock);
        // Plumb accessing the shooter through the override.
        shooter = shooterOverride.getNormalInterface();
    }

    /**
     * Create the Pneumatics Control Module (PCM) subsystem.
     */
    public void createPneumatics() {
        if (!Config.pcm.present) {
            pcm = Mock.pneumaticsModule(Config.pcm.canId);
            debug("Created a mock compressor");
            return;
        }
        pcm = Hardware.pneumaticsModule(Config.pcm.canId, PneumaticsModuleType.CTREPCM);
    }

    public void createVision() {
        if (!Config.vision.present) {
            vision = new MockVision();
            debug("Created a mock vision subsystem");
            return;
        }
        try {
            jevois = new JevoisImpl();
            vision = new VisionImpl(jevois, location, clock, Config.vision.hMin,
                    Config.vision.sMin,
                    Config.vision.vMin, Config.vision.hMax, Config.vision.sMax, Config.vision.vMax);
        } catch (IOException e) {
            exception("Unable to create an instance of the jevois camera", e);
            e.printStackTrace();
            vision = new MockVision();
        }
    }

    @Override
    public String getName() {
        return "Subsystems";
    }
}
