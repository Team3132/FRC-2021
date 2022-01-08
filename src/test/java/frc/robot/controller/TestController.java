package frc.robot.controller;

import static org.junit.jupiter.api.Assertions.assertTrue;

import frc.robot.Config;
import frc.robot.controller.Sequence.SequenceBuilder;
import frc.robot.lib.LEDColour;
import frc.robot.lib.WheelColour;
import frc.robot.mock.MockBuddyClimb;
import frc.robot.mock.MockClimberDeployer;
import frc.robot.mock.MockColourWheel;
import frc.robot.mock.MockDrivebase;
import frc.robot.mock.MockFlywheelShooter;
import frc.robot.mock.MockLEDStrip;
import frc.robot.mock.MockLoader;
import frc.robot.mock.MockLocation;
import frc.robot.mock.MockZipChain;
import frc.robot.simulator.IntakeSimulator;
import frc.robot.subsystems.Subsystems;
import java.util.Random;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.strongback.mock.MockClock;
import org.strongback.mock.MockPneumaticsModule;

/**
 * Test cases for the Controller and the Sequences
 * 
 * Mocks out almost everything so that no hardware is needed.
 */
public class TestController {
    // Use 10 ms steps between executions.
    private final long ktestStepMs = 10;
    private final long kRandomSeed = 123456;
    private final double kMaxWaitTimeSeconds = 4;
    private MockClock clock;
    private Subsystems subsystems;
    // Store direct access to the simulators so the simulator-only
    // methods can be called.
    @SuppressWarnings("unused")
    private IntakeSimulator intake;
    private TestHelper test;
    // The bit that is being tested under test.
    private Controller exec;

    /**
     * Setup fields used by this test.
     */
    @BeforeEach
    public void setUp() {
        System.out.println("\n******************************");
        clock = new MockClock();
        subsystems = new Subsystems(clock);

        subsystems.intake = intake = new IntakeSimulator();
        subsystems.pcm = new MockPneumaticsModule();
        subsystems.drivebase = new MockDrivebase();
        subsystems.loader = new MockLoader();
        subsystems.shooter = new MockFlywheelShooter();
        subsystems.location = new MockLocation();
        subsystems.colourWheel = new MockColourWheel();
        subsystems.zipChain = new MockZipChain();
        subsystems.climberDeployer = new MockClimberDeployer();
        subsystems.buddyClimb = new MockBuddyClimb();
        subsystems.ledStrip = new MockLEDStrip();

        exec = new Controller(subsystems, () -> WheelColour.UNKNOWN, () -> LEDColour.GREEN);

        test = new TestHelper(() -> {
            clock.incrementByMilliseconds(ktestStepMs);
            // long now = clock.currentTimeInMillis();
            // System.out.printf("==== Cycle starting at time %.03fms ====\n", now/1000.);
            subsystems.intake.execute(clock.currentTimeInMillis());
            return clock.currentTime();
        }, () -> {
            System.out.println(subsystems.intake.toString());
        });

        // If the controller dies we should fail the test.
        test.registerSafetyFunc(
                () -> assertTrue(exec.isAlive(),
                        "The controller has died failing the test. A stack trace should be above"));
    }

    /**
     * Example test.
     * 
     * Use this as a template when designing new tests.
     */
    /*
     * @Test
     * public void testExampleForCopying() {
     * // Update the println statement with your test name.
     * System.out.println("testExampleForCopying");
     * 
     * // Setup initial subsystem state. Lift starts at the bottom and the intake stowed.
     * // Set what you need for the test here. Once the subsystems have been set up,
     * // then the test will move on to the next thenSet(), thenAssert() or thenWait()
     * // statement.
     * test.thenAssert(outtakeOpen(true), intakeMotorPower(0),
     * liftHeight(LiftPosition.INTAKE_POSITION));
     * 
     * // Run the intaking sequence.
     * test.thenSet(sequence(Sequences.getStartIntakingSequence()));
     * 
     * // Then assert that the robot subsystems have eventually been put into the expected states.
     * // Having them in one .thenAssert() means that they will all have to be true at once.
     * // In this case the intake should be in the narrow configuration, the outtake should be open,
     * // the intake motor should have full power forward and the lift should be in the intake
     * position.
     * test.thenAssert(outtakeOpen(true), intakeMotorPower(1),
     * liftHeight(LiftPosition.INTAKE_POSITION));
     * 
     * // The test can then be told to do nothing for some number of seconds.
     * test.thenWait(0.5);
     * 
     * // Then you can tell the robot that a cube was detected in the outtake, which
     * // may make the sequence move on to the next state.
     * // NB, this sensor has been removed, so hasCube() is no longer an option.
     * test.thenSet(hasCube(true));
     * 
     * // And then make the test check that the subsystems are updated to be in the correct state.
     * test.thenAssert(outtakeOpen(false), intakeMotorPower(0),
     * liftHeight(LiftPosition.INTAKE_POSITION));
     * 
     * // Walk through setting the states and asserting that the robot eventually
     * // moves through the required state.
     * // This line executes the steps set up above. Note adding println statements
     * // will print out the statements when the test is setup, not as it moves through
     * // the states.
     * assertTrue(test.run());
     * }
     */

    /**
     * Test for using the onInterrupt part of the start/stop intaking sequences
     * 
     * First we try the normal case
     * Then we try interrupting before the passthrough and loader are set to stop
     */
    @Test
    public void testStopIntakingInterrupt() {
        System.out.println("testInterrupt");
        SequenceBuilder builder = new SequenceBuilder("conflicting sequence");
        builder.then().setIntakeRPS(0); // This will conflict with the stopIntaking sequence.
        Sequence conflictingSeq = builder.build();

        // Setup initial state of the robot.
        test.thenAssert(intakeMotorRPS(0), passthrougMotorPower(0), spinnerMotorRPS(0));

        // Run the intaking sequence.
        test.thenSet(sequence(Sequences.startIntaking()));

        // Then assert that the robot is actually intaking
        test.thenAssert(intakeMotorRPS(Config.intake.targetRPS),
                passthrougMotorPower(Config.loader.passthrough.motorDutyCycle),
                spinnerMotorRPS(Config.loader.spinner.speed.intakingRPS));

        // Run the stop intaking sequence
        test.thenSet(sequence(Sequences.stopIntaking()));

        // This is the normal case. We are expecting everything to stop normally
        test.thenAssert(intakeMotorRPS(0), passthrougMotorPower(0),
                spinnerMotorRPS(Config.loader.spinner.speed.intakingRPS * 0.05));

        /* Interrupted case */

        // Run the intaking sequence.
        test.thenSet(sequence(Sequences.startIntaking()));

        // Then assert that the robot is actually intaking
        test.thenAssert(intakeMotorRPS(Config.intake.targetRPS),
                passthrougMotorPower(Config.loader.passthrough.motorDutyCycle),
                spinnerMotorRPS(Config.loader.spinner.speed.intakingRPS));

        // Run the stop intaking sequence
        test.thenSet(sequence(Sequences.stopIntaking()));

        // Let the controller move into the wait between stopping the intake and stopping everything
        // else
        test.thenAssert(intakeMotorRPS(0),
                passthrougMotorPower(Config.loader.passthrough.motorDutyCycle),
                spinnerMotorRPS(Config.loader.spinner.speed.intakingRPS));

        // Interrupt the sequence
        test.thenSet(sequence(conflictingSeq));

        // Everything should still get set to 0
        test.thenAssert(intakeMotorRPS(0), passthrougMotorPower(0),
                spinnerMotorRPS(Config.loader.spinner.speed.intakingRPS * 0.05));

        assertTrue(test.run());
    }

    /**
     * Pretends to be a crazy operator that keeps changing their mind.
     * 
     * This allows it to check that the robot is always in a safe configuration
     * as the safety checker is checking the state of the robot every time.
     * It sleeps for a random amount of time between desired state changes to
     * allow the robot to get either fully into the new state, or part way.
     * 
     * There is no checking that the robot is actually doing anything useful
     * here, only that it doesn't hurt itself.
     */
    @Test
    public void testCrazyOperatorFuzzTest() {
        System.out.println("testCrazyOperatorFuzzTest");
        // Seed the random number generator so that the same
        // random numbers are generated every time.
        Random generator = new Random(kRandomSeed);

        // Build a large number random steps.
        for (int i = 0; i < 10 /* 0 */; i++) {
            // Ask for a random desired state.
            test.thenSet(sequence(getRandomDesiredSequence(generator)));
            test.thenWait(generator.nextDouble() * kMaxWaitTimeSeconds);
        }

        // Walk through setting the states and asserting that the robot eventually
        // moves through the required state.
        assertTrue(test.run());
    }


    // Helpers only from this point onwards.

    private Sequence getRandomDesiredSequence(Random generator) {
        return Sequences.allSequences[generator.nextInt(Sequences.allSequences.length)];
    }


    /**
     * Either sets the intake motor rps, OR asserts the rps the motor has
     * been set to, depending if it's in a thenSet() or a thenAssert().
     * 
     * @param rps to set/expect to/from the motor.
     * @return a setter or asserter object to pass to the TestHelper.
     */
    private StateSetterOrAsserter intakeMotorRPS(double rps) {
        return new StateSetterOrAsserter() {
            @Override
            public String name() {
                return String.format("IntakeMotorPower(%.1f)", rps);
            }

            @Override
            public void setState() {
                subsystems.intake.setTargetRPS(rps);
            }

            @Override
            public void assertState() throws AssertionError {
                if (Math.abs(subsystems.intake.getTargetRPS() - rps) > 0.1) {
                    throw new AssertionError(
                            "Expected intake motor to have rps " + rps + " but it is "
                                    + subsystems.intake.getTargetRPS());
                }
            }
        };
    }

    /**
     * Either sets the spinner motor rps, OR asserts the rps the motor has
     * been set to, depending if it's in a thenSet() or a thenAssert().
     * 
     * @param rps to set/expect to/from the motor.
     * @return a setter or asserter object to pass to the TestHelper.
     */
    private StateSetterOrAsserter spinnerMotorRPS(double rps) {
        return new StateSetterOrAsserter() {
            @Override
            public String name() {
                return String.format("spinnerMotorPower(%.1f)", rps);
            }

            @Override
            public void setState() {
                subsystems.loader.setTargetSpinnerRPS(rps);
            }

            @Override
            public void assertState() throws AssertionError {
                if (Math.abs(subsystems.loader.getTargetSpinnerRPS() - rps) > 0.1) {
                    throw new AssertionError(
                            "Expected spinner motor to have rps " + rps + " but it is "
                                    + subsystems.loader.getTargetSpinnerRPS());
                }
            }
        };
    }

    /**
     * Either sets the passthrough motor power, OR asserts the power the motor has
     * been set to, depending if it's in a thenSet() or a thenAssert().
     * 
     * @param power to set/expect to/from the motor.
     * @return a setter or asserter object to pass to the TestHelper.
     */
    private StateSetterOrAsserter passthrougMotorPower(double pwr) {
        return new StateSetterOrAsserter() {
            @Override
            public String name() {
                return String.format("PassthroughMotorPower(%.1f)", pwr);
            }

            @Override
            public void setState() {
                subsystems.loader.setTargetPassthroughDutyCycle(pwr);
            }

            @Override
            public void assertState() throws AssertionError {
                if (Math.abs(subsystems.loader.getTargetPassthroughDutyCycle() - pwr) > 0.1) {
                    throw new AssertionError(
                            "Expected passthrough motor to have pwr " + pwr + " but it is "
                                    + subsystems.loader.getTargetPassthroughDutyCycle());
                }
            }
        };
    }


    /**
     * Tells the Controller to run the desired sequence. Only makes sense in a
     * thenSet(), not a thenAssert().
     * 
     * @param sequence the sequence to execute.
     * @return a setter or asserter object to pass to the TestHelper.
     */
    private StateSetterOrAsserter sequence(Sequence sequence) {
        return new StateSetterOrAsserter() {
            @Override
            public String name() {
                return String.format("Sequence(%s)", sequence.getName());
            }

            @Override
            public void setState() {
                exec.run(sequence);
            }

            @Override
            public void assertState() throws AssertionError {
                throw new AssertionError("Invalid usage of sequence() in thenAssert()");
            }
        };
    }
}
