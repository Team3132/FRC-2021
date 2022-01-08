/**
 * Sequences for doing most actions on the robot.
 * 
 * If you add a new sequence, add it to allSequences at the end of this file.
 */
package frc.robot.controller;

import static frc.robot.lib.PoseHelper.createPose2d;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.Config;
import frc.robot.controller.Sequence.SequenceBuilder;
import frc.robot.lib.LEDColour;
import frc.robot.lib.WheelColour;
import java.util.List;

/**
 * Control sequences for most robot operations.
 */
public class Sequences {

    /**
     * Do nothing sequence.
     */
    public static Sequence getEmptySequence() {
        if (emptySeq == null) {
            emptySeq = new SequenceBuilder("empty").build();
        }
        return emptySeq;
    }

    private static Sequence emptySeq = null;

    private static Sequence exclusiveSeq = null;

    /**
     * The first sequence run in the autonomous period.
     */
    public static Sequence getStartSequence() {
        if (startSeq == null) {
            startSeq = new SequenceBuilder("start").build();
        }
        // startbuilder.add().doArcadeVelocityDrive();
        return startSeq;
    }

    private static Sequence startSeq = null;

    /**
     * Returns the sequence to reset the robot. Used to stop ejecting etc. The lift
     * is at intake height, the intake is stowed, all motors are off.
     * 
     * @return
     */
    public static Sequence getResetSequence() {
        if (resetSeq == null) {
            SequenceBuilder builder = new SequenceBuilder("empty");
            builder.then().doDefaultDrive();
            resetSeq = builder.build();
        }
        return resetSeq;
    }

    private static Sequence resetSeq = null;

    /**
     * Turn to face the driver station wall and then switch back to arcade.
     */
    public static Sequence turnToWall() {
        if (driveTestSeq == null) {
            SequenceBuilder builder = new SequenceBuilder("turn to wall");
            builder.then().doTurnToHeading(180);
            // builder.add().doArcadeDrive();
            driveTestSeq = builder.build();
        }
        return driveTestSeq;
    }

    private static Sequence driveTestSeq = null;

    /**
     * Drive to a point on the field, relative to the starting point.
     * 
     * @param angle the final angle (relative to the field) in degrees.
     */
    public static Sequence getDriveToWaypointSequence(double x, double y, double angle) {
        Pose2d start = new Pose2d();
        Pose2d end = createPose2d(x, y, angle);
        SequenceBuilder builder = new SequenceBuilder(String.format("drive to %s", end));
        builder.then().driveRelativeWaypoints(start, List.of(), end, true);
        driveToWaypointSeq = builder.build();
        return driveToWaypointSeq;
    }

    private static Sequence driveToWaypointSeq = null;

    public static Sequence startSlowDriveForward() {
        SequenceBuilder builder = new SequenceBuilder("Slow drive forward");
        builder.then().setDrivebasePower(0.3);
        return builder.build();
    }

    public static Sequence setDrivebaseToArcade() {
        SequenceBuilder builder = new SequenceBuilder("Arcade Drive Routine");
        builder.then().doArcadeDrive();
        return builder.build();
    }

    public static Sequence setDrivebaseToDefault() {
        SequenceBuilder builder = new SequenceBuilder("Default Drive Routine");
        builder.then().doDefaultDrive();
        return builder.build();
    }

    /**
     * Extends the intake and then runs the motor to intake the cargo.
     * 
     * @return
     */

    public static Sequence startIntaking() {
        SequenceBuilder builder = new SequenceBuilder("Start intaking");
        // Wait for the intake to extend before turning motor
        builder.then().deployIntake().blockShooter();
        // builder.add().setIntakeMotorOutput(INTAKE_MOTOR_OUTPUT)
        builder.then().setIntakeRPS(Config.intake.targetRPS)
                .setSpinnerRPS(Config.loader.spinner.speed.intakingRPS)
                .setPassthroughDutyCycle(Config.loader.passthrough.motorDutyCycle);
        // builder.add().waitForBalls(5);
        // Reverse to eject excess > 5 balls to avoid penalty
        /*
         * builder.add().setIntakeRPS(-INTAKE_TARGET_RPS);
         * builder.add().setDelayDelta(1); builder.add().setIntakeRPS(0)
         * .setSpinnerRPS(0) .setPassthroughDutyCycle(0);
         */
        return builder.build();
    }

    public static Sequence reverseIntaking() {
        SequenceBuilder builder = new SequenceBuilder("Reverse intaking");
        // Wait for the intake to extend before turning motor
        builder.then().deployIntake().blockShooter();
        builder.then().setIntakeRPS(-Config.intake.targetRPS)
                .setPassthroughDutyCycle(-Config.loader.passthrough.motorDutyCycle);
        return builder.build();
    }

    public static Sequence stopIntaking() {
        SequenceBuilder builder = new SequenceBuilder("Stop intaking");
        builder.then().setIntakeRPS(0);
        // Let passthrough run for 0.25s longer to get all balls through
        builder.then().setDelayDelta(0.25);
        builder.then().setPassthroughDutyCycle(0);
        builder.then().setSpinnerRPS(Config.loader.spinner.speed.intakingRPS * 0.05);
        builder.createInterruptState();

        return builder.build();
    }

    public static Sequence raiseIntake() {
        SequenceBuilder builder = new SequenceBuilder("Raise intake");
        builder.then().stowIntake();
        return builder.build();
    }

    /**
     * Start Test Loader Sequence
     * 
     */
    public static Sequence startLoaderTest() {
        SequenceBuilder builder = new SequenceBuilder("Start Loader Test");
        builder.then().setPassthroughDutyCycle(0.5);
        builder.then().setSpinnerRPS(0.3);
        builder.then().setDelayDelta(10);
        builder.then().setPassthroughDutyCycle(0);
        builder.then().setSpinnerRPS(0);
        builder.then().setDelayDelta(5);
        // Switch/Extend Occurs here
        builder.then().setSpinnerRPS(0.2);
        builder.then().setDelayDelta(5);
        builder.then().setSpinnerRPS(0);

        return builder.build();
    }

    // Testing methods
    public static Sequence startIntakingOnly() {
        SequenceBuilder builder = new SequenceBuilder("Start Intaking only");
        builder.then().deployIntake();
        builder.then().setIntakeRPS(Config.intake.targetRPS).deployIntake();
        return builder.build();
    }

    public static Sequence stopIntakingOnly() {
        SequenceBuilder builder = new SequenceBuilder("Stop intaking only");
        builder.then().setIntakeRPS(0);
        builder.then().stowIntake();
        return builder.build();
    }

    // This is to test the Loader system
    public static Sequence startLoader() {
        SequenceBuilder builder = new SequenceBuilder("Start loader");
        builder.then().setSpinnerRPS(Config.loader.spinner.speed.intakingRPS);
        return builder.build();
    }

    // Reverse Loader
    public static Sequence reverseIntakingAndLoader() {
        SequenceBuilder builder = new SequenceBuilder("Reverse intake and loader");
        // Wait for the intake to extend before turning motor
        builder.then().deployIntake().blockShooter();
        builder.then().setIntakeRPS(-Config.intake.targetRPS)
                .setPassthroughDutyCycle(-Config.loader.passthrough.motorDutyCycle);
        builder.then().setSpinnerRPS(-Config.loader.spinner.speed.intakingRPS);
        return builder.build();
    }

    /**
     * Spin up the shooter and position the hood to get ready for a far shot. To
     * spin down use a button mapped to stopShooting()
     */
    public static Sequence spinUpCloseShot(double speed) {
        SequenceBuilder builder = new SequenceBuilder("spinUpCloseShot" + speed);
        builder.then().setShooterRPS(speed).extendShooterHood();
        return builder.build();
    }

    /**
     * Spin up the shooter and position the hood to get ready for a far shot. To
     * spin down use a button mapped to stopShooting()
     */
    public static Sequence spinUpFarShot(double speed) {
        SequenceBuilder builder = new SequenceBuilder("spinUpFarShot" + speed);
        builder.then().setShooterRPS(speed).retractShooterHood();
        return builder.build();
    }

    /**
     * Shoot the balls using whatever hood position and shooter speed is currently
     * set
     * 
     * WARNING: This sequence will never finish if the shooter speed is currently
     * set to zero It sets the LEDs to purple if this happens
     */
    public static Sequence startShooting() {
        SequenceBuilder builder = new SequenceBuilder("Start Shooting");
        // Another sequence should have set the shooter speed and hood position already

        // Wait for the shooter wheel to settle.
        builder.then().waitForShooter();
        // Let the balls out of the loader and into the shooter.
        builder.then().unblockShooter();
        // Start the loader to push the balls.
        builder.then().setSpinnerRPS(Config.loader.spinner.speed.shootingRPS);
        // Wait 1 seconds before spinning passthrough
        builder.then().setDelayDelta(1);
        // Spin passthrough
        builder.then().setPassthroughDutyCycle(Config.loader.passthrough.motorDutyCycle);


        /*
         * // Wait for all of the balls to leave. builder.add().waitForBalls(0); // Turn
         * off everything. builder.add().setShooterRPS(0)
         * .setLoaderPassthroughMotorOutput(0) .setLoaderSpinnerMotorRPS(0)
         * .blockShooter();
         */
        return builder.build();
    }

    public static Sequence stopShooting() {
        SequenceBuilder builder = new SequenceBuilder("Stop shooting");
        // Turn off everything.
        builder.then().setShooterRPS(0).setPassthroughDutyCycle(0).setSpinnerRPS(0).blockShooter();
        return builder.build();
    }

    public static Sequence startDriveByVision() {
        SequenceBuilder builder = new SequenceBuilder("Start drive by vision");
        builder.then().doVisionDrive();
        return builder.build();
    }

    /**
     * Aim the robot if a vision target is visible.
     * Quits as soon as it's aimed.
     */
    public static Sequence visionAim() {
        SequenceBuilder builder = new SequenceBuilder("Vision aim");
        builder.then().doVisionAim();
        // Will move on when the vision target is visible, otherwise it will
        // give up quickly allowing the rest of the auto routine to run.
        return builder.build();
    }

    /**
     * Assist the driver by taking over the steering when a vision
     * target is visible.
     */
    public static Sequence visionAssist() {
        SequenceBuilder builder = new SequenceBuilder("Vision assist");
        builder.then().doVisionAssist();
        // Stay driving in vision assist mode.
        return builder.build();
    }

    public static Sequence extendZipChain() {
        SequenceBuilder builder = new SequenceBuilder("Extend zip chain");
        builder.then().setZipChainDutyCycle(Config.zipChain.dutyCycle);
        return builder.build();
    }

    public static Sequence retractZipChain() {
        SequenceBuilder builder = new SequenceBuilder("Retract zip chain");
        builder.then().setZipChainDutyCycle(-Config.zipChain.dutyCycle);
        return builder.build();
    }

    public static Sequence stopZipChain() {
        SequenceBuilder builder = new SequenceBuilder("Stop zip chain");
        builder.then().setZipChainDutyCycle(0);
        return builder.build();
    }

    public static Sequence extendClimberDeployer() {
        SequenceBuilder builder = new SequenceBuilder("Extend climber deployer");
        builder.then().deployIntake();
        builder.then().releaseClimberRatchet();
        builder.then().setClimberDeployerDutyCycle(Config.climberDeployer.dutyCycle);
        return builder.build();
    }

    public static Sequence retractClimberDeployer() {
        SequenceBuilder builder = new SequenceBuilder("Retract climber deployer");
        builder.then().setClimberDeployerDutyCycle(-Config.climberDeployer.dutyCycle);
        return builder.build();
    }

    public static Sequence stopClimberDeployer() {
        SequenceBuilder builder = new SequenceBuilder("Stop climber deployer");
        builder.then().setClimberDeployerDutyCycle(0);
        builder.then().applyClimberRatchet();
        return builder.build();
    }

    public static Sequence releaseClimberDeployer() {
        SequenceBuilder builder = new SequenceBuilder("Release climber deployer");
        builder.then().deployIntake();
        builder.then().releaseClimber();
        return builder.build();
    }

    public static Sequence stowClimberDeployer() {
        SequenceBuilder builder = new SequenceBuilder("Stow climber deployer");
        builder.then().holdClimber();
        return builder.build();
    }

    public static Sequence startColourWheelRotational() {
        SequenceBuilder builder = new SequenceBuilder("Start rotational control");
        builder.then().extendColourWheel();
        builder.then().colourWheelRotational();
        builder.then().retractColourWheel();
        return builder.build();
    }

    public static Sequence startColourWheelPositional(WheelColour colour) {
        SequenceBuilder builder = new SequenceBuilder("Start positional control");
        builder.then().extendColourWheel();
        builder.then().startColourWheelPositional(colour);
        builder.then().retractColourWheel();
        return builder.build();
    }

    public static Sequence stopColourWheel() {
        SequenceBuilder builder = new SequenceBuilder("Stop colour wheel spinner");
        builder.then().stopColourWheel();
        builder.then().retractColourWheel();
        return builder.build();
    }

    public static Sequence colourWheelAnticlockwise() {
        SequenceBuilder builder = new SequenceBuilder("Moving colour wheel anticlockwise");
        builder.then().extendColourWheel();
        builder.then().colourWheelAnticlockwise();
        return builder.build();
    }

    public static Sequence colourWheelClockwise() {
        SequenceBuilder builder = new SequenceBuilder("Moving colour wheel clockwise");
        builder.then().extendColourWheel();
        builder.then().colourWheelClockwise();
        return builder.build();
    }

    public static Sequence enableClimbMode() {
        SequenceBuilder builder = new SequenceBuilder("enable climb mode");
        builder.then().deployIntake();
        builder.then().enableClimbMode();
        builder.then().doArcadeClimb();
        builder.createInterruptState();
        return builder.build();
    }

    public static Sequence enableDriveMode() {
        SequenceBuilder builder = new SequenceBuilder("enable drive mode");
        builder.then().applyClimberRatchet();
        builder.then().enableDriveMode();
        builder.then().doArcadeDrive();
        builder.createInterruptState();
        return builder.build();
    }

    public static Sequence applyClimberRatchet() {
        SequenceBuilder builder = new SequenceBuilder("Apply climber ratchet");
        builder.then().applyClimberRatchet();
        builder.createInterruptState();
        return builder.build();
    }

    public static Sequence releaseClimberRatchet() {
        SequenceBuilder builder = new SequenceBuilder("Release climber ratchet");
        builder.then().deployIntake();
        builder.then().releaseClimberRatchet();
        return builder.build();
    }

    public static Sequence deployBuddyClimb() {
        SequenceBuilder builder = new SequenceBuilder("deploy buddy climb attachment");
        builder.then().deployBuddyClimb();
        return builder.build();
    }

    public static Sequence stowBuddyClimb() {
        SequenceBuilder builder = new SequenceBuilder("stow buddy climb attachment");
        builder.then().stowBuddyClimb();
        return builder.build();
    }

    public static Sequence setLEDColour(LEDColour c) {
        SequenceBuilder builder = new SequenceBuilder("set LEDS to " + c);
        builder.then().setColour(c);
        return builder.build();
    }

    // For testing. Needs to be at the end of the file.
    public static Sequence[] allSequences = new Sequence[] {

            getEmptySequence(), getStartSequence(), getResetSequence(), startIntaking(),
            stopIntaking(),
            startShooting(), stopShooting(), startIntakingOnly(), stopIntakingOnly(), startLoader(),
            deployBuddyClimb(), stowBuddyClimb(), enableClimbMode(), enableDriveMode(),
            visionAssist(), visionAim(),};
}
