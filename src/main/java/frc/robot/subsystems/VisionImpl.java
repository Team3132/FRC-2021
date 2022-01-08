package frc.robot.subsystems;

import static frc.robot.lib.LowPassFilter.filterValues;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Config;
import frc.robot.interfaces.Jevois;
import frc.robot.interfaces.Location;
import frc.robot.interfaces.Vision;
import frc.robot.lib.Subsystem;
import frc.robot.lib.chart.Chart;
import java.io.IOException;
import org.strongback.components.Clock;

public class VisionImpl extends Subsystem implements Vision, Runnable {
    private Jevois jevois;
    private Location location;
    private Clock clock;
    private double visionHMin, visionSMin, visionVMin;
    private double visionHMax, visionSMax, visionVMax;
    private TargetDetails lastSeenTarget = new TargetDetails(); // the target we are shooting into
    private TargetDetails lastSeenOpponentTarget = new TargetDetails(); // opponent target
    private boolean connected = false;

    public VisionImpl(Jevois jevois, Location location, Clock clock,
            double visionHMin, double visionSMin, double visionVMin, double visionHMax,
            double visionSMax,
            double visionVMax) {
        super("Vision");
        this.jevois = jevois;
        this.location = location;
        this.clock = clock;
        this.visionHMin = visionHMin;
        this.visionSMin = visionSMin;
        this.visionVMin = visionVMin;
        this.visionHMax = visionHMax;
        this.visionSMax = visionSMax;
        this.visionVMax = visionVMax;

        Chart.register(() -> isConnected(), "%s/connected", name);
        Chart.register(() -> lastSeenTarget.location.getX(), "%s/curX", name);
        Chart.register(() -> lastSeenTarget.location.getY(), "%s/curY", name);
        Chart.register(() -> lastSeenTarget.location.getRotation().getDegrees(), "%s/heading",
                name);
        // Chart.register(() -> clock.currentTime() - lastSeenTarget.seenAtSec, "%s/seenAt", name)
        Chart.register(() -> lastSeenTarget.imageTimestamp, "%s/seenAtSec", name);
        Chart.register(() -> lastSeenTarget.targetFound, "%s/targetFound", name);
        Chart.register(() -> lastSeenTarget.distance, "%s/distance", name);
        Chart.register(() -> lastSeenTarget.angle, "%s/angle", name);

        Chart.register(() -> lastSeenOpponentTarget.location.getX(), "%s/opponent/curX", name);
        Chart.register(() -> lastSeenOpponentTarget.location.getY(), "%s/opponent/curY", name);
        Chart.register(() -> lastSeenOpponentTarget.location.getRotation().getDegrees(),
                "%s/opponent/heading", name);
        Chart.register(() -> lastSeenOpponentTarget.targetFound, "%s/opponent/targetFound", name);
        // Chart.register(() -> clock.currentTime() - lastSeenOpponentTarget.seenAtSec,
        // "%s/opponent/seenAt", name)
        Chart.register(() -> lastSeenOpponentTarget.imageTimestamp, "%s/opponent/seenAtSec", name);

        // Start reading from the Jevois camera.
        (new Thread(this)).start();
    }

    /**
     * Return the details of the last target seen. Let the caller decide if the data
     * is too old/stale.
     */
    @Override
    public synchronized TargetDetails getTargetDetails() {
        return lastSeenTarget;
    }

    /**
     * Main loop. Runs in its own thread so it can block.
     */
    @Override
    public void run() {
        try {
            while (true) {
                debug("Waiting for the camera server to start up");
                Thread.sleep(5000);
                doProcessing();
            }
        } catch (InterruptedException e) {
            warning("InterruptedException, likely shutting down");
        }
        connected = false;
    }

    /**
     * Try to connect to and process images from a camera.
     * 
     * @throws InterruptedException
     */
    public void doProcessing() throws InterruptedException {
        debug("Starting to read from Jevois camera\n");
        try {
            // Attempt to detect if there is a camera plugged in. It will throw an exception
            // if not.
            debug(jevois.issueCommand("info"));
            connected = true;

            // Update the HSV filter ranges from the config values.

            jevois.issueCommand(String.format("setHSVMin %.0f %.0f %.0f", visionHMin,
                    visionSMin, visionVMin));
            jevois.issueCommand(String.format("setHSVMax %.0f %.0f %.0f", visionHMax,
                    visionSMax, visionVMax));
            /*
             * jevois.issueCommand(String.format("position %.0f %.0f %.0f",
             * Config.vision.cameraHeight,
             * Config.vision.cameraPitch,
             * Config.vision.cameraRoll));
             */

            while (true) {
                processLine(jevois.readLine());
            }
        } catch (IOException e) {
            exception("Failed to read from jevois, aborting vision processing\n", e);
            connected = false;
        }
    }

    /**
     * Parses a line from the vision and calculates the target position on the field
     * based on where the robot was at that time. No line is read when a target
     * isn't seen.
     * 
     * Example line: D3 1.0 true 12.23 20.2 9.1 FIRST
     * 
     * Line format: D3 <imageAge> <found> <distance> <angle> <skew> FIRST
     * 
     * Where: D3: static string to indicate that this is a found vision target.
     * imageAge: time in seconds since image taken
     * found: boolean for if goal was detected
     * distance: horizontal distance from goal in metres
     * angle: degrees
     * skew: degrees
     * FIRST: static string.
     */
    private void processLine(String line) {
        // Split the line on whitespace.
        // "D3 timestamp found distance angle FIRST"
        String[] parts = line.split("\\s+");

        if (!parts[0].equals("D3")) {
            info("Ignoring non-vision target line: %s", line);
            return;
        }
        // Logger.debug("Vision::processLine(%s)\n", line);
        TargetDetails newTarget = new TargetDetails();
        newTarget.targetFound = Boolean.parseBoolean(parts[2]);

        if (!Boolean.parseBoolean(parts[2]))
            return; // If target is not detected, ignore the line

        // A target was seen, update the TargetDetails in case it's asked for.
        // Fill in a new TargetDetails so it can be returned if asked for and it won't
        // change as the caller uses it.

        newTarget.imageTimestamp = clock.currentTime() - Double.parseDouble(parts[1]);
        newTarget.distance = Double.parseDouble(parts[3]);
        newTarget.angle = -Double.parseDouble(parts[4]);
        newTarget.skew = Double.parseDouble(parts[5]);

        Pose2d robotPosition = location.getHistoricalPose(newTarget.imageTimestamp);
        newTarget.location =
                robotPosition.plus(new Transform2d(Config.vision.cameraPosition.getTranslation(),
                        Config.vision.cameraPosition.getRotation()));
        newTarget.location = newTarget.location.transformBy(
                new Transform2d(new Translation2d(0.0, 0.0), newTarget.location.getRotation()
                        .plus(new Rotation2d(Math.toRadians(newTarget.angle - newTarget.skew)))));

        // Logger.debug("Location set.");
        // newTarget.location.heading += newTarget.angle

        // NOTE: We are assuming that the robot starts the match facing the opponent driverstation
        // (the wall that our vision target is on).
        // If we see our goal (-90 <= heading <= 90):
        if (true) { // (heading >= -90 && heading <= 90) {
            // Low pass filter
            // We reject the last seen target if it is older than <timeOffset> seconds

            if (lastSeenTarget.isValid(clock.currentTime())) {
                newTarget.location = new Pose2d(
                        filterValues(newTarget.location.getX(), lastSeenTarget.location.getX(),
                                Config.vision.goalLowPassAlpha),
                        filterValues(newTarget.location.getY(), lastSeenTarget.location.getY(),
                                Config.vision.goalLowPassAlpha),
                        new Rotation2d(Math.toRadians(
                                filterValues(newTarget.location.getRotation().getDegrees(),
                                        lastSeenTarget.location.getRotation().getDegrees(),
                                        Config.vision.goalLowPassAlpha))));
            }

            synchronized (this) {
                lastSeenTarget = newTarget;
            }
            // log.sub("Vision: Updated target %s", lastSeenTarget);

        } else { // If we see the opponent goal:
            // Low pass filter
            // We reject the last seen opponent target if it is older than <timeOffset> seconds
            if (lastSeenOpponentTarget.isValid(clock.currentTime())) {
                newTarget.location = new Pose2d(filterValues(newTarget.location.getX(),
                        lastSeenOpponentTarget.location.getX(), Config.vision.goalLowPassAlpha),
                        filterValues(newTarget.location.getY(),
                                lastSeenOpponentTarget.location.getY(),
                                Config.vision.goalLowPassAlpha),
                        new Rotation2d(Math.toRadians(
                                filterValues(newTarget.location.getRotation().getDegrees(),
                                        lastSeenOpponentTarget.location.getRotation().getDegrees(),
                                        Config.vision.goalLowPassAlpha))));
            }

            synchronized (this) {
                lastSeenOpponentTarget = newTarget;
            }
            // log.sub("Vision: Updated target %s", lastSeenOpponentTarget);
        }
    }

    @Override
    public void updateDashboard() {
        TargetDetails mostRecentTarget;
        String target;
        if (lastSeenTarget.imageTimestamp > lastSeenOpponentTarget.imageTimestamp) {
            mostRecentTarget = lastSeenTarget;
            target = "Our Target";
        } else {
            mostRecentTarget = lastSeenOpponentTarget;
            target = "Opponent Target";
        }

        double lockAgeSec = (clock.currentTime() - mostRecentTarget.imageTimestamp);
        double angle = 0, distance = 0;
        if (mostRecentTarget.isValid(clock.currentTime())) {
            Pose2d robotPos = location.getCurrentPose();
            angle = -robotPos.relativeTo(mostRecentTarget.location).getRotation().getDegrees();
            distance = robotPos.getTranslation()
                    .getDistance(mostRecentTarget.location.getTranslation());
        }

        SmartDashboard.putString("Vision Target", target);
        SmartDashboard.putBoolean("Vision camera connected", connected);
        SmartDashboard.putNumber("Vision distance to target", distance);
        SmartDashboard.putNumber("Vision lockAgeSec", lockAgeSec);
        SmartDashboard.putNumber("Vision angle", angle);
        SmartDashboard.putBoolean("Vision targetFound", mostRecentTarget.targetFound);
        SmartDashboard.putBoolean("Vision is Valid", mostRecentTarget.isValid(clock.currentTime()));
        SmartDashboard.putNumber("Vision Skew",
                mostRecentTarget.location.getRotation().getDegrees());

    }

    /**
     * @return hasConnection returns the current status of the connection to the
     *         external vision processor
     */
    @Override
    public boolean isConnected() {
        return connected;
    }
}
