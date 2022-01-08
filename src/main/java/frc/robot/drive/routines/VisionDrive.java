package frc.robot.drive.routines;



import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Config;
import frc.robot.interfaces.DriveTelemetry;
import frc.robot.interfaces.Location;
import frc.robot.interfaces.Vision;
import frc.robot.interfaces.Vision.TargetDetails;
import frc.robot.lib.chart.Chart;
import org.strongback.components.Clock;

/**
 * Drive to a point a configured distance in front of the goal and
 * turn the robot to the target.
 */
public class VisionDrive extends AutoDriveBase {
    private Vision vision;
    private Location location;
    private Clock clock;

    public VisionDrive(DriveTelemetry telemetry,
            Vision vision, Location location, Clock clock) {
        super("visionAssist", telemetry,
                clock);
        this.vision = vision;
        this.location = location;
        this.clock = clock;
        Chart.register(() -> getVisionWaypoint().getX(), "Drive/visionDrive/waypoint/x");
        Chart.register(() -> getVisionWaypoint().getY(), "Drive/visionDrive/waypoint/y");
    }

    @Override
    public double getTargetSpeed() {
        if (vision == null || !vision.isConnected())
            return 0;
        TargetDetails details = vision.getTargetDetails();

        if (!details.isValid(clock.currentTime()))
            return 0;
        // We have a recent target position relative to the robot starting position.
        Pose2d current = location.getCurrentPose();
        double distanceBeforeGoal = Config.drivebase.routine.visionDrive.distanceBeforeGoal;
        double speed = Config.drivebase.routine.visionDrive.speedScale
                * Math.max(0,
                        current.getTranslation().getDistance(details.location.getTranslation())
                                - distanceBeforeGoal);
        // Cap it so that the robot quickly gets to max speed.
        return Math.min(speed, Config.drivebase.routine.visionDrive.maxSpeed);
    }

    @Override
    public double getTargetTurn() {
        if (vision == null || !vision.isConnected())
            return 0;
        TargetDetails details = vision.getTargetDetails();
        if (!details.isValid(clock.currentTime()))
            return 0;
        // We have a recent target position relative to the robot starting position.
        Pose2d current = location.getCurrentPose();
        Pose2d waypoint = getVisionWaypoint();
        return Config.drivebase.routine.visionDrive.angleScale
                * -current.relativeTo(waypoint).getRotation().getDegrees();
    }

    public Pose2d getVisionWaypoint() {
        if (vision == null || !vision.isConnected()) {
            return new Pose2d(0, 0, new Rotation2d(0));
        }
        TargetDetails details = vision.getTargetDetails();
        Pose2d current = location.getCurrentPose();
        if (current.getTranslation().getDistance(details.location
                .getTranslation()) > Config.drivebase.routine.visionDrive.splineMinDistanceMetres) {
            return details.location
                    .plus(new Transform2d(new Translation2d(
                            -current.getTranslation().getDistance(details.location.getTranslation())
                                    * Config.drivebase.routine.visionDrive.waypointDistanceScale,
                            0), new Rotation2d(0)));
        } else {
            return details.location;
        }
    }

    @Override
    public boolean hasFinished() {
        // Stop when the target speed is close to zero which is when
        // the robot is close to where it should be or it can't see
        // a vision target.
        return Math.abs(getTargetSpeed()) < 0.1 && Math.abs(getTargetTurn()) < 0.1;
    }

}
