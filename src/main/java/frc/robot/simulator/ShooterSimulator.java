
package frc.robot.simulator;



import frc.robot.interfaces.Shooter;
import frc.robot.lib.MovementSimulator;
import frc.robot.lib.Subsystem;
import org.strongback.components.Solenoid.Position;

/**
 * Very basic intake simulator used for unit testing.
 * Does not do gravity/friction etc.
 */
public class ShooterSimulator extends Subsystem implements Shooter {

    private final double kMaxSpeed = 180; // degrees/sec
    private final double kMaxAccel = 200; // degrees/sec/sec
    private final double kMinAngle = 0;
    private final double kMaxAngle = 45;
    private final double kMovementTolerance = 1; // How close before it's classed as being in
                                                 // position.
    private double targetRPS = 0;
    private double shooterTime = 0;
    private MovementSimulator arm = new MovementSimulator("hood", kMaxSpeed, kMaxAccel, kMinAngle,
            kMaxAngle, kMovementTolerance);

    public ShooterSimulator() {
        super("ShooterSimulator");
    }

    @Override
    public Shooter setTargetRPS(double rps) {
        this.targetRPS = rps;
        this.shooterTime = System.currentTimeMillis();
        return this;
    }

    @Override
    public double getTargetRPS() {
        return targetRPS;
    }

    @Override
    public boolean isAtTargetSpeed() {
        if ((System.currentTimeMillis() - this.shooterTime) < 1000) {
            return true;
        }
        return false;
    }

    @Override
    public Shooter setHoodPosition(Position position) {
        arm.setTargetPos(position == Position.EXTENDED ? kMaxAngle : kMinAngle);
        return this;
    }

    @Override
    public boolean hoodIsInPosition() {
        return arm.isInPosition();
    }
}
