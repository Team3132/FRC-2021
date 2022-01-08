package frc.robot.mock;



import frc.robot.interfaces.Shooter;
import frc.robot.lib.Subsystem;
import org.strongback.components.Solenoid.Position;

public class MockFlywheelShooter extends Subsystem implements Shooter {

    private double targetRPS = 0;

    public MockFlywheelShooter() {
        super("MockShooter");
    }

    @Override
    public Shooter setTargetRPS(double rps) {
        targetRPS = rps;
        return this;
    }

    @Override
    public boolean isAtTargetSpeed() {
        return true;
    }

    @Override
    public double getTargetRPS() {
        return targetRPS;
    }

    @Override
    public Shooter setHoodPosition(Position position) {
        return this;
    }

    @Override
    public boolean hoodIsInPosition() {
        return true;
    }
}
