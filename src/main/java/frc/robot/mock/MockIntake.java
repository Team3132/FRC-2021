package frc.robot.mock;



import frc.robot.interfaces.Intake;
import frc.robot.lib.Subsystem;
import org.strongback.components.Solenoid.Position;


/**
 * Mock subsystem responsible for the intake
 */
public class MockIntake extends Subsystem implements Intake {
    private double output = 0;
    private Position position = Position.RETRACTED;

    public MockIntake() {
        super("MockIntake");
    }

    @Override
    public Intake setPosition(Position position) {
        return this;
    }

    @Override
    public boolean isInPosition() {
        return true;
    }

    @Override
    public Intake setTargetRPS(double rps) {
        output = rps;
        return this;
    }

    @Override
    public double getTargetRPS() {
        return output;
    }

    @Override
    public boolean isExtended() {
        return position == Position.EXTENDED;
    }

    @Override
    public boolean isRetracted() {
        return position == Position.RETRACTED;
    }
}
