package frc.robot.mock;



import frc.robot.interfaces.ColourWheel;
import frc.robot.interfaces.ColourWheel.ColourAction.ColourWheelType;
import frc.robot.lib.Subsystem;
import frc.robot.lib.WheelColour;
import org.strongback.components.Solenoid.Position;

public class MockColourWheel extends Subsystem implements ColourWheel {
    private ColourAction action = new ColourAction(ColourWheelType.NONE, WheelColour.UNKNOWN);
    private Position position = Position.RETRACTED;

    public MockColourWheel() {
        super("MockColourWheel");
    }

    @Override
    public ColourWheel setDesiredAction(ColourAction action) {
        this.action = action;
        return this;
    }

    @Override
    public ColourAction getDesiredAction() {
        return action;
    }

    @Override
    public boolean isFinished() {
        return true;
    }

    @Override
    public boolean isArmExtended() {
        return position == Position.EXTENDED;
    }

    @Override
    public boolean isArmRetracted() {
        return position == Position.RETRACTED;
    }

    @Override
    public void setArmPosition(Position position) {
        this.position = position;
    }

    @Override
    public boolean armIsInPosition() {
        return true;
    }
}
