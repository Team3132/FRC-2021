package frc.robot.subsystems;



import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.interfaces.BuddyClimb;
import frc.robot.lib.Subsystem;
import frc.robot.lib.chart.Chart;
import org.strongback.components.FeatureSolenoid;
import org.strongback.components.FeatureSolenoid.Mode;
import org.strongback.components.Solenoid;

public class BuddyClimbImpl extends Subsystem implements BuddyClimb {
    private FeatureSolenoid solenoid;

    public BuddyClimbImpl(Solenoid solenoid) {
        super("BuddyClimb");
        this.solenoid = new FeatureSolenoid(solenoid);

        Chart.register(() -> solenoid.isExtended(), "%s/extended", name);
        Chart.register(() -> solenoid.isRetracted(), "%s/retracted", name);
    }

    @Override
    public BuddyClimb setEnabled(Mode mode) {
        solenoid.setMode(mode);
        return this; // Allows chaining of calls.
    }

    @Override
    public boolean isInPosition() {
        return solenoid.isInPosition();
    }

    /**
     * Update the operator console with the status of the buddy climb subsystem.
     */
    @Override
    public void updateDashboard() {
        SmartDashboard.putString("Buddy climb position", solenoid.toString());
    }
}
