
package frc.robot.controller;

import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import frc.robot.controller.Sequence.SequenceBuilder;
import org.junit.jupiter.api.Test;

public class TestSequence {
    @Test
    public void testDoesConflict() {
        SequenceBuilder sb = new SequenceBuilder("");
        Sequence a = sb.build();
        // a={}
        assertFalse(a.doesConflict(a));

        sb = new SequenceBuilder("");
        sb.then().setIntakeRPS(1).setClimberDeployerDutyCycle(1);
        Sequence b = sb.build();
        // b={intake,climber}
        assertTrue(b.doesConflict(b));

        // a={}, b={intake,climber}
        assertFalse(a.doesConflict(b));
        assertFalse(b.doesConflict(a));

        sb = new SequenceBuilder("");
        sb.then().colourWheelAnticlockwise();
        Sequence c = sb.build();
        // b={intake,climber} c={colourwheel}
        assertFalse(b.doesConflict(c));
        assertFalse(c.doesConflict(b));

        sb.then().setIntakeRPS(2);
        c = sb.build();
        // b={intake,climber} c={colourwheel,intake}
        assertTrue(b.doesConflict(c));
        assertTrue(c.doesConflict(b));
    }
}
