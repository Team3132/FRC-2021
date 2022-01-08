package frc.robot.subsystems;

import static org.junit.jupiter.api.Assertions.assertEquals;

import frc.robot.interfaces.Shooter;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.strongback.mock.Mock;
import org.strongback.mock.MockMotor;
import org.strongback.mock.MockSolenoid;


public class TestShooter {
    MockMotor shooterMotor;
    MockSolenoid shooterSolenoid;
    Shooter shooter;

    public static final int SHOOTER_TARGET_SPEED_RPS = 100;

    @BeforeEach
    public void setUp() {
        shooterMotor = Mock.stoppedMotor();
        shooterSolenoid = Mock.Solenoids.singleSolenoid(0);
        shooter = new FlywheelShooter(shooterMotor, shooterSolenoid);
    }

    @Test
    public void testShooterEnable() {
        shooter.enable();
        assertEquals(0, shooterMotor.getSpeed(), 0.01);
    }

    @Test
    public void testShooterDisable() {
        shooter.disable();
        assertEquals(0, shooterMotor.getSpeed(), 0.01);
    }

    @Test
    public void testShooterSetTargetSpeed() {
        shooter.setTargetRPS(SHOOTER_TARGET_SPEED_RPS);
        assertEquals(SHOOTER_TARGET_SPEED_RPS, shooterMotor.getSpeed(), 0.1);
    }

    @Test
    public void testShooterSetTargetSpeedAndDisable() {
        shooter.setTargetRPS(SHOOTER_TARGET_SPEED_RPS);
        shooter.disable();
        assertEquals(0, shooterMotor.getSpeed(), 0.01);
    }
}
