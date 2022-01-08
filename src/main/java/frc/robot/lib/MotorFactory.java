package frc.robot.lib;



import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import frc.robot.Config;
import frc.robot.lib.chart.Chart;
import frc.robot.lib.log.Log;
import org.strongback.components.Clock;
import org.strongback.components.Motor;
import org.strongback.components.NetworkTableHelperImpl;
import org.strongback.components.PIDF;
import org.strongback.hardware.Hardware;
import org.strongback.hardware.HardwareSparkMAX;
import org.strongback.hardware.HardwareTalonSRX;

public class MotorFactory {

    public static Motor getDriveMotor(boolean leftMotor, Clock clock) {
        leftMotor = leftMotor ^ Config.drivebase.swapLeftRight;
        int[] canIds = leftMotor ? Config.drivebase.canIdsLeftWithEncoders
                : Config.drivebase.canIdsRightWithEncoders;

        switch (Config.drivebase.motorControllerType) {
            case Config.motorController.sparkMAX: {
                HardwareSparkMAX spark = getSparkMAX("drive", canIds, leftMotor, NeutralMode.Brake,
                        Config.drivebase.pidf);
                spark.setScale(Config.encoder.SparkMAXTicks, Config.drivebase.gearboxRatio,
                        Config.drivebase.metresPerRev);
                spark.setSensorPhase(Config.drivebase.sensorPhase);

                /*
                 * Setup Current Limiting
                 */
                if (Config.drivebase.currentLimiting) {
                    // Limit to 35 Amps when current exceeds 40 amps for 100ms
                    spark.setSmartCurrentLimit(Config.drivebase.contCurrent,
                            Config.drivebase.contCurrent);
                    spark.setSecondaryCurrentLimit(Config.drivebase.peakCurrent);
                }
                return spark;
            }

            default:
                Log.error("MotorFactory",
                        "Invalid drive motor controller '%s'. Please use 'TalonSRX' or 'SparkMAX'. Using TalonSRX.",
                        Config.drivebase.motorControllerType);
                // Falling through to TalonSRX.

            case Config.motorController.talonSRX:
                HardwareTalonSRX talon = getTalon("drive", canIds, leftMotor, NeutralMode.Brake,
                        Config.drivebase.pidf); // don't invert output
                talon.setScale(Config.encoder.falconTicks, Config.drivebase.gearboxRatio,
                        Config.drivebase.metresPerRev);
                talon.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 10);
                talon.setSensorPhase(Config.drivebase.sensorPhase);
                talon.configClosedloopRamp(Config.drivebase.rampRate, 10);
                talon.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 10, 10);

                /*
                 * Setup Current Limiting
                 */
                if (Config.drivebase.currentLimiting) {
                    talon.configContinuousCurrentLimit(Config.drivebase.contCurrent, 0); // limit to
                                                                                         // 35 Amps
                                                                                         // when
                                                                                         // current
                                                                                         // exceeds
                                                                                         // 40 amps
                                                                                         // for
                    // 100ms
                    talon.configPeakCurrentLimit(Config.drivebase.peakCurrent, 0);
                    talon.configPeakCurrentDuration(100, 0);
                    talon.enableCurrentLimit(true);
                }
                return talon;
        }
    }

    public static HardwareSparkMAX getIntakeMotor() {
        HardwareSparkMAX motor = getSparkMAX("intake", Config.intake.canID, true, NeutralMode.Brake,
                Config.intake.pidf);
        motor.setScale(Config.encoder.SparkMAXTicks, Config.intake.gearboxRatio);
        motor.setClosedLoopRampRate(0.5);
        return motor;
    }

    public static HardwareTalonSRX getZipChainMotor() {
        HardwareTalonSRX motor = getTalon("zipchain motor", Config.zipChain.canID, false,
                NeutralMode.Brake, Config.zipChain.pidf);
        motor.configClosedloopRamp(0.5, 10);
        return motor;
    }

    public static HardwareTalonSRX getClimberDeployerMotor() {
        HardwareTalonSRX motor =
                getTalon("climber deployer motor", Config.climberDeployer.canID, true,
                        NeutralMode.Brake, Config.climberDeployer.pidf);
        motor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 10);
        motor.setScale(Config.encoder.versaIntegratedTicks, Config.climberDeployer.gearboxRatio,
                Config.climberDeployer.metresPerRev);
        motor.configContinuousCurrentLimit(Config.climberDeployer.currentLimit, 10);
        motor.configClosedloopRamp(0.5, 10);
        motor.configForwardSoftLimitThreshold(18737, 10);
        motor.configForwardSoftLimitEnable(true, 10);
        motor.configReverseSoftLimitThreshold(0, 10);
        motor.configReverseSoftLimitEnable(true, 10);
        motor.setPosition(0);
        return motor;
    }

    public static HardwareTalonSRX getColourWheelMotor() {
        HardwareTalonSRX motor = getTalon("colour", Config.colourWheel.canID, true,
                NeutralMode.Brake, Config.colourWheel.pidf);
        motor.configClosedloopRamp(.25, 10);
        return motor;
    }

    public static HardwareTalonSRX getLoaderSpinnerMotor() {
        HardwareTalonSRX motor =
                getTalon("loader spinner", Config.loader.spinner.canID, false, NeutralMode.Brake,
                        Config.loader.spinner.pidf);
        // In sensor (beambreak) for loader
        motor.configForwardLimitSwitchSource(LimitSwitchSource.Deactivated,
                LimitSwitchNormal.NormallyOpen, 10);
        // Out sensor (beambreak) for loader
        motor.configReverseLimitSwitchSource(LimitSwitchSource.Deactivated,
                LimitSwitchNormal.NormallyOpen, 10);
        motor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 10);
        motor.setScale(Config.encoder.versaIntegratedTicks, Config.loader.spinner.gearboxRatio);
        motor.configClosedloopRamp(0, 10);
        return motor;
    }

    public static HardwareSparkMAX getLoaderPassthroughMotor() {
        HardwareSparkMAX motor =
                getSparkMAX("loader passthrough", Config.loader.passthrough.canID, true,
                        NeutralMode.Brake, Config.loader.passthrough.pidf);
        return motor;
    }

    public static HardwareTalonSRX getShooterMotor(Clock clock) {
        HardwareTalonSRX motor =
                getTalon("shooter", Config.shooter.canIds, false, NeutralMode.Coast,
                        Config.shooter.pidf);
        motor.setSensorPhase(true);
        motor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 10);
        motor.setScale(Config.encoder.s4tTicks, Config.shooter.gearboxRatio);
        motor.selectProfileSlot(0, 0);

        motor.configClosedloopRamp(1, 10);
        motor.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 10, 10);

        motor.configContinuousCurrentLimit(Config.shooter.contCurrent, 10);
        motor.configPeakCurrentLimit(Config.shooter.peakCurrent, 10);
        motor.configPeakCurrentDuration(Config.shooter.peakCurrentDuration, 10);

        return motor;
    }

    /**
     * Code to allow us to log output current per Spark MAX and set up followers so that
     * it appears as a single motor but can be an arbitary number of motors configured
     * in the per robot configuration.
     * 
     * @param name the name to use for saving the PIDF values in the network tables.
     * @param canIDs list of canIDs. First entry is the leader and the rest follow it.
     * @param invert change the direction.
     * @param mode what to do when the the speed is set to zero.
     * @param pidf the P, I, D & F values to use.
     * @param log for registration of the current reporting.
     * @return the leader HardwareTalonSRX
     */
    private static HardwareTalonSRX getTalon(String name, int[] canIDs, boolean invert,
            NeutralMode mode,
            PIDF pidf) {
        HardwareTalonSRX leader = Hardware.Motors.talonSRX(abs(canIDs[0]), invert, mode);
        Chart.register(() -> leader.getSupplyCurrent(), "Talons/%d/Current", canIDs[0]);
        leader.configContinuousCurrentLimit(
                Config.motorController.currentLimit.defaultContinuousAmps, 10);
        leader.configPeakCurrentLimit(Config.motorController.currentLimit.defaultPeakAmps, 10);
        leader.enableCurrentLimit(true);
        TunableMotor.tuneMotor(leader, pidf, new NetworkTableHelperImpl(name));

        for (int n = 1; n < canIDs.length; n++) {
            boolean shouldInvert = invert;
            if (canIDs[n] < 0)
                shouldInvert = !shouldInvert;
            HardwareTalonSRX follower =
                    Hardware.Motors.talonSRX(abs(canIDs[n]), shouldInvert, mode);
            follower.getHWTalon().follow(leader.getHWTalon());
            Chart.register(() -> follower.getSupplyCurrent(), "Talons/%d/Current", canIDs[n]);
        }
        return leader;
    }

    /**
     * Code to allow us to log output current for a single talon.
     * 
     * @param name the name to use for saving the PIDF values in the network tables.
     * @param canID CAN ID for this motor controller. Must be unique.
     * @param invert change the direction.
     * @param mode what to do when the the speed is set to zero.
     * @param pidf the P, I, D & F values to use.
     * @param log for registration of the current reporting.
     * @return the HardwareTalonSRX motor controller.
     */
    private static HardwareTalonSRX getTalon(String name, int canID, boolean invert,
            NeutralMode mode, PIDF pidf) {
        Log.debug("MotorFactory", "%s: " + canID, "talon");
        int[] canIDs = new int[1];
        canIDs[0] = canID;
        return getTalon(name, canIDs, invert, mode, pidf);
    }

    /**
     * Code to allow us to log output current per Spark MAX and set up followers so that
     * it appears as a single motor but can be an arbitary number of motors configured
     * in the per robot configuration.
     * 
     * @param name the name to use for saving the PIDF values in the network tables.
     * @param canIDs list of canIDs. First entry is the leader and the rest follow it.
     * @param invert change the direction.
     * @param mode what to do when the the speed is set to zero.
     * @param pidf the P, I, D & F values to use.
     * @param log for registration of the current reporting.
     * @return the leader SparkMAX
     */

    private static HardwareSparkMAX getSparkMAX(String name, int[] canIDs, boolean invert,
            NeutralMode mode, PIDF pidf) {
        HardwareSparkMAX leader =
                Hardware.Motors.sparkMAX(abs(canIDs[0]), MotorType.kBrushless, invert);
        leader.setIdleMode(mode == NeutralMode.Brake ? IdleMode.kBrake : IdleMode.kCoast);
        Chart.register(() -> leader.getOutputCurrent(), "SparkMAX/%d/Current", canIDs[0]);
        leader.setSmartCurrentLimit(Config.motorController.currentLimit.defaultContinuousAmps, 10);
        leader.setSecondaryCurrentLimit(Config.motorController.currentLimit.defaultPeakAmps, 10);
        TunableMotor.tuneMotor(leader, pidf, new NetworkTableHelperImpl(name));

        for (int n = 1; n < canIDs.length; n++) {
            boolean shouldInvert = invert;
            if (canIDs[n] < 0)
                shouldInvert = !shouldInvert;
            HardwareSparkMAX follower =
                    Hardware.Motors.sparkMAX(abs(canIDs[n]), MotorType.kBrushless, shouldInvert);
            follower.getHWSpark().follow(leader.getHWSpark());
            Chart.register(() -> follower.getOutputCurrent(), "SparkMAX/%d/Current", canIDs[n]);
        }
        return leader;
    }

    /**
     * Code to allow us to log output current for a single Spark MAX.
     * 
     * @param name the name to use for saving the PIDF values in the network tables.
     * @param canID CAN ID for this motor controller. Must be unique.
     * @param invert change the direction.
     * @param mode what to do when the the speed is set to zero.
     * @param pidf the P, I, D & F values to use.
     * @param log for registration of the current reporting.
     * @return the HardwareSparkMAX motor controller.
     */
    private static HardwareSparkMAX getSparkMAX(String name, int canID, boolean invert,
            NeutralMode mode, PIDF pidf) {
        Log.debug("MotorFactory", "%s: " + canID, " spark max");
        int[] canIDs = new int[1];
        canIDs[0] = canID;
        return getSparkMAX(name, canIDs, invert, mode, pidf);
    }

    private static int abs(int value) {
        return value >= 0 ? value : -value;
    }
}
