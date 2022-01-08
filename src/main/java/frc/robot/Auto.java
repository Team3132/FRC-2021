package frc.robot;

import static frc.robot.lib.PoseHelper.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.controller.Controller;
import frc.robot.controller.Sequence;
import frc.robot.controller.Sequence.SequenceBuilder;
import frc.robot.controller.Sequences;
import frc.robot.interfaces.LogHelper;
import frc.robot.lib.AutoPaths;
import java.io.IOException;
import java.util.List;

/**
 * Handles auto routine selection.
 * 
 * Auto routines should be defined in Sequences.java
 */
public class Auto implements LogHelper {
    private SendableChooser<Sequence> autoProgram = new SendableChooser<Sequence>();
    private SendableChooser<Integer> initBallSelector = new SendableChooser<Integer>();

    public Auto() {
        addAutoSequences();
        initAutoChooser();
        addBallOptions();
        initBallChooser();
    }

    private void addBallOptions() {
        initBallSelector.addOption("0", Integer.valueOf(0));
        initBallSelector.addOption("1", Integer.valueOf(1));
        initBallSelector.addOption("2", Integer.valueOf(2));
        initBallSelector.setDefaultOption("3", Integer.valueOf(3));
        initBallSelector.addOption("4", Integer.valueOf(4));
        initBallSelector.addOption("5", Integer.valueOf(5));
    }

    public void executedSelectedSequence(Controller controller) {
        Sequence seq = autoProgram.getSelected();
        info("Starting selected auto program %s", seq.getName());
        controller.run(seq);
    }

    public int getSelectedBallAmount() {
        Integer numBalls = initBallSelector.getSelected();
        info("Starting with %s balls", numBalls);
        return numBalls;
    }

    private void addAutoSequences() {
        autoProgram.setDefaultOption("Nothing", Sequences.getEmptySequence());
        // autoProgram.addOption("Drive forward 10in",
        // Sequences.getDriveToWaypointSequence(10 * Config.constants.inchesToMetres, 0, 0));
        // addDriveTestSequence();
        // addDriveTestSplineSequence();
        // addDriveTestUSequence();
        // addPathweaverTest();
        // addBasicShootIntakeDriveShootSequence();
        // addBasicFilmSequence();
        add8BallAutoSequence();
        add5BallOpponentTrench();
        addShootAndDrive();
        // addAutoNavBarrelRacing();
        // addAutoNavBounce();
    }

    @SuppressWarnings("unused")
    private void addBasicFilmSequence() {

        SequenceBuilder builder = new SequenceBuilder("Basic film routine");

        builder.then().setCurrentPostion(Config.field.startingFilmPosGoal);

        builder.then().deployIntake();
        builder.appendSequence(Sequences.startIntaking());
        builder.then().driveRelativeWaypoints(Config.field.startingFilmPosGoal,
                List.of(Config.field.startingFilmPosGoal.getTranslation()
                        .plus(new Translation2d(-1, 0))),
                intakeAt(Config.field.redevousFilmBalls, -70), false);
        // builder.then().driveRelativeWaypoints(Config.field.startingFilmPosGoal,
        // List.of(), Config.field.redevousFilmBalls, false);
        // builder.then().

        builder.then().setDelayDelta(1);
        builder.appendSequence(Sequences.stopIntaking());
        builder.appendSequence(Sequences.spinUpCloseShot(Config.shooter.speed.closeRPS));
        // builder.then().driveRelativeWaypoints(Config.field.redevousFilmBalls,
        // List.of(), Config.field.shootingFilmPos, true);
        builder.then().driveRelativeWaypoints(intakeAt(Config.field.redevousFilmBalls, -70),
                List.of(Config.field.shootingFilmPos.getTranslation()
                        .plus(new Translation2d(-3, 0))),
                Config.field.shootingFilmPos, true);

        builder.appendSequence(Sequences.startShooting());

        builder.then().setDelayDelta(5);

        builder.appendSequence(Sequences.stopShooting());

        // builder.then().doVisionAim();
        builder.then().driveRelativeWaypoints(Config.field.shootingFilmPos, List.of(),
                Config.field.endingFilmPosGoal,
                false);

        autoProgram.addOption("Basic Film Sequence", builder.build());
    }

    @SuppressWarnings("unused")
    private void addDriveTestSequence() {
        SequenceBuilder builder = new SequenceBuilder("Drive backwards 2m then forwards 2m");
        // Go backwards 2m
        Pose2d start1 = createPose2d(Config.field.fieldCentreX, Config.field.fieldCentreY, 0);
        Pose2d end1 = createPose2d(Config.field.fieldCentreX - 2, Config.field.fieldCentreY, 0);
        builder.then().driveRelativeWaypoints(start1, List.of(), end1, false); // backwards.
        // Go forwards 2m
        Pose2d start = createPose2d(Config.field.fieldCentreX - 2, Config.field.fieldCentreY, 0);
        Pose2d end = createPose2d(Config.field.fieldCentreX, Config.field.fieldCentreY, 0);
        builder.then().driveRelativeWaypoints(start, List.of(), end, true);
        autoProgram.addOption("Drive test 2m", builder.build());
    }

    @SuppressWarnings("unused")
    private void addDriveTestSplineSequence() {
        SequenceBuilder builder =
                new SequenceBuilder("Drive backwards 2mx1m then forward 2mx-1m");
        Pose2d start1 = createPose2d(Config.field.fieldCentreX, Config.field.fieldCentreY, 0);
        Pose2d end1 = createPose2d(Config.field.fieldCentreX - 2, Config.field.fieldCentreY - 1, 0);
        builder.then().driveRelativeWaypoints(start1, List.of(), end1, false); // backwards.
        // Go backwards 2m
        Pose2d start =
                createPose2d(Config.field.fieldCentreX - 2, Config.field.fieldCentreY - 1, 0);
        Pose2d end = createPose2d(Config.field.fieldCentreX, Config.field.fieldCentreY, 0);
        builder.then().driveRelativeWaypoints(start, List.of(), end, true);
        autoProgram.addOption("Drive test spline 2mx1m", builder.build());
    }

    @SuppressWarnings("unused")
    private void addDriveTestUSequence() {
        SequenceBuilder builder = new SequenceBuilder("Drive u-turn 2m");
        Pose2d start1 = createPose2d(Config.field.fieldCentreX, Config.field.fieldCentreY, 0);
        Pose2d end1 = createPose2d(Config.field.fieldCentreX, Config.field.fieldCentreY + 2, 180);
        builder.then().driveRelativeWaypoints(start1, List.of(), end1, false); // backwards.
        builder.then().setDelayDelta(1);
        Pose2d start = createPose2d(Config.field.fieldCentreX, Config.field.fieldCentreY + 2, 180);
        Pose2d end = createPose2d(Config.field.fieldCentreX, Config.field.fieldCentreY, 0);
        builder.then().driveRelativeWaypoints(start, List.of(), end, true);
        builder.then().setDelayDelta(1);

        builder.then().driveRelativeWaypoints(start1, List.of(), end1, false); // backwards.
        builder.then().setDelayDelta(1);
        builder.then().driveRelativeWaypoints(start, List.of(), end, true);
        builder.then().setDelayDelta(1);

        builder.then().driveRelativeWaypoints(start1, List.of(), end1, false); // backwards.
        builder.then().setDelayDelta(1);
        builder.then().driveRelativeWaypoints(start, List.of(), end, true);
        builder.then().setDelayDelta(1);

        builder.then().driveRelativeWaypoints(start1, List.of(), end1, false); // backwards.
        builder.then().setDelayDelta(1);
        builder.then().driveRelativeWaypoints(start, List.of(), end, true);
        builder.then().setDelayDelta(1);

        builder.then().driveRelativeWaypoints(start1, List.of(), end1, false); // backwards.
        builder.then().setDelayDelta(1);
        builder.then().driveRelativeWaypoints(start, List.of(), end, true);
        builder.then().setDelayDelta(1);
        autoProgram.addOption("Drive u-turn 2m", builder.build());
    }

    @SuppressWarnings("unused")
    private void addBasicShootIntakeDriveShootSequence() {
        SequenceBuilder builder = new SequenceBuilder("Basic shoot intake drive shoot");

        builder.then("Set pos: auto line").setCurrentPostion(Config.field.autoLineGoal);

        // Start shooting
        builder.appendSequence(Sequences.spinUpFarShot(Config.shooter.speed.autoLineRPS));
        builder.appendSequence(Sequences.startShooting());
        builder.then().setDelayDelta(2);

        // Start intaking
        builder.appendSequence(Sequences.startIntaking());

        // Drive backwards to pick up the three balls.
        // Drive to third ball via the first ball
        builder.then("Driving to third trench ball").driveRelativeWaypoints(
                Config.field.autoLineGoal,
                List.of(Config.field.allianceTrenchFirstBall.getTranslation()
                        .plus(new Translation2d(Config.robot.halfRobotLength, 0.5))),
                intakeAt(Config.field.allianceTrenchThirdBall, 0), false); // backwards.

        // Stop intaking
        builder.appendSequence(Sequences.stopIntaking());

        // Go forwards 2m to shoot.
        Pose2d end = approachPose(Config.field.allianceTrenchThirdBall, 2, 0);

        builder.then("Drive 2m").driveRelativeWaypoints(
                intakeAt(Config.field.allianceTrenchThirdBall, 0), List.of(),
                end, true);

        builder.then().doVisionAim();
        // Shoot the balls.
        builder.appendSequence(Sequences.spinUpFarShot(Config.shooter.speed.farTargetRPS));
        builder.appendSequence(Sequences.startShooting());
        builder.then("Shooting").setDelayDelta(2);

        builder.appendSequence(Sequences.stopShooting());

        autoProgram.addOption("Basic shoot intake drive shoot", builder.build());
    }

    private void addShootAndDrive() {

        SequenceBuilder builder = new SequenceBuilder("Shoot and Drive");

        builder.then().setCurrentPostion(Config.field.startingFilmPosGoal);

        builder.appendSequence(Sequences.spinUpFarShot(Config.shooter.speed.autoLineRPS));
        builder.appendSequence(Sequences.startShooting());

        builder.then().setDelayDelta(2);

        builder.appendSequence(Sequences.stopShooting());

        builder.then().driveRelativeWaypoints(Config.field.startingFilmPosGoal, List.of(),
                Config.field.startingFilmPosGoal
                        .plus(new Transform2d(new Translation2d(-1, 0), new Rotation2d(0))),
                false);

        autoProgram.addOption("Shoot and Drive", builder.build());
    }

    private void add8BallAutoSequence() {

        SequenceBuilder builder = new SequenceBuilder("8 Ball Auto");

        builder.then("Setting pos: auto line in trench")
                .setCurrentPostion(Config.field.autoLineAllianceTrench);

        builder.appendSequence(Sequences.spinUpFarShot(Config.shooter.speed.autoLineRPS));

        builder.then().deployIntake();

        // Let shooter spin up a little before running every other motor
        // builder.add().setDelayDelta(0.5);

        // Start intaking
        builder.appendSequence(Sequences.startIntaking());

        // Drive backwards to pick up the two balls, starting with front bumpers on auto
        // line

        builder.then("Drive backwards").driveRelativeWaypoints(Config.field.autoLineAllianceTrench,
                List.of(),
                intakeAt(Config.field.allianceTrenchSecondBall, 0), false); // backwards

        // Keep running intake until after vision aim to ensure all balls make it into
        // the loader.
        builder.then().doVisionAim();
        // Stop intaking
        builder.then().setIntakeRPS(0).setPassthroughDutyCycle(0).setSpinnerRPS(0);
        // Start shooting
        builder.appendSequence(Sequences.startShooting());
        builder.then().setDelayDelta(1.75);

        // Aim straight again after vision aim
        builder.then().doTurnToHeading(0);

        // Pick up the last 3 balls
        builder.appendSequence(Sequences.startIntaking());

        builder.then("Intake trench balls").driveRelativeWaypoints(
                intakeAt(Config.field.allianceTrenchSecondBall, 0),
                List.of(), intakeAt(Config.field.allianceTrenchFifthBall, 0), false);

        // Drive forward and shoot
        builder.then("Driving forward").driveRelativeWaypoints(
                intakeAt(Config.field.allianceTrenchFifthBall, 0),
                List.of(), intakeAt(Config.field.allianceTrenchSecondBall, 0), true);
        // Keep running intake until after vision aim to ensure all balls make it into
        // the loader.
        builder.then().doVisionAim();
        // Stop intaking
        builder.then().setIntakeRPS(0).setPassthroughDutyCycle(0).setSpinnerRPS(0);
        // Shoot the balls.
        builder.appendSequence(Sequences.spinUpFarShot(Config.shooter.speed.autoLineRPS));
        builder.appendSequence(Sequences.startShooting());
        builder.then("Shooting").setDelayDelta(2);

        builder.appendSequence(Sequences.stopShooting());

        autoProgram.addOption("8 Ball Auto", builder.build());
    }

    private void add5BallOpponentTrench() {
        SequenceBuilder builder = new SequenceBuilder("5 Ball Opponent Trench Auto");

        builder.then("Setting pos: auto line facing opponent trench")
                .setCurrentPostion(Config.field.autoLineOpposingTrench);

        builder.appendSequence(Sequences.startIntaking());

        // Drive backwards to pick up the two balls, starting with front bumpers on auto
        // line
        try {
            builder.then("Drive backwards").driveRelativeWaypoints(AutoPaths.k5ballOpponent1);
        } catch (IOException e) {
            autoProgram.addOption("FAILED - 5 ball opponent 1", Sequences.getEmptySequence());
        }

        builder.then().setDelayDelta(0.25);

        builder.appendSequence(Sequences.spinUpFarShot(Config.shooter.speed.autoLineRPS));

        try {
            builder.then("Drive to goal").driveRelativeWaypoints(AutoPaths.k5ballOpponent2);
        } catch (IOException e) {
            autoProgram.addOption("FAILED - 5 ball opponent 2", Sequences.getEmptySequence());
        }

        builder.appendSequence(Sequences.stopIntaking());

        builder.then().doVisionAim();
        builder.then().setDelayDelta(0.5);

        builder.appendSequence(Sequences.startShooting());
        builder.then().setDelayDelta(5);

        builder.appendSequence(Sequences.stopShooting());

        autoProgram.addOption("5 Ball Opponent Trench Auto", builder.build());
    }

    @SuppressWarnings("unused")
    private void addAutoNavBarrelRacing() {
        SequenceBuilder builder = new SequenceBuilder("AutoNav Barrel Racing Path");
        builder.then("Setting pos: barrel racing start pos")
                .setCurrentPostion(Config.field.barrelRacingStartPos);

        try {
            builder.then().driveRelativeWaypoints(AutoPaths.kBarrelRacing);
        } catch (IOException e) {
            autoProgram.addOption("FAILED - AutoNav Barrel Racing Path",
                    Sequences.getEmptySequence());
        }
        autoProgram.addOption("AutoNav Barrel Racing Path", builder.build());
    }

    @SuppressWarnings("unused")
    private void addAutoNavBounce() {
        SequenceBuilder builder = new SequenceBuilder("AutoNav Bounce Path");
        builder.then("Setting pos: barrel racing start pos")
                .setCurrentPostion(Config.field.barrelRacingStartPos);

        try {
            builder.then().driveRelativeWaypoints(AutoPaths.kBounce0);
            builder.then().driveRelativeWaypoints(AutoPaths.kBounce1);
            builder.then().driveRelativeWaypoints(AutoPaths.kBounce2);
            builder.then().driveRelativeWaypoints(AutoPaths.kBounce3);
        } catch (IOException e) {
            autoProgram.addOption("FAILED - AutoNav Bounce Path", Sequences.getEmptySequence());
        }
        autoProgram.addOption("AutoNav Bounce Path", builder.build());
    }

    @SuppressWarnings("unused")
    private void addPathweaverTest() {
        SequenceBuilder builder = new SequenceBuilder("Pathweaver Test");

        try {
            builder.then().setCurrentPostion(createPose2d(3.436, 6.232, 180.0));
            builder.then().driveRelativeWaypoints(AutoPaths.kToBalls);
            builder.then().setDelayDelta(1);
            builder.then().driveRelativeWaypoints(AutoPaths.kToGoal);
            builder.then().setDelayDelta(1);
            builder.then().driveRelativeWaypoints(AutoPaths.kToStart);
            info("Created Pathweaver routine");
            autoProgram.addOption("Pathweaver Test", builder.build());
        } catch (Exception e) {
            autoProgram.addOption("FAILED - Pathweaver Test", Sequences.getEmptySequence());
        }
    }

    private void initAutoChooser() {
        SmartDashboard.putData("Auto program", autoProgram);
    }

    private void initBallChooser() {
        SmartDashboard.putData("Initial balls", initBallSelector);
    }

    @Override
    public String getName() {
        return "Auto";
    }
}
