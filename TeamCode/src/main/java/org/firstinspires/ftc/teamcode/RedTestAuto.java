package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.AxonSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DualIntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.FlywheelSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.GateSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.HoodSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.TurretAimer;

/**
 * Red test auto: shoot preloads at the start, collect from the corner, shoot from the far side,
 * park in the loading zone. Values below can be edited here or in FTC Dashboard before Init.
 * Inches and degrees. Origin = audience-side red corner, +x right, +y away from the audience.
 */
@Config
@Autonomous(name = "RedTestAuto", group = "Main")
public class RedTestAuto extends LinearOpMode {
    // ================= SHOT 1 (from the start position) =================
    public static double FLYWHEEL_RPM_1 = 3000;
    public static double HOOD_POS_1 = 0.300;          // 0.300 highest .. 0.412 lowest
    public static double TARGET_X_1 = 59, TARGET_Y_1 = 61; // near red cell (estimate)
    public static boolean USE_MANUAL_AIM_1 = false;   // true = ignore the calculation
    public static double MANUAL_AIM_DEG_1 = 0;        // turret from robot front, + = left
    public static double AIM_TRIM_DEG_1 = 0;          // added to the calculated angle, + = left

    // ================= SHOT 2 (far side) =================
    public static double FLYWHEEL_RPM_2 = 3000;
    public static double HOOD_POS_2 = 0.300;
    public static double TARGET_X_2 = 59, TARGET_Y_2 = 88; // far red cell (estimate)
    public static boolean USE_MANUAL_AIM_2 = false;
    public static double MANUAL_AIM_DEG_2 = 48;
    public static double AIM_TRIM_DEG_2 = 0;

    // ================= POSITIONS =================
    public static double START_X = 58.75, START_Y = 7.9, START_HEADING = 90;
    public static double PICKUP_X = 10, PICKUP_Y = 8.5, PICKUP_HEADING = -90;
    public static double PICKUP_CONTROL1_X = 58.75, PICKUP_CONTROL1_Y = 33.75;
    public static double PICKUP_CONTROL2_X = 10, PICKUP_CONTROL2_Y = 42.75;
    public static double SHOOT_X = 33.75, SHOOT_Y = 124.75, SHOOT_HEADING = -106;
    public static double PARK_X = 12, PARK_Y = 108, PARK_HEADING = 180;

    // ================= SHOOTING / INTAKE =================
    public static double SHOOT_FEED_POWER = 0.6;      // intake power while shooting
    public static double SHOOT_FEED_SECONDS = 2.0;    // total feeding time per shot spot
    public static double SHOOT_TIMEOUT = 5.0;         // give up on a shot spot after this
    public static double SPINUP_FALLBACK = 1.5;       // wait used if no flywheel encoder
    public static double BACKOFF_SECONDS = 0.05;      // reverse intake before spin up
    public static double PICKUP_DRIVE_POWER = 0.6;    // drive speed into the corner
    public static double PICKUP_WAIT = 0.75;          // keep intaking after arriving
    public static double INTAKE_AFTER_PICKUP = 0.5;   // keep intaking while pulling away

    // ================= SAFETY =================
    public static double PATH_TIMEOUT = 6.0;          // move on if a path takes longer
    public static double PARK_AT = 25.0;              // drop everything and park at this time

    private enum State { SHOOT_1, TO_PICKUP, PICKUP, TO_SHOOT, SHOOT_2, PARK, DONE }

    private State state = State.SHOOT_1;
    private final ElapsedTime stateTime = new ElapsedTime();
    private final ElapsedTime autoTime = new ElapsedTime();
    private double fedSeconds = 0;

    private Follower follower;
    private DualIntakeSubsystem intake;
    private FlywheelSubsystem flywheel;
    private GateSubsystem gate;
    private HoodSubsystem hood;

    @Override
    public void runOpMode() {
        follower = Constants.createFollower(hardwareMap);
        Pose start = pose(START_X, START_Y, START_HEADING);
        follower.setStartingPose(start);

        intake = new DualIntakeSubsystem(hardwareMap);
        flywheel = new FlywheelSubsystem(hardwareMap);
        gate = new GateSubsystem(hardwareMap);
        hood = new HoodSubsystem(hardwareMap);
        AxonSubsystem turret = new AxonSubsystem(hardwareMap); // moves to forward
        turret.useFullPwmRange();
        TurretAimer aimer = new TurretAimer();
        gate.block();

        Pose pickup = pose(PICKUP_X, PICKUP_Y, PICKUP_HEADING);
        Pose shoot = pose(SHOOT_X, SHOOT_Y, SHOOT_HEADING);
        Pose park = pose(PARK_X, PARK_Y, PARK_HEADING);

        PathChain toPickup = follower.pathBuilder()
                .addPath(new BezierCurve(start,
                        new Pose(PICKUP_CONTROL1_X, PICKUP_CONTROL1_Y),
                        new Pose(PICKUP_CONTROL2_X, PICKUP_CONTROL2_Y),
                        pickup))
                .setTangentHeadingInterpolation()
                .build();
        PathChain toShoot = follower.pathBuilder()
                .addPath(new BezierLine(pickup, shoot))
                .setLinearHeadingInterpolation(pickup.getHeading(), shoot.getHeading())
                .build();

        telemetry.addLine("Ready. Robot at the start position, preloads in.");
        telemetry.update();
        waitForStart();

        autoTime.reset();
        enterShot(State.SHOOT_1, FLYWHEEL_RPM_1, HOOD_POS_1);
        long lastNanos = System.nanoTime();

        while (opModeIsActive()) {
            follower.update();
            flywheel.update();
            long nanos = System.nanoTime();
            double dt = (nanos - lastNanos) / 1e9;
            lastNanos = nanos;

            if (autoTime.seconds() >= PARK_AT && state != State.PARK && state != State.DONE) {
                startPark(park);
            }

            switch (state) {
                case SHOOT_1:
                    if (shootDone(dt)) {
                        gate.block();
                        flywheel.stop();
                        intake.setPower(1);
                        follower.followPath(toPickup, PICKUP_DRIVE_POWER, true);
                        enter(State.TO_PICKUP);
                    }
                    break;
                case TO_PICKUP:
                    intake.setPower(1);
                    if (pathDone()) enter(State.PICKUP);
                    break;
                case PICKUP:
                    intake.setPower(1);
                    if (stateTime.seconds() >= PICKUP_WAIT) {
                        follower.followPath(toShoot, 1.0, true);
                        hood.setPosition(HOOD_POS_2);
                        flywheel.setRpm(FLYWHEEL_RPM_2);
                        enter(State.TO_SHOOT);
                    }
                    break;
                case TO_SHOOT:
                    // finish intaking, back off briefly, then spin up on the way
                    double t = stateTime.seconds();
                    if (t < INTAKE_AFTER_PICKUP) {
                        intake.setPower(1);
                    } else if (t < INTAKE_AFTER_PICKUP + BACKOFF_SECONDS) {
                        intake.reverse();
                    } else {
                        intake.off();
                        flywheel.setRunning(true);
                    }
                    if (pathDone()) enterShot(State.SHOOT_2, FLYWHEEL_RPM_2, HOOD_POS_2);
                    break;
                case SHOOT_2:
                    if (shootDone(dt)) startPark(park);
                    break;
                case PARK:
                    if (pathDone()) enter(State.DONE);
                    break;
                case DONE:
                    break;
            }

            // ---- Turret: aim at shot 1 early on, at shot 2 once we leave the corner ----
            boolean first = state == State.SHOOT_1;
            boolean second = state == State.TO_SHOOT || state == State.SHOOT_2;
            Pose now = follower.getPose();
            double goal = TurretAimer.FORWARD;
            if (first || second) {
                boolean manual = first ? USE_MANUAL_AIM_1 : USE_MANUAL_AIM_2;
                if (manual) {
                    double degrees = first ? MANUAL_AIM_DEG_1 : MANUAL_AIM_DEG_2;
                    goal = TurretAimer.FORWARD + TurretAimer.SIGN * degrees / TurretAimer.RANGE_DEG;
                } else {
                    // Aim from where the robot will be when it shoots
                    Pose from = first ? start : shoot;
                    if (state == State.SHOOT_2) from = now;
                    goal = aimer.aim(from, 0, 0, 0,
                            first ? TARGET_X_1 : TARGET_X_2,
                            first ? TARGET_Y_1 : TARGET_Y_2,
                            first ? FLYWHEEL_RPM_1 : FLYWHEEL_RPM_2,
                            first ? AIM_TRIM_DEG_1 : AIM_TRIM_DEG_2);
                }
            }
            turret.setPosition(aimer.update(goal, dt));
            turretGoal = goal;
            turretReady = aimer.onTarget(goal);

            telemetry.addData("state", state);
            telemetry.addData("time", "%.1f", autoTime.seconds());
            telemetry.addData("pose", "x=%.1f y=%.1f h=%.0f",
                    now.getX(), now.getY(), Math.toDegrees(now.getHeading()));
            telemetry.addData("turret angle (deg, + = left)", "%.1f",
                    (turretGoal - TurretAimer.FORWARD) / TurretAimer.SIGN * TurretAimer.RANGE_DEG);
            telemetry.addData("turret ready", turretReady);
            telemetry.addData("rpm", "%.0f / %.0f", flywheel.getActualRpm(), flywheel.getTargetRpm());
            telemetry.update();
        }

        intake.off();
        flywheel.stop();
        gate.block();
    }

    private double turretGoal = 0.5;
    private boolean turretReady = false;

    private void enter(State next) {
        state = next;
        stateTime.reset();
    }

    private void enterShot(State next, double rpm, double hoodPos) {
        hood.setPosition(hoodPos);
        flywheel.setRpm(rpm);
        flywheel.setRunning(true);
        gate.block();
        intake.off();
        fedSeconds = 0;
        turretReady = false;
        enter(next);
    }

    /** Opens the gate once ready, feeds only while the flywheels are at speed. True when finished. */
    private boolean shootDone(double dt) {
        boolean atSpeed = flywheel.atSpeed()
                || (!flywheel.hasEncoder() && stateTime.seconds() > SPINUP_FALLBACK);
        // The turret gets 0.3 s to start moving before "ready" is trusted
        boolean aimed = turretReady && stateTime.seconds() > 0.3;
        if (atSpeed && aimed) {
            gate.open();
            intake.setPower(SHOOT_FEED_POWER);
            fedSeconds += dt;
        } else {
            intake.off();
        }
        return fedSeconds >= SHOOT_FEED_SECONDS || stateTime.seconds() >= SHOOT_TIMEOUT;
    }

    private boolean pathDone() {
        return !follower.isBusy() || stateTime.seconds() >= PATH_TIMEOUT;
    }

    private void startPark(Pose park) {
        gate.block();
        intake.off();
        flywheel.stop();
        Pose from = follower.getPose();
        follower.followPath(follower.pathBuilder()
                .addPath(new BezierLine(from, park))
                .setLinearHeadingInterpolation(from.getHeading(), park.getHeading())
                .build(), 1.0, true);
        enter(State.PARK);
    }

    private static Pose pose(double x, double y, double headingDegrees) {
        return new Pose(x, y, Math.toRadians(headingDegrees));
    }
}
