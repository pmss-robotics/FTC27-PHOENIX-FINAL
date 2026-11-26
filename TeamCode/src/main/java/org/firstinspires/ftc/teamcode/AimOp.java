package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.AxonSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DualIntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.FlywheelSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.GateSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.HoodSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.TurretAimer;

/**
 * Field-centric drive, intake / shoot cycle, and shoot-on-the-move turret aiming.
 * Turret and shot values are in subsystems/TurretAimer.java.
 *
 * Gamepad1:
 *   left stick = drive, right stick X = turn
 *   hold right bumper = intake (gate closed), release = short backoff, then flywheels spin up
 *   hold Y = open gate and feed the shot, release = everything off
 *   X = turret auto-aim on/off (starts on; off = facing forward)
 *   A = hood highest, B = hood lowest
 *   dpad up/down = shooter RPM, dpad left/right = aim trim
 *   left bumper = flip flywheel direction
 *   BACK = reset pose to the start pose
 */
@TeleOp(name = "AimOp", group = "Main")
public class AimOp extends LinearOpMode {
    // ---- Field (real inches). Origin = back-left corner, +x = right, +y = forward ----
    private static final double TILE = 23.5;          // 6 tiles = 141". Measure yours

    // Start: facing forward, robot center measured from the walls
    private static final double START_X = 146 / 25.4; // 146 mm from the left wall
    private static final double START_Y = 200 / 25.4; // 200 mm from the back wall
    private static final double START_HEADING = Math.PI / 2; // facing +y

    // Target: center of tile (3, 3), counting from tile (0, 0) in the start corner
    private static final double TARGET_X = 3.5 * TILE;
    private static final double TARGET_Y = 3.5 * TILE;

    private static final double START_RPM = 3000;
    private static final double RPM_STEP = 100;
    private static final double TRIM_STEP_DEG = 1.0;
    private static final long BACKOFF_MS = 25;

    @Override
    public void runOpMode() {
        Pose startPose = new Pose(START_X, START_Y, START_HEADING);
        Follower follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);

        DualIntakeSubsystem intake = new DualIntakeSubsystem(hardwareMap);
        FlywheelSubsystem flywheel = new FlywheelSubsystem(hardwareMap);
        GateSubsystem gate = new GateSubsystem(hardwareMap);
        HoodSubsystem hood = new HoodSubsystem(hardwareMap);
        AxonSubsystem turret = new AxonSubsystem(hardwareMap); // starts at 0.5, forward
        turret.useFullPwmRange();
        TurretAimer aimer = new TurretAimer();

        boolean aiming = true;
        boolean spinning = false;
        long backoffUntil = 0;
        double trimDeg = 0;
        boolean lastRb = false, lastY = false, lastX = false, lastLb = false;
        boolean lastUp = false, lastDown = false, lastLeft = false, lastRight = false;

        waitForStart();
        flywheel.setRpm(START_RPM);
        gate.block();
        hood.max();
        follower.startTeleopDrive(true);
        long lastNanos = System.nanoTime();

        while (opModeIsActive()) {
            boolean rb = gamepad1.right_bumper;
            boolean y = gamepad1.y;
            long now = System.currentTimeMillis();

            // ---- Drive, field-centric: stick up = away from the back wall ----
            follower.setTeleOpDrive(
                    -gamepad1.left_stick_y,
                    -gamepad1.left_stick_x,
                    -gamepad1.right_stick_x,
                    false,           // field-centric
                    START_HEADING);  // which field direction is "stick up"
            follower.update();
            if (gamepad1.back) follower.setPose(startPose);

            // ---- Intake / shoot cycle ----
            if (rb && !lastRb) spinning = false;
            if (lastRb && !rb && !y) {
                backoffUntil = now + BACKOFF_MS;
                spinning = true;
            }
            if (lastY && !y) spinning = false;

            if (y) {
                gate.open();
                intake.on();
                flywheel.setRunning(true);
            } else if (rb) {
                gate.block();
                intake.on();
                flywheel.setRunning(false);
            } else if (now < backoffUntil) {
                gate.block();
                intake.reverse();
                flywheel.setRunning(false);
            } else {
                gate.block();
                intake.off();
                flywheel.setRunning(spinning);
            }

            if (gamepad1.left_bumper && !lastLb) flywheel.flipDirection();
            if (gamepad1.dpad_up && !lastUp) flywheel.adjustRpm(RPM_STEP);
            if (gamepad1.dpad_down && !lastDown) flywheel.adjustRpm(-RPM_STEP);
            flywheel.update();

            // ---- Hood ----
            if (gamepad1.a) hood.max();
            if (gamepad1.b) hood.low();

            // ---- Turret ----
            if (gamepad1.x && !lastX) aiming = !aiming;
            if (gamepad1.dpad_left && !lastLeft) trimDeg += TRIM_STEP_DEG;    // counterclockwise
            if (gamepad1.dpad_right && !lastRight) trimDeg -= TRIM_STEP_DEG;  // clockwise

            Pose pose = follower.getPose();
            Vector velocity = follower.getVelocity();
            // Computed even when not aiming, so the velocity filter stays current
            double aimGoal = aimer.aim(pose,
                    velocity.getXComponent(), velocity.getYComponent(), follower.getAngularVelocity(),
                    TARGET_X, TARGET_Y, flywheel.getTargetRpm(), trimDeg);
            double goal = aiming ? aimGoal : TurretAimer.FORWARD;

            long nanos = System.nanoTime();
            turret.setPosition(aimer.update(goal, (nanos - lastNanos) / 1e9));
            lastNanos = nanos;

            lastRb = rb;
            lastY = y;
            lastX = gamepad1.x;
            lastLb = gamepad1.left_bumper;
            lastUp = gamepad1.dpad_up;
            lastDown = gamepad1.dpad_down;
            lastLeft = gamepad1.dpad_left;
            lastRight = gamepad1.dpad_right;

            telemetry.addData("pose", "x=%.1f y=%.1f h=%.0f",
                    pose.getX(), pose.getY(), Math.toDegrees(pose.getHeading()));
            telemetry.addData("turret", aiming ? "AIMING" : "forward");
            telemetry.addData("turret servo", "%.3f", turret.getPosition());
            telemetry.addData("turret on target", aimer.onTarget(goal));
            telemetry.addData("distance to target", "%.1f", aimer.getDistance());
            telemetry.addData("flight time (s)", "%.2f", aimer.getFlightTime());
            telemetry.addData("moving lead (deg)", "%.1f", aimer.getLeadDegrees());
            telemetry.addData("aim trim (deg)", trimDeg);
            telemetry.addData("target RPM", flywheel.getTargetRpm());
            telemetry.addData("actual RPM", flywheel.getActualRpm());
            telemetry.addData("at speed", flywheel.atSpeed());
            telemetry.update();
        }

        intake.off();
        flywheel.stop();
    }
}
