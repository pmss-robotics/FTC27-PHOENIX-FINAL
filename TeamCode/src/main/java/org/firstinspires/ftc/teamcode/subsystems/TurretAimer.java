package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.util.Range;

/**
 * Turret aiming with shoot-on-the-move lead, backlash compensation and a speed / acceleration limit.
 * Distances in inches, time in seconds. All public static values can be tuned live in FTC Dashboard.
 */
@Config
public class TurretAimer {
    // ---- Turret geometry ----
    public static double FORWARD = 0.5;         // servo position facing forward
    public static double RANGE_DEG = 350 * 1.5; // turret degrees for servo 0 -> 1
    public static double SIGN = -1;             // -1: higher servo = clockwise
    public static double BACK_OFFSET = 2.0;     // pivot distance behind robot center

    // ---- Gear protection ----
    public static double MAX_DEG_PER_SEC = 150;  // lower if the gears still slip
    public static double MAX_DEG_PER_SEC2 = 500; // how hard it may start and stop

    // ---- Backlash ----
    public static double BACKLASH_DEG = 5.0;     // total gear slop to take up. TUNE
    public static double REVERSE_THRESHOLD_DEG = 0.75; // ignore smaller direction changes (jitter)

    // ---- Shot model (estimates) ----
    public static double WHEEL_DIAMETER = 72 / 25.4;
    public static double EXIT_EFFICIENCY = 0.5;  // ball speed / wheel surface speed. TUNE
    public static double LAUNCH_ANGLE_DEG = 50;  // from horizontal. TUNE
    public static double RELEASE_DELAY = 0.15;   // fire command -> ball leaves. TUNE
    public static double TURRET_LAG = 0.08;      // servo response time. TUNE
    public static double VELOCITY_FILTER = 0.4;  // 0..1, higher = reacts faster, noisier
    public static double STILL_SPEED = 2.0;      // in/s, below this counts as not moving

    private double vx, vy, omega;
    private double command = FORWARD;   // rate-limited servo position, before backlash
    private double speed = 0;           // servo units per second
    private double output = FORWARD;
    private double reference = FORWARD; // last goal that set the direction
    private int direction = 0;

    private double flightTime, distance, leadDeg;

    /**
     * Servo position that scores from here while moving.
     * velX / velY are field velocity, angular is rad/s counterclockwise.
     */
    public double aim(Pose pose, double velX, double velY, double angular,
                      double targetX, double targetY, double rpm, double trimDeg) {
        vx += VELOCITY_FILTER * (velX - vx);
        vy += VELOCITY_FILTER * (velY - vy);
        omega += VELOCITY_FILTER * (angular - omega);
        boolean still = Math.hypot(vx, vy) < STILL_SPEED;
        double useVx = still ? 0 : vx;
        double useVy = still ? 0 : vy;

        // Where the turret pivot will be, and how fast it will move, when the ball leaves
        double h = pose.getHeading() + omega * RELEASE_DELAY;
        double offsetX = -BACK_OFFSET * Math.cos(h);
        double offsetY = -BACK_OFFSET * Math.sin(h);
        double pivotX = pose.getX() + useVx * RELEASE_DELAY + offsetX;
        double pivotY = pose.getY() + useVy * RELEASE_DELAY + offsetY;
        double launchVx = useVx - omega * offsetY; // includes the pivot swinging as the robot turns
        double launchVy = useVy + omega * offsetX;

        // The ball keeps the launch velocity, so aim at target - velocity * flight time.
        double surfaceSpeed = Math.PI * WHEEL_DIAMETER * Math.max(rpm, 500) / 60.0;
        double horizontalSpeed = Math.max(1,
                surfaceSpeed * EXIT_EFFICIENCY * Math.cos(Math.toRadians(LAUNCH_ANGLE_DEG)));
        double aimX = targetX, aimY = targetY;
        for (int i = 0; i < 8; i++) { // flight time depends on the aim point, so iterate
            flightTime = Math.min(1.5, Math.hypot(aimX - pivotX, aimY - pivotY) / horizontalSpeed);
            aimX = targetX - launchVx * flightTime;
            aimY = targetY - launchVy * flightTime;
        }
        distance = Math.hypot(targetX - pivotX, targetY - pivotY);

        double aimAngle = Math.atan2(aimY - pivotY, aimX - pivotX);
        leadDeg = Math.toDegrees(wrap(aimAngle - Math.atan2(targetY - pivotY, targetX - pivotX)));

        double turretHeading = pose.getHeading() + omega * TURRET_LAG;
        double relDeg = Math.toDegrees(wrap(aimAngle - turretHeading)) + trimDeg;

        // Pick the reachable turn closest to where the turret is now
        double best = Double.NaN;
        for (int turn = -1; turn <= 1; turn++) {
            double pos = FORWARD + SIGN * (relDeg + 360 * turn) / RANGE_DEG;
            if (pos < 0 || pos > 1) continue;
            if (Double.isNaN(best) || Math.abs(pos - command) < Math.abs(best - command)) best = pos;
        }
        return Double.isNaN(best) ? command : best;
    }

    /** Move toward goal within the speed limits. Returns the position to send to the servos. */
    public double update(double goal, double dt) {
        dt = Range.clip(dt, 0, 0.05);

        double threshold = REVERSE_THRESHOLD_DEG / RANGE_DEG;
        if (goal > reference + threshold) {
            direction = 1;
            reference = goal;
        } else if (goal < reference - threshold) {
            direction = -1;
            reference = goal;
        } else if ((goal - reference) * direction > 0) {
            reference = goal;
        }

        double maxSpeed = MAX_DEG_PER_SEC / RANGE_DEG;
        double maxAccel = MAX_DEG_PER_SEC2 / RANGE_DEG;
        // Backlash offset is part of the target, so it is rate limited too
        double target = Range.clip(goal + direction * BACKLASH_DEG / 2 / RANGE_DEG, 0, 1);

        double error = target - command;
        double wanted = Math.signum(error)
                * Math.min(maxSpeed, Math.sqrt(2 * maxAccel * Math.abs(error)));
        speed += Range.clip(wanted - speed, -maxAccel * dt, maxAccel * dt);
        double step = speed * dt;
        if (step * error > 0 && Math.abs(step) >= Math.abs(error)) {
            command = target;
            speed = 0;
        } else {
            command = Range.clip(command + step, 0, 1);
        }
        output = command;
        return output;
    }

    /** True when the turret has caught up with its target. */
    public boolean onTarget(double goal) {
        return Math.abs(goal + direction * BACKLASH_DEG / 2 / RANGE_DEG - output) * RANGE_DEG < 3;
    }

    public double getFlightTime() { return flightTime; }
    public double getDistance() { return distance; }
    public double getLeadDegrees() { return leadDeg; }

    private static double wrap(double radians) {
        return Math.atan2(Math.sin(radians), Math.cos(radians));
    }
}
