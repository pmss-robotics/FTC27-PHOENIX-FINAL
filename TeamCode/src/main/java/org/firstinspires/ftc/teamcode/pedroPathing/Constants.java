package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/** Pedro Pathing 2 setup. FILL = measure or tune it. CHECK = carried over, verify it. */
public class Constants {

    public static FollowerConstants followerConstants = new FollowerConstants()

            .mass(10.0)                              // FILL: robot mass in kg
            .forwardZeroPowerAcceleration(-54.26147119547814)     // FILL: Forward Zero Power Acceleration tuner
            .lateralZeroPowerAcceleration(-70.08631434527223)     // FILL: Lateral Zero Power Acceleration tuner
            // FILL: PIDF tuners. These are starting values.
            .translationalPIDFCoefficients(new PIDFCoefficients(0.19, 0, 0.016, 0.04))
            .headingPIDFCoefficients(new PIDFCoefficients(2.0, 0, 0.1, 0.01))
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.025, 0, 0.00001, 0.6, 0.01))
            .centripetalScaling(0.0005);             // FILL: Centripetal tuner

    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1.0)
            .leftFrontMotorName("leftFront")         // CHECK
            .leftRearMotorName("leftBack")           // CHECK
            .rightFrontMotorName("rightFront")       // CHECK
            .rightRearMotorName("rightBack")         // CHECK
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .xVelocity(78.99747689314714)                         // FILL: Forward Velocity tuner
            .yVelocity(65.8722649521715);                        // FILL: Lateral Velocity tuner


    public static PinpointConstants localizerConstants = new PinpointConstants()
            .hardwareMapName("pinpoint")
            .distanceUnit(DistanceUnit.INCH)
            .forwardPodY(1.338)                      // CHECK: forward pod offset
            .strafePodX(-4.37 )                      // CHECK: strafe pod offset

            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD)  // CHECK
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD);   // CHECK

    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 1, 1);

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .mecanumDrivetrain(driveConstants)
                .pinpointLocalizer(localizerConstants)
                .pathConstraints(pathConstraints)
                .build();
    }
}
