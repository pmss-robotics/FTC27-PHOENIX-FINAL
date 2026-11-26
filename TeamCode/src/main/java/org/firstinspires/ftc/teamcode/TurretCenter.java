package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.AxonSubsystem;

/** Holds the turret servos at 0.5 (forward) so the turret can be re-seated on its gears. */
@TeleOp(name = "TurretCenter", group = "Test")
public class TurretCenter extends LinearOpMode {
    @Override
    public void runOpMode() {
        AxonSubsystem turret = new AxonSubsystem(hardwareMap);
        turret.useFullPwmRange(); // same range as AimOp

        telemetry.addLine("Servos move to 0.5 now. Keep clear.");
        telemetry.update();

        while (!isStopRequested()) { // holds during Init and after Start
            turret.setPosition(0.5);
            telemetry.addData("turret servo", "%.3f (hold)", turret.getPosition());
            telemetry.addData("encoder angle (deg)", "%.1f", turret.getAngleDegrees());
            telemetry.update();
            idle();
        }
    }
}
