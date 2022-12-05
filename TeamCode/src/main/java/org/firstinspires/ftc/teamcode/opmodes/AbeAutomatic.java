package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.lib.abe.AbeBot;
import org.firstinspires.ftc.teamcode.lib.abe.AbeTeleOp;
import org.firstinspires.ftc.teamcode.lib.utils.GlobalStorage;

@TeleOp
public class AbeAutomatic extends AbeTeleOp {
    @Override
    public void runOpMode() throws InterruptedException {
        GlobalStorage.globalTelemetry = telemetry;

        setup();

        // note: roadrunner coordinates
        setStartPoint(8.5, 36, 0);

        waitForStart();

        while(opModeIsActive()) {
            // drive stuff
            double forward = -gamepad1.left_stick_y;
            double strafe = gamepad1.left_stick_x;
            double speed = 30.0 * (1.15 - Math.max(gamepad1.left_trigger, gamepad1.right_trigger));

            this.abe.drive.driveFieldOriented(forward*speed, strafe*speed);

            update();

            telemetry.update();
        }
    }
}
