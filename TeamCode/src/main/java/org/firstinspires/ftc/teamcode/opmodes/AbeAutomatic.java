package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.lib.abe.AbeBot;
import org.firstinspires.ftc.teamcode.lib.abe.AbeTeleOp;

@TeleOp
public class AbeAutomatic extends AbeTeleOp {
    @Override
    public void runOpMode() throws InterruptedException {
        initializeAbe();

        setStartPoint(108.0, 8.5, 0);

        waitForStart();

        while(opModeIsActive()) {
            // drive stuff
            double forward = -gamepad1.left_stick_y;
            double strafe = gamepad1.left_stick_x;
            double speed = 1.3 - gamepad1.right_trigger;

            this.abe.drive.driveFieldOriented(forward*speed, strafe*speed);

            update();
        }
    }
}
