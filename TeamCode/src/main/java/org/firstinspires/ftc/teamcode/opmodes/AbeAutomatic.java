package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.lib.abe.AbeBot;

@TeleOp
public class AbeAutomatic extends LinearOpMode {
    // el robot
    private AbeBot abe;

    @Override
    public void runOpMode() throws InterruptedException {
        try {
            abe = new AbeBot(hardwareMap);
        } catch (Exception e) {
            telemetry.addLine("error: " + e.getMessage());
            telemetry.update();

            waitForStart();

            return;
        }

        waitForStart();

        while(opModeIsActive()){

        }
    }
}
