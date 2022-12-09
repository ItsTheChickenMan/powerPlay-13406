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
            update();

            telemetry.update();
        }
    }
}
