package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.lib.abe.AbeBot;
import org.firstinspires.ftc.teamcode.lib.abe.AbeTeleOp;

@TeleOp
public class AbeAutomatic extends AbeTeleOp {
    @Override
    public void runOpMode() throws InterruptedException {
        initializeAbe();

        waitForStart();

        while(opModeIsActive()){

        }
    }
}
