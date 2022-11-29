package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.teamcode.lib.motion.AngleAdjuster;

@TeleOp(group = "Tests")
public class AngleAdjusterTest extends LinearOpMode {
    DcMotorEx angleMotor;
    TouchSensor limitSensor;

    public void loadHardware(){
        this.angleMotor = hardwareMap.get(DcMotorEx.class, "angleMotor");
        this.limitSensor = hardwareMap.get(TouchSensor.class, "limitSwitch");
    }

    @Override
    public void runOpMode(){
        // init

        telemetry.addLine("Initializing hardware...");
        telemetry.update();

        this.loadHardware();

        // create motor
        AngleAdjuster motor = new AngleAdjuster(this.angleMotor, 24.0, 384.5, this.limitSensor);

        telemetry.addLine("Done.");
        telemetry.update();

        // TODO: abstract to class
        double[] values = {0, 0};
        String[] prompts = {"Angle? (positive = down, negative = up)", "Velocity? (No negative)"};
        int index = -1;
        boolean first = true;
        boolean held = false;

        while(opModeInInit()){
            if( (gamepad1.a && !held) || first) {
                first = false;
                held = true;

                index++;

                if (index >= values.length) {
                    telemetry.addLine("Done");
                    telemetry.update();
                    break;
                }
            } else if(!gamepad1.a){
                held = false;
            }

            if(gamepad1.left_stick_y > 0.01 || gamepad1.left_stick_y < 0.01){
                double change = -gamepad1.left_stick_y / 200; // FIXME: dumb

                values[index] += change;
            }

            telemetry.addLine(prompts[index]);
            telemetry.addData("value", values[index]);
            telemetry.update();
        }

        // wait for driver to press start
        waitForStart();

        // start with a simple 45 degree test
        motor.rotateAngleDegrees(values[0], values[1]);

        // run
        while(opModeIsActive()){
            motor.check();
        }
    }
}