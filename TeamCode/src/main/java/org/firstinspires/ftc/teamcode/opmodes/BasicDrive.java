package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.lib.abe.AbeConstants;
import org.firstinspires.ftc.teamcode.lib.drive.MecanumDrive;
import org.firstinspires.ftc.teamcode.lib.motion.PositionableMotor;

/**
 * @brief Basic opmode for driving around in normal, bot relative mecanum drive
 */

// we ALWAYS put @TeleOp in front of op modes
@TeleOp
public class BasicDrive extends LinearOpMode {
    // hardware definitions
    DcMotorEx frontLeft;
    DcMotorEx frontRight;
    DcMotorEx backLeft;
    DcMotorEx backRight;

    /**
     * @brief Load all hardware required for this opmode
     *
     * @note for most Power Play opmodes, we'll have a class for this so that we don't have to do it for each opmode.  We use a different one here because we don't need as much hardware
     */
    public void loadHardware(){
        this.frontLeft = hardwareMap.get(DcMotorEx.class, "frontLeft");
        this.frontRight = hardwareMap.get(DcMotorEx.class, "frontRight");
        this.backLeft = hardwareMap.get(DcMotorEx.class, "backLeft");
        this.backRight = hardwareMap.get(DcMotorEx.class, "backRight");
    }

    @Override
    public void runOpMode() throws InterruptedException {
        // initialize hardware
        // NOTE: we use telemetry for logging to the driver station console.  note that you can see these lines as you run the op mode
        telemetry.addLine("Initializing hardware...");
        telemetry.update(); // always remember to call update()!

        // grab hardware
        this.loadHardware();

        // set motor directions
        // we have to do this because motors generally run clockwise for positive, but if all the motors run clockwise then the left ones go backwards
        this.frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        this.backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        this.frontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        this.backRight.setDirection(DcMotorSimple.Direction.FORWARD);

        // set the motors to stop themselves rather than drift
        this.frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // initialize our drive train
        MecanumDrive driveTrain = new MecanumDrive(
                new PositionableMotor(this.frontLeft, AbeConstants.DRIVE_GEAR_RATIO, AbeConstants.DRIVE_TICK_RATIO),
                new PositionableMotor(this.frontRight, AbeConstants.DRIVE_GEAR_RATIO, AbeConstants.DRIVE_TICK_RATIO),
                new PositionableMotor(this.backLeft, AbeConstants.DRIVE_GEAR_RATIO, AbeConstants.DRIVE_TICK_RATIO),
                new PositionableMotor(this.backRight, AbeConstants.DRIVE_GEAR_RATIO, AbeConstants.DRIVE_TICK_RATIO)
        );

        // we log this so that we know initialization is done
        telemetry.addLine("Initializing done.");
        telemetry.update(); // always remember to call update()!

        // waitForStart() pauses the whole program until the driver presses the play button
        waitForStart();

        // opModeIsActive() = true until the driver presses stop
        while(opModeIsActive()){
            // get our controller input
            // desired forward movement (negative because the controller returns a backwards value)
            double forward = -gamepad1.left_stick_y;
            double strafe = gamepad1.left_stick_x;
            double rot = gamepad1.right_stick_x;

            // drive
            driveTrain.driveNormal(forward, strafe, rot, 0.5);

            // and we're done!  sometimes we do logging in this, but it's not necessary yet.
        }

        // here we could put things that should happen once the opmode is stopped.
    }
}
