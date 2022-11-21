package org.firstinspires.ftc.teamcode.lib.drive;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.lib.motion.PositionableMotor;

/**
 * @brief Basic mecanum drive
 *
 * @deprecated use roadrunner instead
 */
public class MecanumDrive {
    // the motors for this drive train.
    public PositionableMotor frontLeft;
    public PositionableMotor frontRight;
    public PositionableMotor backLeft;
    public PositionableMotor backRight;

    /**
     * @brief Construct a mecanum drive from four pre-constructed positionable motors
     *
     * @note THIS DOES NOT SET DIRECTIONS FOR YOU!  Set them yourself.
     *
     * @param frontLeft
     * @param frontRight
     * @param backLeft
     * @param backRight
     */
    public MecanumDrive(PositionableMotor frontLeft, PositionableMotor frontRight, PositionableMotor backLeft, PositionableMotor backRight) {
        this.frontLeft = frontLeft;
        this.frontRight = frontRight;
        this.backLeft = backLeft;
        this.backRight = backRight;
    }

    /**
     * @brief Construct a mecanum drive from initialized motors assuming that all of them have the same gear and tick ratios
     *
     * @param frontLeft
     * @param frontRight
     * @param backLeft
     * @param backRight
     * @param gearRatio motor shaft rotations per driven gear rotations
     * @param tickRatio ticks per rotation
     */
    public MecanumDrive(DcMotorEx frontLeft, DcMotorEx frontRight, DcMotorEx backLeft, DcMotorEx backRight, double gearRatio, double tickRatio){
        this.frontLeft = new PositionableMotor(frontLeft, gearRatio, tickRatio);
        this.frontRight = new PositionableMotor(frontRight, gearRatio, tickRatio);
        this.backLeft = new PositionableMotor(backLeft, gearRatio, tickRatio);
        this.backRight = new PositionableMotor(backRight, gearRatio, tickRatio);
    }

    /**
     * @brief Set the motor powers for each motor individually
     *
     * @param fl power for frontLeft
     * @param fr power for frontRight
     * @param bl power for backLeft
     * @param br power for backRight
     */
    public void setAllPowers(double fl, double fr, double bl, double br){
        // use .setPower(power) to set each motor's power from -1 to 1
        // the setPower method already clamps the value for us, so we don't need to!
        // Fun Fact: I spend about 2 hours trying to fix a bug that turned out to source from me mixing up "fl" and "fr" in this method.  pay attention!
        this.frontLeft.setPower(fl);
        this.frontRight.setPower(fr);
        this.backLeft.setPower(bl);
        this.backRight.setPower(br);
    }

    /**
     * @brief Drive the drive train with normal mecanum logic
     *
     * This can also be referred to as relative drive, since the movement of the robot is relative to the robot's orientation
     *
     * @param forward desired y (forward/back) movement (bot relative)
     * @param strafe desired x (side/side) movement (bot relative)
     * @param rot rotation (bot relative)
     */
    public void driveNormal(double forward, double strafe, double rot){
        this.driveNormal(forward, strafe, rot, 1.0);
    }

    /**
     * @brief Drive the drive train with normal mecanum logic + a speed coefficient
     *
     * Speed coefficient should be a percentage.  No limits to the speed.
     *
     * @param forward amount of desired forward movement
     * @param strafe amount of desired strafe movement
     * @param rot amount of desired rotation
     * @param speed percentage of total power
     */
    public void driveNormal(double forward, double strafe, double rot, double speed){
        // calculate power values for each motor
        // NOTE: these might be tough to understand, but they should make intuitive sense when you watch the robot move in certain directions.
        // each value represents the power, between 1 and -1, of each their respective motors (fl = frontLeft, etc.)
        // think of each axis as the desired movement in that direction (it's expected to be a value between -1 and 1).  forward corresponds to desired forward/backward movement and strafe corresponds to desired left/right movement.
        // for the bot to go forward, all motors need to be powered in the same direction, so the first term of each value is positive forward.
        // for the bot to rotate, the left motors need to run opposite of the right motors, so we subtract the desired rotation from the left motors while adding it to the right motors
        // the strafing is a little more weird.  Think about it as each wheel diagonal to another wheel forming a pair with that wheel, and then the two pairs of diagonal wheels run opposite of each other.
        // when combined, we get our drive system!
        double fl = forward + strafe - rot;
        double fr = forward - strafe + rot;
        double bl = forward - strafe - rot;
        double br = forward + strafe + rot;

        // normalize powers
        /*double largest = Math.max(Math.max(Math.max(fl, fr), bl), br);

        // prevent divide by 0
        if(largest != 0) {
            fl /= largest;
            fr /= largest;
            bl /= largest;
            br /= largest;
        }*/

        fl *= speed;
        fr *= speed;
        bl *= speed;
        br *= speed;

        // down here, we simply assign the powers to the motors
        // this is down through the setPower method, which takes a value from -1 to 1 (and clamps it automatically for us, how sweet)
        // however, setting each power individually is annoying, so we use a method that we wrote ourselves to do it a little quicker:
        // if you see a method I wrote and you don't know how it works, chances are that I documented it, so look around for it!  I'll try and tell you where it is too (this one is right below the constructor)
        this.setAllPowers(fl, fr, bl, br);
    }

    /**
     * @brief Drive the drive train with field oriented drive logic
     *
     * "Field oriented drive" is when the robot drives relative to the field's axes.
     * This means that the "desired forward movement" not longer corresponds to which direction the bot is facing, but rather which direction the driver is facing (more or less, unless the driver moves around).
     *
     * @fixme implement
     *
     * @param forward
     * @param strafe
     * @param rot
     */
    public void driveFieldOriented(double forward, double strafe, double rot, double orientation, double speed) {
        // FIXME: implement
    }
}