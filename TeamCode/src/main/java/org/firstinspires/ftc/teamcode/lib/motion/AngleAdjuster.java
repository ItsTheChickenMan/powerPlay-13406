package org.firstinspires.ftc.teamcode.lib.motion;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.TouchSensor;

public class AngleAdjuster extends PositionableMotor {
    private TouchSensor limitSwitch;

    public AngleAdjuster(DcMotorEx motor, double gearRatio, double tickRatio, TouchSensor limitSwitch) {
        // super constructor
        super(motor, gearRatio, tickRatio);

        // limit switch
        this.limitSwitch = limitSwitch;
    }

    public AngleAdjuster(DcMotorEx motor, double gearRatio, double tickRatio, double lowerLimit, double upperLimit, TouchSensor limitSwitch) {
        // super constructor
        super(motor, gearRatio, tickRatio, lowerLimit, upperLimit);

        // limit switch
        this.limitSwitch = limitSwitch;
    }

    /**
     * @brief Call this every frame to determine if we have to shut off
     */
    public void check(){
        if(this.limitSwitch.isPressed()){
            this.disable();
        }
    }
}
