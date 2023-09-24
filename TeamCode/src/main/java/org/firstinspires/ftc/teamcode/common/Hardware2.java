package org.firstinspires.ftc.teamcode.common;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Hardware2 {
    public DcMotorEx lf;

    HardwareMap hwMap           =  null;

    /* Constructor */
    public Hardware2(){

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and Initialize Motors

        lf = hwMap.get(DcMotorEx.class, "left_front");

        //Set Motor Directions
        lf.setDirection(DcMotorSimple.Direction.FORWARD);

        // Set all motors to zero power
        setMotorPower(0);

        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        setRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setRunMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void setMotorPower(double power){
        if (power == 0.0){
            // Grady Conwell Was Here
            lf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        } else {
            lf.setPower(0);
        }
    }

    public void setRunMode(DcMotor.RunMode runState){
        lf.setMode(runState);
        //make sure to not add arm here
    }

}
