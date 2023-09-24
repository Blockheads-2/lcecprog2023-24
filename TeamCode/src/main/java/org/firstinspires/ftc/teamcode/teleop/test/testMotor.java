package org.firstinspires.ftc.teamcode.teleop.test;

import android.view.View;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.common.Hardware2;
import org.firstinspires.ftc.teamcode.common.HardwareDrive;

@TeleOp(name="Test motor!!", group="Drive")

public class testMotor extends OpMode {
    /* Declare OpMode members. */
    Hardware2 robot = new Hardware2();

    /** The relativeLayout field is used to aid in providing interesting visual feedback
     * in this sample application; you probably *don't* need this when you use a color sensor on your
     * robot. Note that you won't see anything change on the Driver Station, only on the Robot Controller. */
    View relativeLayout;

    @Override
    public void init() {
        robot.init(hardwareMap);

        telemetry.addData("Say", "Hello Driver");
    }

    @Override
    public void init_loop() {

    }

    @Override
    public void start() {
    }

    @Override
    public void loop() {
        UpdatePlayer1();
        UpdatePlayer2();
        UpdateButton();
        UpdateTelemetry();
    }
    void UpdatePlayer1(){
        robot.lf.setPower(gamepad1.left_stick_y);
    }

    void UpdatePlayer2(){

    }

    void UpdateTelemetry(){

        telemetry.addData("X", gamepad1.left_stick_x);
        telemetry.addData("Y", -gamepad1.left_stick_y);
        telemetry.addData("R", gamepad1.right_stick_x);

        telemetry.addData("Motor power", robot.lf.getPower());

        telemetry.update();
    }

    void UpdateButton(){

    }
}

