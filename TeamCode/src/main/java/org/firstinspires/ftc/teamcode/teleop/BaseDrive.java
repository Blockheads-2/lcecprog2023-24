package org.firstinspires.ftc.teamcode.teleop;

import android.view.View;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.common.Button;
import org.firstinspires.ftc.teamcode.common.Constants;
import org.firstinspires.ftc.teamcode.common.HardwareDrive;

@TeleOp(name="Base Drive", group="Drive")
//@Disabled
public class BaseDrive extends OpMode{
    /* Declare OpMode members. */
    HardwareDrive robot = new HardwareDrive();
    private ElapsedTime runtime = new ElapsedTime();

    /** The relativeLayout field is used to aid in providing interesting visual feedback
     * in this sample application; you probably *don't* need this when you use a color sensor on your
     * robot. Note that you won't see anything change on the Driver Station, only on the Robot Controller. */
    View relativeLayout;

    @Override
    public void init() {
        robot.init(hardwareMap);

        telemetry.addData("Say", "Hello Driver");
        runtime.reset();
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
//        UpdatePlayer2();
        UpdateButton();
        UpdateTelemetry();
    }
    void UpdatePlayer1(){
        double drivePower = DriveTrainSpeed();

        DriveTrainBase(drivePower);
        DriveTrainSpeed();
        DriveMicroAdjust(0.4);
    }

    void UpdatePlayer2(){

    }

    void UpdateTelemetry(){

        telemetry.addData("X", gamepad1.left_stick_x);
        telemetry.addData("Y", -gamepad1.left_stick_y);
        telemetry.addData("R", gamepad1.right_stick_x);

        telemetry.update();
    }

    void UpdateButton(){

    }

    void DriveTrainBase(double drivePower){
        double directionX = Math.pow(gamepad1.left_stick_x, 1); //Strafe
        double directionY = -Math.pow(gamepad1.left_stick_y, 1); //Forward
        double directionR = Math.pow(gamepad1.right_stick_x, 1); //Turn


        robot.lf.setPower((directionY + directionR + directionX) * drivePower * 0.5);
//        robot.rf.setPower((directionY - directionR - directionX) * drivePower);
        robot.lb.setPower((directionY + directionR - directionX) * drivePower * 0.5);
//        robot.rb.setPower((directionY - directionR + directionX) * drivePower);

    }

    void DriveMicroAdjust(double power){
        if (gamepad1.dpad_up){
            robot.lf.setPower(power);
//            robot.rf.setPower(power);
            robot.lb.setPower(power);
//            robot.rb.setPower(power);
        }
        else if (gamepad1.dpad_down){
            robot.lf.setPower(-power);
//            robot.rf.setPower(-power);
            robot.lb.setPower(-power);
//            robot.rb.setPower(-power);
        }
        else if (gamepad1.dpad_right){
            robot.lf.setPower(power);
//            robot.rf.setPower(-power);
            robot.lb.setPower(-power);
//            robot.rb.setPower(power);
        }
        else if (gamepad1.dpad_left){
            robot.lf.setPower(-power);
//            robot.rf.setPower(power);
            robot.lb.setPower(power);
//            robot.rb.setPower(-power);
        }

        if (gamepad1.left_trigger == 1){
            robot.lf.setPower(-power);
//            robot.rf.setPower(power);
            robot.lb.setPower(-power);
//            robot.rb.setPower(power);
        }
        else if (gamepad1.right_trigger == 1){
            robot.lf.setPower(power);
//            robot.rf.setPower(-power);
            robot.lb.setPower(power);
//            robot.rb.setPower(-power);
        }
    }

    double DriveTrainSpeed(){
        double drivePower = 0.75;

        if (gamepad1.right_bumper)
            drivePower = 1;
        else if (gamepad1.left_bumper)
            drivePower = 0.25;


        return drivePower;
    }
    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }
}
