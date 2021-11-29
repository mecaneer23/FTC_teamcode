/*
Control Scheme:
  Gamepad 1 - robot locomotion:
    left stick - xy position of robot
    right stick - rotation of robot
    right bumper 1/2 speed slowmode
    dpad - 1.0 power in any given direction
*/

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp
public class FreightFrenzy extends LinearOpMode {

    private DcMotor frontLeft;
    private DcMotor backLeft;
    private DcMotor frontRight;
    private DcMotor backRight;
    private DcMotor CarouselSpinner;

    @Override
    public void runOpMode() {
        float x;
        float y;
        float clockwise;
        double fl;
        double fr;
        double bl;
        double br;
	    
        frontLeft = hardwareMap.get(DcMotor.class,
            "frontLeft");
        backLeft = hardwareMap.get(DcMotor.class,
            "backLeft");
        frontRight = hardwareMap.get(DcMotor.class,
            "frontRight");
        backRight = hardwareMap.get(DcMotor.class,
            "backRight");
        CarouselSpinner = hardwareMap.get(DcMotor.class,
            "CarouselSpinner");
        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.REVERSE);
        CarouselSpinner.setDirection(DcMotor.Direction.REVERSE);

        waitForStart();
        if (opModeIsActive()) {
            frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            CarouselSpinner.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            while (opModeIsActive()) {
                x = gamepad1.left_stick_x;
                y = gamepad1.left_stick_y;
                clockwise = gamepad1.right_stick_x;

                if (gamepad1.dpad_up) {
                    y = (float) - 1.0;
                }
                if (gamepad1.dpad_down) {
                    y = (float) 1.0;
                }
                if (gamepad1.dpad_right) {
                    x = (float) 1.0;
                }
                if (gamepad1.dpad_left) {
                    x = (float) - 1.0;
                }

                fl = y - x - clockwise;
                fr = y + x + clockwise;
                bl = y + x - clockwise;
                br = y - x + clockwise;
                
                if (gamepad1.right_bumper) {
                    fl /= 2;
                    fr /= 2;
                    bl /= 2;
                    br /= 2;
                } 
                    
                frontLeft.setPower(fl);
                frontRight.setPower(fr);
                backLeft.setPower(bl);
                backRight.setPower(br);

                if (gamepad1.a) {
                    CarouselSpinner.setPower((float)0.95);
                } else if (gamepad1.b) {
                    CarouselSpinner.setPower((float)-0.95);
                } else {
                    CarouselSpinner.setPower((float)0);                  
                }

                telemetry.update();
            }
        }
    }
}
