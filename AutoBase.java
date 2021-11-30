package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.*;

import java.util.Locale;

/**
 * AutoPathController is an abstraction of the drivetrain. Use this for Autos
 * where a predefined path needs to be executed.
 */
public class AutoBase {
    DcMotor left_front, right_front, left_back, right_back;

    Telemetry telemetry;

    // https://first-tech-challenge.github.io/SkyStone/com/qualcomm/hardware/bosch/BNO055IMUImpl.html
    BNO055IMU imu;
    Orientation angles;
    Acceleration gravity;

    // Orientation relative to signing the top of the field.
    // https://github.com/acmerobotics/road-runner/blob/master/gui/src/main/resources/field.png
    double currentAngle;

    static final ElapsedTime runtime = new ElapsedTime();

    // Drive speed constants
    static final double DRIVE_SPEED = 1.0;
    static final double TURN_SPEED = 1.0;


    static final double     HEADING_THRESHOLD       = 1 ;      // As tight as we can make it with an integer gyro
    static final double     P_TURN_COEFF            = 0.1;     // Larger is more responsive, but also less stable
    static final double     P_DRIVE_COEFF           = 0.15;     // Larger is more responsive, but also less stable
    static final double LEFT_DAMPING_CONSTANT = 0.885;
    static final double RIGHT_DAMPING_CONSTANT = 0.885;
    static final double DISTANCE_TO_ANGLE_CONSTANT = 1.2;

    // Motor encoder configuration constant
    static final double TETRIX_TICKS_PER_MOTOR_REV = 1440;
    static final double ANDYMARK_TICKS_PER_MOTOR_REV = 1120;
    static final double PULSES_PER_REVOLUTION = ANDYMARK_TICKS_PER_MOTOR_REV;
    static final double WHEEL_DIAMETER_IN = 4;
    static final double PULSES_PER_IN = PULSES_PER_REVOLUTION / (WHEEL_DIAMETER_IN * 3.1415);

    public void InitAuto(
            HardwareMap hardwareMap,
            String left_front_name,
            String right_front_name,
            String left_back_name,
            String right_back_name,
            Telemetry telemetry
    ) {
        initIMU(hardwareMap);

        left_front = hardwareMap.get(DcMotor.class, left_front_name);
        right_front = hardwareMap.get(DcMotor.class, right_front_name);
        left_back = hardwareMap.get(DcMotor.class, left_back_name);
        right_back = hardwareMap.get(DcMotor.class, right_back_name);

        right_front.setDirection(DcMotor.Direction.REVERSE);
        right_back.setDirection(DcMotor.Direction.REVERSE);

        this.telemetry = telemetry;

        ZeroPowerBehaviorBRAKE();
    }

    // public void update() {
    //     angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
    //     gravity = imu.getGravity();
    //     telemetry.addData("Angles: ", angles.toString());
    //     telemetry.addData("Gravity: ", gravity.toString());
    // }

    private static void initIMU(HardwareMap hardwareMap) {
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);
    }

    private static void ZeroPowerBehaviorBRAKE() {
        left_front.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right_front.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        left_back.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right_back.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    private static void setRunUsingEncoders() {
        left_front.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
        right_front.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
        left_back.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
        right_back.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
    }

    private static void setRunWithoutEncoders() {
        left_front.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODERS);
        right_front.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODERS);
        left_back.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODERS);
        right_back.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODERS);
    }

    private static void setRunToPosition() {
        left_front.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        right_front.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        left_back.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        right_back.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    private static void resetEncoders() {
        // method may be stopandreset, but I'm not sure
        left_front.setMode(DcMotor.RunMode.RESET_ENCODERS);
        right_front.setMode(DcMotor.RunMode.RESET_ENCODERS);
        left_back.setMode(DcMotor.RunMode.RESET_ENCODERS);
        right_back.setMode(DcMotor.RunMode.RESET_ENCODERS);
    }
    
    private static void setTargetPositionsForward(int distance) {
        left_front.setTargetPosition(distance);
        right_front.setTargetPosition(distance);
        left_back.setTargetPosition(distance);
        right_back.setTargetPosition(distance);
    }

    private static void setTargetPositionsBackward(int distance) {
        left_front.setTargetPosition(-distance);
        right_front.setTargetPosition(-distance);
        left_back.setTargetPosition(-distance);
        right_back.setTargetPosition(-distance);
    }

    private static void setTargetPositionsLeft(int distance) {
        left_front.setTargetPosition(-distance);
        right_front.setTargetPosition(distance);
        left_back.setTargetPosition(distance);
        right_back.setTargetPosition(-distance);
    }

    private static void setTargetPositionsRight(int distance) {
        left_front.setTargetPosition(distance);
        right_front.setTargetPosition(-distance);
        left_back.setTargetPosition(-distance);
        right_back.setTargetPosition(distance);
    }

    private static void stopDriving() {
        left_front.setPower(0);
        right_front.setPower(0);
        left_back.setPower(0);
        right_back.setPower(0);
    }

    private static void goForward(double MotorPower) {
        left_front.setPower(MotorPower);
        right_front.setPower(MotorPower);
        left_back.setPower(MotorPower);
        right_back.setPower(MotorPower);
    }

    private static void goBackward(double MotorPower) {
        left_front.setPower(-MotorPower);
        right_front.setPower(-MotorPower);
        left_back.setPower(-MotorPower);
        right_back.setPower(-MotorPower);
    }

    private static void goLeft(double MotorPower) {
        left_front.setPower(-MotorPower);
        right_front.setPower(MotorPower);
        left_back.setPower(MotorPower);
        right_back.setPower(-MotorPower);
    }

    private static void goRight(double MotorPower) {
        left_front.setPower(MotorPower);
        right_front.setPower(-MotorPower);
        left_back.setPower(-MotorPower);
        right_back.setPower(MotorPower);
    }
    
    public void driveForward(double MotorPower, int distanceIN) {
        if (MotorPower == (double) -2.0) {
            MotorPower = DRIVE_SPEED;
        }
        resetEncoders();
        setTargetPositionsForward(distanceIN);
        setRunToPosition();
        goForward(MotorPower);
        while (
            left_front.isBusy() &&
            right_front.isBusy() &&
            left_back.isBusy() &&
            right_back.isBusy()
        ) {
            // waiting for target position to be reached
        }
        stopDriving();
        setRunUsingEncoders();
    }
    
    public void driveBackward(double MotorPower, int distanceIN) {
        if (MotorPower == (double) -2.0) {
            MotorPower = DRIVE_SPEED;
        }
        resetEncoders();
        setTargetPositionsBackward(distanceIN);
        setRunToPosition();
        goBackward(MotorPower);
        while (
            left_front.isBusy() &&
            right_front.isBusy() &&
            left_back.isBusy() &&
            right_back.isBusy()
        ) {
            // waiting for target position to be reached
        }
        stopDriving();
        setRunUsingEncoders();
    }
    
    public void strafeLeft(double MotorPower, int distanceIN) {
        if (MotorPower == (double) -2.0) {
            MotorPower = DRIVE_SPEED;
        }
        resetEncoders();
        setTargetPositionsLeft(distanceIN);
        setRunToPosition();
        goLeft(MotorPower);
        while (
            left_front.isBusy() &&
            right_front.isBusy() &&
            left_back.isBusy() &&
            right_back.isBusy()
        ) {
            // waiting for target position to be reached
        }
        stopDriving();
        setRunUsingEncoders();
    }
    
    public void strafeRight(double MotorPower, int distanceIN) {
        if (MotorPower == (double) -2.0) {
            MotorPower = DRIVE_SPEED;
        }
        resetEncoders();
        setTargetPositionsRight(distanceIN);
        setRunToPosition();
        goRight(MotorPower);
        while (
            left_front.isBusy() &&
            right_front.isBusy() &&
            left_back.isBusy() &&
            right_back.isBusy()
        ) {
            // waiting for target position to be reached
        }
        stopDriving();
        setRunUsingEncoders();
    }
}

/*
    during init:
    initAuto()

    During run loop:
    1. driveForward
    2. driveBackward
    3. strafeLeft
    4. strafeRight
    5. strafeNW
    6. strafeNE
    7. strafeSE
    8. strafeSW
    9. turnLeft
    10. turnRight
*/
