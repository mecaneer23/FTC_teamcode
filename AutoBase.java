package org.firstinspires.ftc.teamcode.Auto;

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

    // Motor encoder configuration constants
    static final double PULSES_PER_REVOLUTION = 140;
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

    }

    public void update() {
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        gravity = imu.getGravity();
        telemetry.addData("Angles: ", angles.toString());
        telemetry.addData("Gravity: ", gravity.toString());
    }

    public void initIMU(HardwareMap hardwareMap) {
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

    public void setZeroPowerBehavior() {
        left_front.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right_front.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        left_back.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right_back.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void setRunUsingEncoder() {
        left_front.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        right_front.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        left_back.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        right_back.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void setRunToPosition() {
        left_front.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        right_front.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        left_back.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        right_back.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void setZeroPower() {
        left_front.setPower(0);
        right_front.setPower(0);
        left_back.setPower(0);
        right_back.setPower(0);
    }

    public void

    public void forward(int distanceIN) {
        while ((PULSES_PER_IN * PULSES_PER_REVOLUTION) != distanceIN) {
            left_front.setPower(DRIVE_SPEED);
            right_front.setPower(DRIVE_SPEED);
            left_back.setPower(DRIVE_SPEED);
            right_back.setPower(DRIVE_SPEED);
        }
    }
}

/*
    Need 10 methods:
    Forward, Backward, Left, Right
    Diagonal (for any direction), Rotation (Left and right)
*/
