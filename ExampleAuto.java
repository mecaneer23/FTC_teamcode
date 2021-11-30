package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.AutoBase;

@Autonomous
public class ExampleAuto extends LinearOpMode {
    @Override
    public void runOpMode() {
        AutoBase auto = new AutoBase();
        auto.InitAuto(
            hardwareMap,
            "frontLeft",
            "frontRight",
            "backLeft",
            "backRight",
            telemetry,
            (double) 18, //tbd
            (double) 18 //tbd
        );
        waitForStart();
        if (opModeIsActive()) {
            /* 
            feel free to incorporate calls to the following:
            sleep(time);
            auto.driveForward(distanceIN, (double) 2);
            auto.driveBackward(distanceIN, (double) 2);
            auto.strafeLeft(distanceIN, (double) 2);
            auto.strafeRight(distanceIN, (double) 2);
            auto.strafeNW(distanceIN, (double) 2);
            auto.strafeNE(distanceIN, (double) 2);
            auto.strafeSW(distanceIN, (double) 2);
            auto.strafeSE(distanceIN, (double) 2);
            auto.turnLeft(degrees, (double) 2);
            auto.turnRight(degrees, (double) 2);
            */
        }
    }
}