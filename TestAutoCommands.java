package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.AutoBase;

@Autonomous
public class TestAutoCommands extends LinearOpMode {
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
            telemetry.addData("Every Direction 12in", "");
            telemetry.update();
            telemetry.addData("sleep", "starting sleeping");
            telemetry.update();
            sleep(1);
            telemetry.addData("sleep", "done sleeping");
            telemetry.update();
            // auto.driveForward(12, (double) 2);
            // auto.driveBackward(12, (double) 2);
            // auto.strafeLeft(12, (double) 2);
            // auto.strafeRight(12, (double) 2);
            // auto.strafeNW(12, (double) 2);
            // auto.strafeNE(12, (double) 2);
            // auto.strafeSW(12, (double) 2);
            // auto.strafeSE(12, (double) 2);
            // auto.turnLeft(90, (double) 2);
            // auto.turnRight(90, (double) 2);
            auto.driveForward(12);
            auto.driveBackward(12);
            auto.strafeLeft(12);
            auto.strafeRight(12);
            auto.strafeNW(12);
            auto.strafeNE(12);
            auto.strafeSW(12);
            auto.strafeSE(12);
            auto.turnLeft(90);
            auto.turnRight(90);
        }
    }
}