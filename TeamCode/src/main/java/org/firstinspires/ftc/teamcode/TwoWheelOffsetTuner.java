package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "TwoWheel Offset Tuner")
public class TwoWheelOffsetTuner extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
        TwoDeadWheelLocalizer localizer = (TwoDeadWheelLocalizer) drive.localizer;

        localizer.setOffsets(0, 0);

        telemetry.addLine("Ready. Press Play to spin.");
        telemetry.update();
        waitForStart();

        drive.updatePoseEstimate();

        double lastHeading = localizer.getPose().heading.toDouble();
        double totalHeading = 0.0;

        while (opModeIsActive() && Math.abs(totalHeading) < 2 * Math.PI) {

            drive.leftFront.setPower(-0.25);
            drive.leftBack.setPower(-0.25);
            drive.rightFront.setPower(0.25);
            drive.rightBack.setPower(0.25);

            drive.updatePoseEstimate();

            double heading = localizer.getPose().heading.toDouble();
            double delta = heading - lastHeading;
            if (delta > Math.PI)  delta -= 2 * Math.PI;
            if (delta < -Math.PI) delta += 2 * Math.PI;
            totalHeading += delta;
            lastHeading = heading;

            telemetry.addData("X (in)", localizer.getPose().position.x);
            telemetry.addData("Y (in)", localizer.getPose().position.y);
            telemetry.addData("Heading (deg)", Math.toDegrees(totalHeading));
            telemetry.update();
        }

        drive.leftFront.setPower(0);
        drive.leftBack.setPower(0);
        drive.rightFront.setPower(0);
        drive.rightBack.setPower(0);

        drive.updatePoseEstimate();
        Pose2d finalPose = localizer.getPose();

        double lateralOffsetInches = finalPose.position.x / (2 * Math.PI);
        double forwardOffsetInches = finalPose.position.y / (2 * Math.PI);

        while (opModeIsActive()) {
            telemetry.addLine("=== TUNING COMPLETE ===");
            telemetry.addData("lateralOffset (parYTicks * inPerTick)", lateralOffsetInches);
            telemetry.addData("forwardOffset (perpXTicks * inPerTick)", forwardOffsetInches);
            telemetry.update();
        }
    }
}