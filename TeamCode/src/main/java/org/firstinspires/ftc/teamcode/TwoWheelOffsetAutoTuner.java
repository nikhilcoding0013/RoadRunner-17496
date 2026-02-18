package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "TwoWheel Offset Auto Tuner")
public class TwoWheelOffsetAutoTuner extends LinearOpMode {

    public static double SPIN_POWER = 0.25;
    public static int SPINS = 5;

    // Plug in values from single-spin tuner here
    public static double lateralOffset = 0.0;
    public static double forwardOffset = 0.0;

    @Override
    public void runOpMode() throws InterruptedException {

        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
        TwoDeadWheelLocalizer localizer = (TwoDeadWheelLocalizer) drive.localizer;

        telemetry.addLine("Ready. Press Play to begin.");
        telemetry.update();
        waitForStart();

        double totalLateralCorrection = 0.0;
        double totalForwardCorrection = 0.0;

        for (int spin = 0; spin < SPINS && opModeIsActive(); spin++) {

            // Apply current best offsets
            localizer.setOffsets(forwardOffset, lateralOffset);

            // Stop and settle
            setMotors(drive, 0, 0);
            sleep(300);

            // Reset pose
            localizer.setPose(new Pose2d(0, 0, 0));
            drive.updatePoseEstimate();

            double lastHeading = localizer.getPose().heading.toDouble();
            double totalHeading = 0.0;

            // Spin one full rotation
            while (opModeIsActive() && Math.abs(totalHeading) < 2 * Math.PI) {

                setMotors(drive, -SPIN_POWER, SPIN_POWER);
                drive.updatePoseEstimate();

                double heading = localizer.getPose().heading.toDouble();
                double delta = heading - lastHeading;
                if (delta > Math.PI)  delta -= 2 * Math.PI;
                if (delta < -Math.PI) delta += 2 * Math.PI;
                totalHeading += delta;
                lastHeading = heading;

                telemetry.addData("Spin", spin + 1 + " / " + SPINS);
                telemetry.addData("Heading (deg)", Math.toDegrees(totalHeading));
                telemetry.addData("X (in)", localizer.getPose().position.x);
                telemetry.addData("Y (in)", localizer.getPose().position.y);
                telemetry.update();
            }

            setMotors(drive, 0, 0);
            sleep(200);
            drive.updatePoseEstimate();

            Pose2d finalPose = localizer.getPose();

            // Calculate correction from residual drift
            double lateralCorrection = finalPose.position.x / (2 * Math.PI);
            double forwardCorrection = finalPose.position.y / (2 * Math.PI);

            totalLateralCorrection += lateralCorrection;
            totalForwardCorrection += forwardCorrection;

            // Apply correction immediately for next spin
            lateralOffset += lateralCorrection;
            forwardOffset += forwardCorrection;

            telemetry.addLine("--- Spin " + (spin + 1) + " complete ---");
            telemetry.addData("Residual X", finalPose.position.x);
            telemetry.addData("Residual Y", finalPose.position.y);
            telemetry.addData("lateralOffset", lateralOffset);
            telemetry.addData("forwardOffset", forwardOffset);
            telemetry.update();
            sleep(500);
        }

        while (opModeIsActive()) {
            telemetry.addLine("=== TUNING COMPLETE ===");
            telemetry.addData("FINAL lateralOffset", lateralOffset);
            telemetry.addData("FINAL forwardOffset", forwardOffset);
            telemetry.update();
        }
    }

    private void setMotors(MecanumDrive drive, double left, double right) {
        drive.leftFront.setPower(left);
        drive.leftBack.setPower(left);
        drive.rightFront.setPower(right);
        drive.rightBack.setPower(right);
    }
}