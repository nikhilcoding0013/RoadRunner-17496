package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@Config
@TeleOp(name = "TwoWheel Offset Auto Tuner")
public class TwoWheelOffsetAutoTuner extends LinearOpMode {

    // ===== TUNING PARAMETERS (dashboard editable) =====
    public static double K = 0.075;
    public static int ITERATIONS = 10;
    public static int SPINS_PER_ITERATION = 3;
    public static double SPIN_POWER = 0.275; // Spin motor power

    // ===== INITIAL OFFSETS (from last run or rough guess) =====
    public static double forwardOffset = -5.8;
    public static double lateralOffset = -2.4;

    @Override
    public void runOpMode() throws InterruptedException {

        FtcDashboard dashboard = FtcDashboard.getInstance();

        // Initialize MecanumDrive with TwoDeadWheelLocalizer
        Pose2d startPose = new Pose2d(0, 0, 0);
        MecanumDrive drive = new MecanumDrive(hardwareMap, startPose);

        // Apply starting offsets if localizer is TwoDeadWheel
        if (drive.localizer instanceof TwoDeadWheelLocalizer) {
            ((TwoDeadWheelLocalizer) drive.localizer).setOffsets(forwardOffset, lateralOffset);
        }

        telemetry.addLine("Two-Wheel Offset Auto Tuner Ready");
        telemetry.addLine("Robot will spin automatically");
        telemetry.update();

        waitForStart();

        for (int iter = 0; iter < ITERATIONS && opModeIsActive(); iter++) {

            double sumDeltaX = 0.0;
            double sumDeltaY = 0.0;

            for (int spinNum = 0; spinNum < SPINS_PER_ITERATION && opModeIsActive(); spinNum++) {

                // Stop robot before each spin
                drive.leftFront.setPower(0);
                drive.leftBack.setPower(0);
                drive.rightFront.setPower(0);
                drive.rightBack.setPower(0);
                sleep(200);

                // Reset pose for isolated measurement
                drive.localizer.setPose(new Pose2d(0, 0, 0));

                double lastHeading = drive.localizer.getPose().heading.toDouble();
                int halfSpins = 0; // count half rotations

                // Spin until 1 full rotation (2 half spins)
                while (opModeIsActive() && halfSpins < 2) {

                    double heading = drive.localizer.getPose().heading.toDouble();

                    // Zero-crossing detection for half spin
                    if (lastHeading < 0 && heading >= 0) halfSpins++;
                    if (lastHeading > 0 && heading <= 0) halfSpins++;

                    lastHeading = heading;

                    // Spin motors (keep same direction as before)
                    drive.leftFront.setPower(SPIN_POWER);
                    drive.leftBack.setPower(SPIN_POWER);
                    drive.rightFront.setPower(SPIN_POWER);
                    drive.rightBack.setPower(SPIN_POWER);

                    // Update pose
                    drive.updatePoseEstimate();

                    // Telemetry updates during spin
                    Pose2d pose = drive.localizer.getPose();
                    telemetry.addData("Iteration", iter + 1);
                    telemetry.addData("Spin", spinNum + 1);
                    telemetry.addData("Half Spins", halfSpins);
                    telemetry.addData("Heading (deg)", Math.toDegrees(pose.heading.toDouble()));
                    telemetry.addData("ΔX (in)", pose.position.x);
                    telemetry.addData("ΔY (in)", pose.position.y);
                    telemetry.update();

                    sleep(10);
                }

                // Stop robot after spin
                drive.leftFront.setPower(0);
                drive.leftBack.setPower(0);
                drive.rightFront.setPower(0);
                drive.rightBack.setPower(0);
                sleep(150);

                // Measure drift from spin
                Pose2d finalPose = drive.localizer.getPose();
                sumDeltaX += finalPose.position.x;
                sumDeltaY += finalPose.position.y;
            }

            // Average drift over spins
            double avgDeltaX = sumDeltaX / SPINS_PER_ITERATION;
            double avgDeltaY = sumDeltaY / SPINS_PER_ITERATION;

            // Update offsets
            forwardOffset -= K * avgDeltaX;
            lateralOffset -= K * avgDeltaY;

            // Apply new offsets immediately
            if (drive.localizer instanceof TwoDeadWheelLocalizer) {
                ((TwoDeadWheelLocalizer) drive.localizer).setOffsets(forwardOffset, lateralOffset);
            }

            telemetry.addLine("---- Iteration Complete ----");
            telemetry.addData("Avg ΔX", avgDeltaX);
            telemetry.addData("Avg ΔY", avgDeltaY);
            telemetry.addData("forwardOffset", forwardOffset);
            telemetry.addData("lateralOffset", lateralOffset);
            telemetry.update();

            sleep(500);
        }

        telemetry.addLine("TUNING COMPLETE");
        telemetry.addData("FINAL forwardOffset", forwardOffset);
        telemetry.addData("FINAL lateralOffset", lateralOffset);
        telemetry.update();

        // Keep alive for dashboard viewing
        while (opModeIsActive()) {
            sleep(50);
        }
    }
}
