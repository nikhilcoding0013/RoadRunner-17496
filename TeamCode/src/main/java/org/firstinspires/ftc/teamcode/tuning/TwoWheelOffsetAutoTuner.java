package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@Config
@TeleOp(name = "TwoWheel Offset Auto Tuner")
public class TwoWheelOffsetAutoTuner extends LinearOpMode {

    // ===== TUNING PARAMETERS (dashboard editable) =====
    public static double K = 0.075;
    public static int ITERATIONS = 10;
    public static int SPINS_PER_ITERATION = 3;
    public static double SPIN_POWER = 0.12;

    // ===== INITIAL OFFSETS (from last run or rough guess) =====
    public static double forwardOffset = -5.8;
    public static double lateralOffset = -2.4;

    @Override
    public void runOpMode() throws InterruptedException {

        FtcDashboard dashboard = FtcDashboard.getInstance();

        // Initialize localizer + drive
        TwoWheelLocalizer localizer = new TwoWheelLocalizer(hardwareMap);
        MecanumDrive drive = new MecanumDrive(hardwareMap, localizer);

        // Apply starting offsets
        localizer.setOffsets(forwardOffset, lateralOffset);

        telemetry.addLine("Two-Wheel Offset Auto Tuner Ready");
        telemetry.addLine("Robot will spin automatically");
        telemetry.update();

        waitForStart();

        for (int iter = 0; iter < ITERATIONS && opModeIsActive(); iter++) {

            double sumDeltaX = 0.0;
            double sumDeltaY = 0.0;

            for (int spin = 0; spin < SPINS_PER_ITERATION && opModeIsActive(); spin++) {

                // Ensure robot is fully stopped
                drive.setDrivePower(new Pose2d());
                sleep(200);

                // Reset pose for isolated measurement
                localizer.setPose(new Pose2d());

                double startHeading = localizer.getPose().getHeading();

                // Spin until ~360 degrees rotation
                while (opModeIsActive() &&
                        Math.abs(localizer.getPose().getHeading() - startHeading)
                                < Math.toRadians(360)) {

                    drive.setDrivePower(new Pose2d(0, 0, SPIN_POWER));
                    drive.update();
                    localizer.update();

                    sleep(10);
                }

                // Stop robot
                drive.setDrivePower(new Pose2d());
                sleep(150);

                // Measure drift
                Pose2d pose = localizer.getPose();
                sumDeltaX += pose.getX();
                sumDeltaY += pose.getY();

                telemetry.addData("Iteration", iter + 1);
                telemetry.addData("Spin", spin + 1);
                telemetry.addData("ΔX (in)", pose.getX());
                telemetry.addData("ΔY (in)", pose.getY());
                telemetry.addData("Heading (deg)",
                        Math.toDegrees(pose.getHeading()));
                telemetry.update();

                sleep(250);
            }

            // Average drift over spins
            double avgDeltaX = sumDeltaX / SPINS_PER_ITERATION;
            double avgDeltaY = sumDeltaY / SPINS_PER_ITERATION;

            // Update offsets (sign may be flipped if divergence occurs)
            forwardOffset -= K * avgDeltaX;
            lateralOffset -= K * avgDeltaY;

            // Apply new offsets immediately
            localizer.setOffsets(forwardOffset, lateralOffset);

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
