package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

import static org.firstinspires.ftc.teamcode.drive.ThoborVARS.putere_dpad;
import static org.firstinspires.ftc.teamcode.drive.ThoborVARS.tragaciInchis;

@TeleOp(name = "TeleOP O Roata", group = "TeleOP")
@Disabled
public class TELEOP_DEBUG extends LinearOpMode {

    public double tragaciPus = 0;
    public double organizatorPus = 0;
    double buttonReleased = 1;
    double buttonReleased2 = 1;
    double v1, v2, v3, v4;
    @Override
    public void runOpMode() {

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        telemetry.clearAll();
        telemetry.update();
        AsteaptaStart();
        drive.TragaciPozitie(tragaciInchis);
        while (opModeIsActive())
        {
            //=======[miscarea din localization]=====
            drive.setWeightedDrivePower(
                    new Pose2d(
                            -gamepad1.left_stick_y,
                            -gamepad1.left_stick_x,
                            -gamepad1.right_stick_x
                    )
            );

            double x = 1;
            while(gamepad1.dpad_down)
            {
                v1 = (-putere_dpad);
                v2 = (-putere_dpad);
                v3 = (-putere_dpad);
                v4 = (-putere_dpad);
                drive.setMotorPowers(v1*x, 0,0,0);
            }
            while(gamepad1.dpad_right)
            {
                v1 = (+putere_dpad);
                v2 = (-putere_dpad);
                v3 = (-putere_dpad);
                v4 = (putere_dpad);
                drive.setMotorPowers(0, v3 * x,0,0);
            }
            while(gamepad1.dpad_up)
            {
                v1 = (putere_dpad);
                v2 = (putere_dpad);
                v3 = (putere_dpad);
                v4 = (putere_dpad);
                drive.setMotorPowers(0, 0,v4*x,0);
            }
            while(gamepad1.dpad_left)
            {
                v1 = (-putere_dpad);
                v2 = (putere_dpad);
                v3 = (putere_dpad);
                v4 = (-putere_dpad);
                drive.setMotorPowers(0, 0,0,v2*x);
            }

            drive.update();
            Pose2d poseEstimate = drive.getPoseEstimate();
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", poseEstimate.getHeading());
            telemetry.update();

        }
    }
    public void AsteaptaStart() // pentru eroarea cu Motorola
    {
        while (!opModeIsActive() && !isStopRequested())
        {
            telemetry.update();
        }
    }
}
  