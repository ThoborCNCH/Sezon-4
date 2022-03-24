/*
 *
 * ©Thobor 2020-2021
 *
 *           _
 *       .__(.)< (MEOW)
 *        \___)
 * ~~~~~~~~~~~~~~~~~~
 *
 *
 */

package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

import static org.firstinspires.ftc.teamcode.HackGoogle.NuUmbla.ARUNCARE_NORMALA;
import static org.firstinspires.ftc.teamcode.drive.ThoborVARS.*;

@TeleOp(group = "Official")
@Disabled

public class TeleOP_Autonom extends LinearOpMode {

    public double tragaciPus = 0;
    public double organizatorPus = 0;
    public double fatetePuse = 0;
    public double ghearaPusa = 0;
    double buttonReleased = 1;
    double buttonReleased2 = 1;
    double buttonReleased3 = 1;
    double buttonReleased5 = 1;
    double epureFata = 1;


    double v1, v2, v3, v4;

    // Define 2 states, drive control or automatic control
    enum Mode {
        DRIVER_CONTROL,
        AUTOMATIC_CONTROL
    }

    Mode currentMode = Mode.DRIVER_CONTROL;


    @Override
    public void runOpMode() throws InterruptedException {

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        telemetry.clearAll();
        telemetry.update();

        /*
         * in autonom trebuie pus asta: TODO: LocalizareThobor.currentPose = drive.getPoseEstimate();
         * cu linia de mai sus pusa, putem sa ne pastram locatia din autonom in TeleOP pentru
         * a putea urmari robotul in continuare
         */
        drive.setPoseEstimate(LocalizareThobor.currentPose);

        AsteaptaStart();

        if (isStopRequested()) return;

        while (opModeIsActive() && !isStopRequested()) {
            drive.update();
            switch (currentMode) {
                case DRIVER_CONTROL:
                    drive.setWeightedDrivePower(
                            new Pose2d(
                                    -gamepad1.left_stick_y * 0.9,
                                    -gamepad1.left_stick_x * 0.9,
                                    -gamepad1.right_stick_x * 0.9
                            )
                    );
                    while (gamepad1.dpad_down) {
                        v1 = (-putere_dpad);
                        v2 = (-putere_dpad);
                        v3 = (-putere_dpad);
                        v4 = (-putere_dpad);
//                      drive.setMotorPowers(v1 * full_power, v3 * full_power, v4 * full_power, v2 * full_power);
                        drive.setMotorPowersFull(v1, v3, v4, v2);
                    }
                    while (gamepad1.dpad_right) {
                        v1 = (+putere_dpad);
                        v2 = (-putere_dpad);
                        v3 = (-putere_dpad);
                        v4 = (putere_dpad);
                        drive.setMotorPowersFull(v1, v3, v4, v2);
                    }
                    while (gamepad1.dpad_up) {
                        v1 = (putere_dpad);
                        v2 = (putere_dpad);
                        v3 = (putere_dpad);
                        v4 = (putere_dpad);
                        drive.setMotorPowersFull(v1, v3, v4, v2);
                    }
                    while (gamepad1.dpad_left) {
                        v1 = (-putere_dpad);
                        v2 = (putere_dpad);
                        v3 = (putere_dpad);
                        v4 = (-putere_dpad);
                        drive.setMotorPowersFull(v1, v3, v4, v2);
                    }

                    while (gamepad1.left_trigger != 0) {
                        drive.setMotorPowers(-putere_rotire, -putere_rotire, putere_rotire, putere_rotire);
                    }

                    while (gamepad1.right_trigger != 0) {
                        drive.setMotorPowers(putere_rotire, putere_rotire, -putere_rotire, -putere_rotire);
                    }
                    drive.update();
                    Pose2d poseEstimate = drive.getPoseEstimate();
                    telemetry.addData("x", poseEstimate.getX());
                    telemetry.addData("y", poseEstimate.getY());
                    telemetry.addData("heading", poseEstimate.getHeading());
                    telemetry.addData("ce_Se_happen", currentMode);


                    if (gamepad1.left_bumper) {
                        drive.Absoarbe(-putereAbsorbtie);

                    } else if (gamepad2.a) {
                        drive.Absoarbe(putereAbsorbtie);
                    } else {
                        drive.Absoarbe(0);
                    }
                    if (gamepad1.right_bumper) {
                        SampleMecanumDrive.Arunca(putereAruncare);
                    } else if (gamepad1.y) {
                        SampleMecanumDrive.Arunca(puterePowerShot);
                    } else {
                        SampleMecanumDrive.Arunca(0);
                    }

                    if (gamepad2.dpad_up) {
                        drive.RidicaWooble(-putereWooble);

                    } else if (gamepad2.dpad_down) {
                        drive.RidicaWooble(putereWooble);

                    } else {
                        drive.RidicaWooble(0);

                    }
                    //----------[tragaci]---------------------------------------------------------------------
                    if (gamepad2.dpad_right && buttonReleased5 == 1) {
                        buttonReleased5 = 0;
                        if (ghearaPusa == 0) {
                            ghearaPusa = 1;
                            drive.Gheara();
//                            drive.WoobleGheara(pozitie_gheara_closed);
                            telemetry.addLine("Gheara s-a strans");
                        } else {
                            ghearaPusa = 0;
                            drive.Gheara();
//                            drive.WoobleGheara(pozitie_gheara_open);
                            telemetry.addLine("Gheara s-a deschis");
                        }
                    }
                    if (!gamepad2.dpad_right) buttonReleased5 = 1;


                    //----------[tragaci]---------------------------------------------------------------------
                    if (gamepad2.right_bumper && buttonReleased == 1) {
//                        buttonReleased = 0;
//                        if (tragaciPus == 0) {
//                            tragaciPus = 1;
                        drive.TragaciPozitie(tragaciDeschis);
                        sleep(400);
                        drive.TragaciPozitie(tragaciInchis);
                        telemetry.addLine("Tragaci s-a strans");
//                        } else {
//                            tragaciPus = 0;
//
//                            telemetry.addLine("Tragaci s-a deschis");
//                        }
                    }
                    if (!gamepad2.right_bumper) buttonReleased = 1;
/*
                    //----------[organizator]---------------------------------------------------------------------
                    if (gamepad2.left_bumper && buttonReleased2 == 1) {
                        buttonReleased2 = 0;
                        if (organizatorPus == 0) {
                            organizatorPus = 1;
                            drive.OrganizatorPozitie(organizatorInchis);
                            telemetry.addLine("Organizator s-a strans");

                        } else {
                            organizatorPus = 0;
                            drive.OrganizatorPozitie(organizatorDeschis);
                            telemetry.addLine("Organizator s-a deschis");
                        }
                    }
                    if (!gamepad2.left_bumper) buttonReleased2 = 1;
*/
                    if (gamepad2.left_bumper && buttonReleased2 == 1) {
                        buttonReleased2 = 0;
                        if (organizatorPus == 0) {
                            organizatorPus = 1;
                            drive.InceputPozitie(pozitie_inceput_deschis);
                        } else {
                            organizatorPus = 0;
                            drive.InceputPozitie(pozitie_inceput_inchis);
                        }
                    }
                    if (!gamepad2.left_bumper) buttonReleased2 = 1;

                    //----------[fatete]---------------------------------------------------------------------
                    if (gamepad1.a && buttonReleased3 == 1) {
                        buttonReleased3 = 0;
                        if (fatetePuse == 0) {
                            fatetePuse = 1;
//                            drive.fatete(putere_fata_dr, putere_fata_st);
                            telemetry.addLine("Fatete s-au deschis");
                        } else {
                            fatetePuse = 0;
//                            drive.fatete(0, 0);
                            telemetry.addLine("Fatete s-au inchis");
                        }
                    }
                    if (!gamepad1.a) buttonReleased3 = 1;

                    if (gamepad2.b && epureFata == 1) {
                        epureFata = 0;
                        if (fatetePuse == 0) {
                            fatetePuse = 1;
//                            drive.fatete(-putere_fata_dr, -putere_fata_st);
                            telemetry.addLine("Fatete s-au deschis");
                        } else {
                            fatetePuse = 0;
//                            drive.fatete(0, 0);
                            telemetry.addLine("Fatete s-au inchis");
                        }
                    }
                    if (!gamepad2.b) epureFata = 1;


                    if (gamepad2.y) {
                        Trajectory traj1 = drive.trajectoryBuilder(poseEstimate)
                                .lineToLinearHeading(ARUNCARE_NORMALA)
                                .build();
                        drive.followTrajectory(traj1);
                        currentMode = Mode.AUTOMATIC_CONTROL;
                    }


                    break;
                case AUTOMATIC_CONTROL:
                    // daca apasam pe x din gmp2 se opreste autonomul
                    if (gamepad2.x) {
                        drive.cancelFollowing();
                        currentMode = Mode.DRIVER_CONTROL;
                    }

                    //ne intoarcem inapoi in driving mode dupa ce e gata autonom
                    if (!drive.isBusy()) {
                        currentMode = Mode.DRIVER_CONTROL;
                    }
                    break;

            }

        }
    }

    public void AsteaptaStart() // pentru eroarea cu Motorola
    {
        waitForStart();
//        while (!opModeIsActive() && !isStopRequested()) {
//            telemetry.update();
//        }
    }
}