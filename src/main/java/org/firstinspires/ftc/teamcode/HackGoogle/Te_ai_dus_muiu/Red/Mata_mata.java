package org.firstinspires.ftc.teamcode.HackGoogle.Te_ai_dus_muiu.Red;

import static org.firstinspires.ftc.teamcode.HackGoogle.NuUmbla.startposRosuDreapta;
import static org.firstinspires.ftc.teamcode.drive.ThoborVARS.aspect1;
import static org.firstinspires.ftc.teamcode.drive.ThoborVARS.aspect2;
import static org.firstinspires.ftc.teamcode.drive.ThoborVARS.debug;
import static org.firstinspires.ftc.teamcode.drive.ThoborVARS.magni;
import static org.firstinspires.ftc.teamcode.drive.ThoborVARS.zoom;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.R;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.ThoborVARS;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

import java.util.List;
import java.util.Random;

@Autonomous(name = "RosuDreaptaDreapta", group = "nationala")
public class Mata_mata extends LinearOpMode {
    private static final String KEY =
            "ARhzqPT/////AAABmcks6V9uRE/4vJE+8qBUvnsYPXUfYlpJ8y+pzVN/GpzCrJsVanetvGKZxaJMs+3LmpTosqzKWHhdAiOzqd3kFmr4WYOWRErWkQuuVRx5/merGbBTYOAKQ9rkri+O3XR/l3bWk3zVlXUH7wXisifJcM2xoXGON4lYuETqenXu4NFfqOXkDGWI1nBNMM1dFW6AhLEuGt0R1TP6ToWiA1rk6dBvg7W3jGDi7eGYdvQhuo5I+6/ffn/OAyWnt+5DiJFVK365Cubaa0IE5xO3J4SSNcVXaho39lO5o7EhtCmqO2icWi8bYv7o+DHXWPsKfPByyrKSjEaXpvBNQ6S7P5pw9p5I5t6XafbS2LxYE5AJ6zH6";
    private static final String TFOD_MODEL_ASSET = "UltimateGoal.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Quad";
    private static final String LABEL_SECOND_ELEMENT = "Single";
    public ElapsedTime timpD = new ElapsedTime();
    int path = -1;
    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        initCam();

        if (tfod != null) {
            tfod.activate();
            if (zoom == 1) {
//                tfod.setClippingMargins(left, top, right, bottom);
                tfod.setZoom(magni, aspect1 / aspect2);
            }
        }

        SampleMecanumDrive robot = new SampleMecanumDrive(hardwareMap);

        robot.setPoseEstimate(startposRosuDreapta);

        AsteaptaStart();

        Detectie();
        if (opModeIsActive()) {
            while (opModeIsActive()) {
                robot.Gheara();

                tfod.shutdown();

                if (debug != -1) {
                    path = debug;
                }

                if (path == 1) {
                    TrajectorySequence du_te_sa_scuipi_a = robot.trajectorySequenceBuilder(startposRosuDreapta)
                            .turn(Math.toRadians(-32))
                            .forward(40,
                                    SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                    SampleMecanumDrive.getAccelerationConstraint(40))
                            .build();

                    robot.followTrajectorySequence(du_te_sa_scuipi_a);

                    sleep(1000);

                    robot.RidicaWooble(1);
                    sleep(1000);
                    robot.RidicaWooble(0);

                    robot.turn(Math.toRadians(5));
                    sleep(1000);

                    robot.AruncaAutomat(3);

                    TrajectorySequence LaA = robot.trajectorySequenceBuilder(du_te_sa_scuipi_a.end())
                            .turn(Math.toRadians(-176.5))
                            .back(15,
                                    SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                    SampleMecanumDrive.getAccelerationConstraint(40))
                            .build();

                    robot.followTrajectorySequence(LaA);

                    robot.WoobleFinal();
                    TrajectorySequence La = robot.trajectorySequenceBuilder(LaA.end())
                            .strafeRight(15,
                                    SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                    SampleMecanumDrive.getAccelerationConstraint(40))
                            .back(10,
                                    SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                    SampleMecanumDrive.getAccelerationConstraint(40))

                            .build();

                    robot.followTrajectorySequence(La);

                }
                if (path == 2) {
                    TrajectorySequence du_te_sa_scuipi_b = robot.trajectorySequenceBuilder(startposRosuDreapta)
                            .turn(Math.toRadians(-33))
                            .forward(40,
                                    SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                    SampleMecanumDrive.getAccelerationConstraint(40))
                            .build();

                    robot.followTrajectorySequence(du_te_sa_scuipi_b);

                    sleep(1000);

                    robot.RidicaWooble(1);
                    sleep(1000);
                    robot.RidicaWooble(0);

                    robot.turn(Math.toRadians(5));
                    sleep(1000);

                    robot.AruncaAutomat(3);

                    TrajectorySequence LaB = robot.trajectorySequenceBuilder(du_te_sa_scuipi_b.end())
                            .turn(Math.toRadians(-90))
                            .strafeLeft(55,
                                    SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                    SampleMecanumDrive.getAccelerationConstraint(40))
                            .build();

                    robot.followTrajectorySequence(LaB);

                    robot.WoobleFinal();
                    TrajectorySequence Lab = robot.trajectorySequenceBuilder(LaB.end())
                            .strafeRight(20,
                                    SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                    SampleMecanumDrive.getAccelerationConstraint(40))
                            .build();

                    robot.followTrajectorySequence(Lab);

                }
                if (path == 3) {
                    telemetry.addLine("trb sa merg la c");
                    telemetry.update();
                    TrajectorySequence du_te_sa_scuipi_c = robot.trajectorySequenceBuilder(startposRosuDreapta)
                            .turn(Math.toRadians(-35))
                            .forward(40,
                                    SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                    SampleMecanumDrive.getAccelerationConstraint(40))
                            .build();

                    robot.followTrajectorySequence(du_te_sa_scuipi_c);

                    sleep(1000);

                    robot.RidicaWooble(1);
                    sleep(1000);
                    robot.RidicaWooble(0);

                    robot.turn(Math.toRadians(4.5));
                    sleep(1000);

                    robot.AruncaAutomat(3);

                    TrajectorySequence LaC = robot.trajectorySequenceBuilder(du_te_sa_scuipi_c.end())
                            .forward(40,
                                    SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                    SampleMecanumDrive.getAccelerationConstraint(40))
                            .turn(Math.toRadians(-180))
                            .strafeLeft(7)
                            .back(20,
                                    SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                    SampleMecanumDrive.getAccelerationConstraint(40))
                            .build();

                    robot.followTrajectorySequence(LaC);

                    robot.WoobleFinal();
                    TrajectorySequence Lac = robot.trajectorySequenceBuilder(LaC.end())
                            .forward(30)
                            .build();

                    robot.followTrajectorySequence(Lac);
                }

                stop();
            }
        }
    }


    private void initCam() {
        msStuckDetectStop = 2500;

        //init vuforia
        VuforiaLocalizer.Parameters vuforiaParams = new VuforiaLocalizer.Parameters(R.id.cameraMonitorViewId);
        vuforiaParams.vuforiaLicenseKey = ThoborVARS.Vuforia_Key;
        vuforiaParams.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");
        vuforiaParams.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;

        VuforiaLocalizer vuforia = ClassFactory.getInstance().createVuforia(vuforiaParams);

        //pentru a vedea pe dash camera
        FtcDashboard.getInstance().startCameraStream(vuforia, 0);

        //init tenserflow
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.8f;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);


    }

    private void AsteaptaStart() {
        while (!opModeIsActive() && !isStopRequested()) {
            telemetry.update();
        }
    }

    private void Detectie() {
        timpD.reset();
        if (tfod != null) {
            List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
            if (updatedRecognitions != null) {
                telemetry.addData("# Object Detected", updatedRecognitions.size());
                if (updatedRecognitions.size() == 0) {
                    telemetry.addData("Target Zone", "A");
                    path = 1;
                } else {
                    for (Recognition recognition : updatedRecognitions) {
                        if (recognition.getLabel().equals("Quad")) {
                            telemetry.addData("Target Zone", "C");
                            path = 3;
                        } else if (recognition.getLabel().equals("Single")) {
                            telemetry.addData("Target Zone", "B");
                            path = 2;
                        } else {
                            telemetry.addData("Target Zone", "UNKNOWN");
                            Random rand = new Random();
                            path = rand.nextInt(3) + 1;
                        }
                    }
                }
                telemetry.update();
            }
        }
    }
}
