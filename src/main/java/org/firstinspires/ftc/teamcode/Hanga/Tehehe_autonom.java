package org.firstinspires.ftc.teamcode.Hanga;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.util.NanoClock;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.rev.RevTouchSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.R;

import java.util.List;
import java.util.Random;

import static org.firstinspires.ftc.teamcode.drive.ThoborVARS.DirWooble;
import static org.firstinspires.ftc.teamcode.drive.ThoborVARS.GradeInceput;
import static org.firstinspires.ftc.teamcode.drive.ThoborVARS.Jumatate_Patrat;
import static org.firstinspires.ftc.teamcode.drive.ThoborVARS.Vuforia_Key;
import static org.firstinspires.ftc.teamcode.drive.ThoborVARS.organizatorInchis;
import static org.firstinspires.ftc.teamcode.drive.ThoborVARS.pozitie_gheara_closed;
import static org.firstinspires.ftc.teamcode.drive.ThoborVARS.pozitie_gheara_open;
import static org.firstinspires.ftc.teamcode.drive.ThoborVARS.putereWooble;
import static org.firstinspires.ftc.teamcode.drive.ThoborVARS.tragaciDeschis;
import static org.firstinspires.ftc.teamcode.drive.ThoborVARS.tragaciInchis;

@Config
@Disabled
@Autonomous(name = "autonom tehehe", group = "Hanga")
public class Tehehe_autonom extends LinearOpMode {
    private static final String TFOD_MODEL_ASSET = "UltimateGoal.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Quad";
    private static final String LABEL_SECOND_ELEMENT = "Single";
    public static Double bias = 0.315;//default 0.8
    public static Double meccyBias = 0.38;//change to adjust only strafing movement
    public static double vreau_sa_mor = 1; //DA IN PULA MEA
    public static ElapsedTime timpAr = new ElapsedTime();
    public RevTouchSensor buton_servo;
    public RevTouchSensor buton_capat;
    public DcMotorEx aruncare;
    public DcMotorEx absorbtie;
    //======[problem la chisior]
    public Servo tragaci;
    public Servo wooble_gheara;
    public Servo organizator;
    public CRServo fata_dr;
    public CRServo fata_st;
    public CRServo wooblemoto;/*
    public ElapsedTime mRunTime = new ElapsedTime();

    private ElapsedTime runtime = new ElapsedTime();*/
    public ElapsedTime timpD = new ElapsedTime();
    //
    DcMotorEx frontleft;
    DcMotorEx frontright;
    DcMotorEx backleft;
    DcMotorEx backright;
    //28 * 20 / (2ppi * 4.125)
    Double width = 16.92913; //inches
    Integer cpr = 146; //counts per rotation
    Integer gearratio = 5;
    Double diameter = 4.0;
    Double cpi = (cpr * gearratio) / (Math.PI * diameter); //counts per inch, 28cpr * gear ratio / (2 * pi * diameter (in inches, in the center))
    //
    Double conversion = cpi * bias;
    Boolean exit = false;
    //
    BNO055IMU imu;
    Orientation angles;
    Acceleration gravity;
    int path = -1;
    private TFObjectDetector tfod;
    private int primadata = 0;
    private FtcDashboard dashboard;

    //
    private NanoClock clock;

    public void runOpMode() {
        TotInceputul();
        //
        if (opModeIsActive()) {

            while (opModeIsActive() && !isStopRequested()) {
                if (primadata == 0) // that was she said // asta te duce la inele
                {
                    frontleft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    frontright.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    backleft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    backright.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


                    moveToPosition(Jumatate_Patrat-1, 0.5);
//                    moveToPosition(Jumatate_Patrat-1, 0.5);
                    //
                    turnWithGyro(GradeInceput, -0.2);
                    primadata = 1;
                }
                Detectie();
                oprire_vuforia();
                turnWithGyro(GradeInceput, 0.2); //revenire
                telemetry.addData("poz: ", path);
                telemetry.update();
                if (vreau_sa_mor == 1) {
                    switch (path) {
                        case 1: {


                            //
                            moveToPosition(70, 0.4);
                            //

                            strafeToPosition(15, 0.4); // mergem in stanga sa ne cacam in wooble

                            DuWooble(1);

                            strafeToPosition(-19, 0.3); // mergem in stanga sa aruncam

                            moveToPosition(-21, 0.3); // merg inapoi la linie

                            turnWithGyro(7, 0.2); //revenire

                            AruncaAutomat(3,1);
                            //
                            moveToPosition(15, 0.2); // ne parcam
                            sleep(250);
                            stop();
                        }
                        case 2: {

                            //
                            moveToPosition(76, 0.4);
                            turnWithGyro(90, 0.2); //revenire
                            DuWooble(1);
                            strafeToPosition(25, 0.4);
                            moveToPosition(-10, 0.4);
                            turnWithGyro(95, -0.2); //revenire
                            AruncaAutomat(3, 2);
                            moveToPosition(12, 0.2);

                            stop();

                        }
                        case 3: {
                            //
                            moveToPosition(84, 0.4);
                            turnWithGyro(180, -0.2); //revenire
                            sleep(100);
                            DuWooble(1);
                            sleep(100);
                            moveToPosition(32.8, 0.4);
                            turnWithGyro(170, 0.2); //revenire
                            strafeToPosition(-12, 0.4);

                            AruncaAutomat(3,3);
                            moveToPosition(10, 0.2);
                            stop();
                        }

                    }

                }
                //
            }
        }

    }

    //
    /*
    This function's purpose is simply to drive forward or backward.
    To drive backward, simply make the inches input negative.
     */
    public void moveToPosition(double inches, double speed) {
        pornesteMotoare();
        //
        int move = (int) (Math.round(inches * conversion));
        //
        backleft.setTargetPosition(backleft.getCurrentPosition() + move);
        frontleft.setTargetPosition(frontleft.getCurrentPosition() + move);
        backright.setTargetPosition(backright.getCurrentPosition() + move);
        frontright.setTargetPosition(frontright.getCurrentPosition() + move);

        //
        frontleft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontright.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backleft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backright.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //
        frontleft.setPower(speed);
        backleft.setPower(speed);
        frontright.setPower(speed);
        backright.setPower(speed);
        //
        while (frontleft.isBusy() && frontright.isBusy() && backleft.isBusy() && backright.isBusy()) {
            if (exit) {
                frontright.setPower(0);
                frontleft.setPower(0);
                backright.setPower(0);
                backleft.setPower(0);

                opresteMotoare();
                return;
            }
        }
        pornesteMotoare();
        frontright.setPower(0);
        frontleft.setPower(0);
        backright.setPower(0);
        backleft.setPower(0);

        opresteMotoare();
        sleep(250);
        //return;
    }

    //
    /*
    This function uses the Expansion Hub IMU Integrated Gyro to turn a precise number of degrees (+/- 5).
    Degrees should always be positive, make speedDirection negative to turn left.
     */
    public void turnWithGyro(double degrees, double speedDirection) {
        pornesteMotoare();
        //<editor-fold desc="Initialize">
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double yaw = -angles.firstAngle;//make this negative
        telemetry.addData("Speed Direction", speedDirection);
        telemetry.addData("Yaw", yaw);
        telemetry.update();
        //
        telemetry.addData("stuff", speedDirection);
        telemetry.update();
        //
        double first;
        double second;
        //</editor-fold>
        //
        if (speedDirection > 0) {//set target positions
            //<editor-fold desc="turn right">
            if (degrees > 10) {
                first = (degrees - 10) + devertify(yaw);
            } else {
                first = devertify(yaw);
            }
            second = degrees + devertify(yaw);
            //</editor-fold>
        } else {
            //<editor-fold desc="turn left">
            if (degrees > 10) {
                first = devertify(-(degrees - 10) + devertify(yaw));
            } else {
                first = devertify(yaw);
            }
            second = devertify(-degrees + devertify(yaw));
            //
            //</editor-fold>
        }
        //
        //<editor-fold desc="Go to position">
        double firsta = convertify(first - 5);//175
        double firstb = convertify(first + 5);//-175
        //
        turnWithEncoder(speedDirection);
        //
        if (Math.abs(firsta - firstb) < 11) {
            while (!(firsta < yaw && yaw < firstb) && opModeIsActive()) {//within range?
                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                gravity = imu.getGravity();
                yaw = -angles.firstAngle;
                telemetry.addData("Position", yaw);
                telemetry.addData("first before", first);
                telemetry.addData("first after", convertify(first));
                telemetry.update();
            }
        } else {
            //
            while (!((firsta < yaw && yaw < 180) || (-180 < yaw && yaw < firstb)) && opModeIsActive()) {//within range?
                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                gravity = imu.getGravity();
                yaw = -angles.firstAngle;
                telemetry.addData("Position", yaw);
                telemetry.addData("first before", first);
                telemetry.addData("first after", convertify(first));
                telemetry.update();
            }
        }
        //
        double seconda = convertify(second - 5);//175
        double secondb = convertify(second + 5);//-175
        //
        turnWithEncoder(speedDirection / 3);
        //
        if (Math.abs(seconda - secondb) < 11) {
            while (!(seconda < yaw && yaw < secondb) && opModeIsActive()) {//within range?
                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                gravity = imu.getGravity();
                yaw = -angles.firstAngle;
                telemetry.addData("Position", yaw);
                telemetry.addData("second before", second);
                telemetry.addData("second after", convertify(second));
                telemetry.update();
            }
            while (!((seconda < yaw && yaw < 180) || (-180 < yaw && yaw < secondb)) && opModeIsActive()) {//within range?
                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                gravity = imu.getGravity();
                yaw = -angles.firstAngle;
                telemetry.addData("Position", yaw);
                telemetry.addData("second before", second);
                telemetry.addData("second after", convertify(second));
                telemetry.update();
            }
            frontleft.setPower(0);
            frontright.setPower(0);
            backleft.setPower(0);
            backright.setPower(0);
            opresteMotoare();
        }
        //</editor-fold>
        //
        frontleft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontright.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backleft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backright.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontleft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontright.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backleft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backright.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        sleep(250);
    }

    //
    /*
    This function uses the encoders to strafe left or right.
    Negative input for inches results in left strafing.
     */
    public void strafeToPosition(double inches, double speed) {
        pornesteMotoare();
        //
        int move = (int) (Math.round(inches * cpi * meccyBias));
        //
        backleft.setTargetPosition(backleft.getCurrentPosition() - move);
        frontleft.setTargetPosition(frontleft.getCurrentPosition() + move);
        backright.setTargetPosition(backright.getCurrentPosition() + move);
        frontright.setTargetPosition(frontright.getCurrentPosition() - move);
        //
        frontleft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontright.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backleft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backright.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //
        frontleft.setPower(speed);
        backleft.setPower(speed);
        frontright.setPower(speed);
        backright.setPower(speed);
        //
        //noinspection StatementWithEmptyBody
        while (frontleft.isBusy() && frontright.isBusy() && backleft.isBusy() && backright.isBusy()) {
        }
        frontright.setPower(0);
        frontleft.setPower(0);
        backright.setPower(0);
        backleft.setPower(0);
        opresteMotoare();
        sleep(250);
        //return;
    }

    //
    /*
    A tradition within the Thunder Pengwins code, we always start programs with waitForStartify,
    our way of adding personality to our programs.
     */
    public void waitForStartify() {
        waitForStart();
    }

    //
    /*
    These functions are used in the turnWithGyro function to ensure inputs
    are interpreted properly.
     */
    public double devertify(double degrees) {
        if (degrees < 0) {
            degrees = degrees + 360;
        }
        return degrees;
    }

    public double convertify(double degrees) {
        if (degrees > 179) {
            degrees = -(360 - degrees);
        } else if (degrees < -180) {
            degrees = 360 + degrees;
        } else if (degrees > 360) {
            degrees = degrees - 360;
        }
        return degrees;
    }

    //
    /*
    This function is called at the beginning of the program to activate
    the IMU Integrated Gyro.
     */
    public void initGyro() {
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        //parameters.calibrationDataFile = "GyroCal.json"; // see the calibration sample opmode
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        //
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
    }

    //
    /*
    This function is used in the turnWithGyro function to set the
    encoder mode and turn.
     */
    public void turnWithEncoder(double input) {
        frontleft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backleft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontright.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backright.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //
        frontleft.setPower(input);
        backleft.setPower(input);
        frontright.setPower(-input);
        backright.setPower(-input);
    }

    //
    public void MereCamerele() {

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        msStuckDetectStop = 2500;
        VuforiaLocalizer.Parameters vuforiaParams = new VuforiaLocalizer.Parameters(R.id.cameraMonitorViewId);
        vuforiaParams.vuforiaLicenseKey = Vuforia_Key;
        vuforiaParams.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");
        vuforiaParams.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        VuforiaLocalizer vuforia = ClassFactory.getInstance().createVuforia(vuforiaParams);
        FtcDashboard.getInstance().startCameraStream(vuforia, 0);
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.8f;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
    }

    public void Detectie() {
        timpD.reset();
        while (timpD.time() < 2 && !isStopRequested()) {
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

    public void oprire_vuforia() {
        if (tfod != null) {
            tfod.shutdown();
        }
    }

    public void PozitiiDeBaza() {
        TragaciPozitie(tragaciInchis);
        OrganizatorPozitie(organizatorInchis);
        WoobleGheara(pozitie_gheara_closed);

    }

    public void TragaciPozitie(double poztragaci) {
        tragaci.setPosition(poztragaci);
    }

    public void OrganizatorPozitie(double pozorganizator) {
        organizator.setPosition(pozorganizator);
    }

    public void WoobleGheara(double pozgheara) {
        wooble_gheara.setPosition(pozgheara);
    }

    public void Arunca(double puterearuncare) {
        aruncare.setPower(puterearuncare);
    }

    public void opresteMotoare() {
        frontright.setMotorDisable();
        frontleft.setMotorDisable();
        backright.setMotorDisable();
        backleft.setMotorDisable();
    }

    public void pornesteMotoare() {
        frontright.setMotorEnable();
        frontleft.setMotorEnable();
        backright.setMotorEnable();
        backleft.setMotorEnable();
    }


    public void AruncaAutomat(int nr_inele, int caz) {
        opresteMotoare();
        for (int i = 1; i <= nr_inele; i++) {
            timpAr.reset();
            while (timpAr.time() < 1.2) {
                if(caz==1)Arunca(0.79);
                if(caz==2)Arunca(0.72);
                if(caz==3)Arunca(0.75);
            }
            TragaciPozitie(tragaciDeschis);
            sleep(550);
            TragaciPozitie(tragaciInchis);
            Arunca(0);
        }
        pornesteMotoare();
    }

    public void RidicaWooble(double putereridicare) {
        wooblemoto.setPower(putereridicare);
    }

    public void DuWooble(int pas) {
        opresteMotoare();
        if (pas == 1) {
            do {
                RidicaWooble(putereWooble);
            } while (!buton_servo.isPressed());
            sleep(200);
            WoobleGheara(pozitie_gheara_open);
            sleep(100);

            RidicaWooble(-1);
            do {
                RidicaWooble(-putereWooble);
            } while (!buton_capat.isPressed());
            RidicaWooble(-0.1);
            sleep(200);
            RidicaWooble(0);
        } else if (pas == 2) {
            WoobleGheara(pozitie_gheara_open);
            do {
                RidicaWooble(putereWooble);
            } while (!buton_servo.isPressed());
            RidicaWooble(0.3);
            sleep(600);
            WoobleGheara(pozitie_gheara_closed);
            sleep(500);

            RidicaWooble(-1);

            do {
                RidicaWooble(-putereWooble);
            } while (!buton_capat.isPressed());
            RidicaWooble(-0.3);
            sleep(100);
            RidicaWooble(-0.1);
            sleep(100);
            RidicaWooble(0);

        } else if (pas == 3) {
            while (true) {
                RidicaWooble(putereWooble);
                if (buton_servo.isPressed()) {
                    WoobleGheara(pozitie_gheara_open);
                    sleep(33);
                    break;
                }
            }
            RidicaWooble(0.2);
            sleep(250);

            RidicaWooble(0);

        } else if (pas == 4) {
            WoobleGheara(pozitie_gheara_closed);
            sleep(500);
            RidicaWooble(-putereWooble);
            do {
                RidicaWooble(-putereWooble);
            } while (!buton_capat.isPressed());
            RidicaWooble(-0.1);
            sleep(150);
            RidicaWooble(0);

        } else if (pas == 5) {
            do {
                RidicaWooble(putereWooble);
            } while (!buton_servo.isPressed());
            RidicaWooble(0.3);
            sleep(150);
            RidicaWooble(0.1);
            sleep(250);
            WoobleGheara(pozitie_gheara_open);
            sleep(250);

            RidicaWooble(-1);

            do {
                RidicaWooble(-putereWooble);
            } while (!buton_capat.isPressed());

            RidicaWooble(-0.3);
            sleep(150);
            RidicaWooble(-0.1);
            sleep(200);
            RidicaWooble(0);

        } else if (pas == 6) {
            do {
                RidicaWooble(putereWooble);
            } while (!buton_servo.isPressed());
            RidicaWooble(0.2);
            sleep(250);
            WoobleGheara(pozitie_gheara_open);
            sleep(250);

            RidicaWooble(-1);

            do {
                RidicaWooble(-putereWooble);
            } while (!buton_capat.isPressed());

            RidicaWooble(-0.3);
            sleep(150);
            RidicaWooble(-0.1);
            sleep(200);
            RidicaWooble(0);
        } else if (pas == 7) {
            WoobleGheara(pozitie_gheara_open);
            sleep(500);
            RidicaWooble(-1);

            do {
                RidicaWooble(-putereWooble);
            } while (!buton_capat.isPressed());

            RidicaWooble(-0.3);
            sleep(150);
            RidicaWooble(-0.1);
            sleep(200);
            RidicaWooble(0);
        }
        pornesteMotoare();
    }

    public void TotInceputul() {
        MereCamerele();

        if (tfod != null) {
            tfod.activate();
            tfod.setZoom(2.5, 16.0 / 9.0);
        }

        //
        initGyro();
        //

        initHard();
        PozitiiDeBaza();

        //
        waitForStartify();
    }

    public void initHard() {
        buton_servo = hardwareMap.get(RevTouchSensor.class, "servo");
        buton_capat = hardwareMap.get(RevTouchSensor.class, "capat");


        absorbtie = hardwareMap.get(DcMotorEx.class, "absorbtie");
        aruncare = hardwareMap.get(DcMotorEx.class, "aruncare");
        /*
        buton_capat.setMode(DigitalChannel.Mode.INPUT);
        buton_servo.setMode(DigitalChannel.Mode.INPUT);*/

        dashboard = FtcDashboard.getInstance();
        dashboard.setTelemetryTransmissionInterval(25);

        clock = NanoClock.system();

        frontleft = hardwareMap.get(DcMotorEx.class, "front_left_motor");
        backleft = hardwareMap.get(DcMotorEx.class, "back_left_motor");
        backright = hardwareMap.get(DcMotorEx.class, "back_right_motor");
        frontright = hardwareMap.get(DcMotorEx.class, "front_right_motor");

        frontleft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        backleft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        backright.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        frontright.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);


        wooblemoto = hardwareMap.get(CRServo.class, "wooblemoto");



        aruncare.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        aruncare.setDirection(DcMotorEx.Direction.REVERSE);
        aruncare.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);


        if (DirWooble == 1) wooblemoto.setDirection(DcMotorEx.Direction.REVERSE);
        else wooblemoto.setDirection(DcMotorEx.Direction.FORWARD);

        wooble_gheara = hardwareMap.get(Servo.class, "wooble_gheara");
        fata_dr = hardwareMap.get(CRServo.class, "fata_dr");
        fata_st = hardwareMap.get(CRServo.class, "fata_st");

        tragaci = hardwareMap.get(Servo.class, "tragaci");

        organizator = hardwareMap.get(Servo.class, "organizator");

        frontleft.setDirection(DcMotorSimple.Direction.REVERSE);
        backleft.setDirection(DcMotorSimple.Direction.REVERSE);
    }

}
