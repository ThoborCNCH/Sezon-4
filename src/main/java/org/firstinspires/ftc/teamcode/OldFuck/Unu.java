// package org.firstinspires.ftc.teamcode.Matei.PID;
// //////
// //////import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
// //////import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
// //////import com.qualcomm.robotcore.hardware.DcMotor;
// //////import com.qualcomm.robotcore.hardware.DcMotorSimple;
// //////import com.qualcomm.robotcore.hardware.PIDCoefficients;
// //////import com.qualcomm.robotcore.hardware.PIDFCoefficients;
// //////import com.qualcomm.robotcore.util.ElapsedTime;
// //////
// //////@Autonomous(name = "PID", group = "a")
// //////public class unu extends LinearOpMode {
// //////
// //////    DcMotor l, r;
// //////
// //////    double integral = 0;
// //////
// //////    PIDCoefficients coeffs = new PIDCoefficients(0, 0, 0);
// //////
// ////////    PIDFCoefficients controller = new PIDFCoefficients(coeffs);
// //////
// ////////    controller.setTargetPosition(setpoint);
// //////
// ////////    double correction = controller.update(measuredPosition);
// //////
// //////
// //////    PIDFController controller = new PIDFController(coeffs, 0, 0, 0, new Function2<Double, Double, Double>() {
// //////        @Override
// //////        public Double invoke(Double position, Double velocity) {
// //////            return kG;
// //////        }
// //////    });
// //////
// //////    // or more concisely with lambdas
// //////    PIDFController controller = new PIDFController(coeffs, 0, 0, 0, (x, v) -> kG);
// //////    ElapsedTime time = new ElapsedTime();
// //////
// //////    @Override
// //////    public void runOpMode() throws InterruptedException {
// //////        l = hardwareMap.dcMotor.get("left_motor");
// //////        r = hardwareMap.dcMotor.get("right_motor");
// //////
// //////        l.setDirection(DcMotorSimple.Direction.REVERSE);
// //////
// //////
// //////    }
// //////}
// ////
// ////
// ////package org.firstinspires.ftc.teamcode.Matei.PID;
// ////
// ////import com.qualcomm.hardware.bosch.BNO055IMU;
// ////import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
// ////import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
// ////import com.qualcomm.robotcore.hardware.DcMotor;
// ////
// ////import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
// ////import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
// ////import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
// ////import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
// ////
// ////@Autonomous(name = "Drive Avoid PID", group = "Exercises")
// //////@Disabled
// ////public class unu extends LinearOpMode {
// ////    DcMotor leftMotor, rightMotor;
// ////    BNO055IMU imu;
// ////    Orientation lastAngles = new Orientation();
// ////    double globalAngle, power = .30, correction, rotation;
// ////    boolean aButton, bButton, touched;
// ////    PIDController pidRotate, pidDrive;
// ////
// ////    // called when init button is  pressed.
// ////    @Override
// ////    public void runOpMode() throws InterruptedException {
// ////        leftMotor = hardwareMap.dcMotor.get("left_motor");
// ////        rightMotor = hardwareMap.dcMotor.get("right_motor");
// ////
// ////        leftMotor.setDirection(DcMotor.Direction.REVERSE);
// ////
// ////        leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
// ////        rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
// ////
// ////        // get a reference to REV Touch sensor.
// ////
// ////        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
// ////
// ////        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
// ////        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
// ////        parameters.loggingEnabled = false;
// ////
// ////        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
// ////        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
// ////        // and named "imu".
// ////        imu = hardwareMap.get(BNO055IMU.class, "imu");
// ////
// ////        imu.initialize(parameters);
// ////
// ////        // Set PID proportional value to start reducing power at about 50 degrees of rotation.
// ////        // P by itself may stall before turn completed so we add a bit of I (integral) which
// ////        // causes the PID controller to gently increase power if the turn is not completed.
// ////        pidRotate = new PIDController(.003, .00003, 0);
// ////
// ////        // Set PID proportional value to produce non-zero correction value when robot veers off
// ////        // straight line. P value controls how sensitive the correction is.
// ////        pidDrive = new PIDController(.05, 0, 0);
// ////
// ////        telemetry.addData("Mode", "calibrating...");
// ////        telemetry.update();
// ////
// ////        // make sure the imu gyro is calibrated before continuing.
// ////        waitForStart();
// ////
// ////        telemetry.addData("Mode", "running");
// ////        telemetry.update();
// ////
// ////        sleep(1000);
// ////
// ////        // Set up parameters for driving in a straight line.
// ////        pidDrive.setSetpoint(0);
// ////        pidDrive.setOutputRange(0, power);
// ////        pidDrive.setInputRange(-90, 90);
// ////        pidDrive.enable();
// ////
// ////        // drive until end of period.
// ////
// ////        while (opModeIsActive()) {
// ////            // Use PID with imu input to drive in a straight line.
// ////            correction = pidDrive.performPID(getAngle());
// ////
// ////            telemetry.addData("1 imu heading", lastAngles.firstAngle);
// ////            telemetry.addData("2 global heading", globalAngle);
// ////            telemetry.addData("3 correction", correction);
// ////            telemetry.addData("4 turn rotation", rotation);
// ////            telemetry.update();
// ////
// ////            // set power levels.
// ////            leftMotor.setPower(power - correction);
// ////            rightMotor.setPower(power + correction);
// ////
// ////            // We record the sensor values because we will test them in more than
// ////            // one place with time passing between those places. See the lesson on
// ////            // Timing Considerations to know why.
// ////
// ////            aButton = gamepad1.a;
// ////            bButton = gamepad1.b;
// ////
// ////            if (touched || aButton || bButton) {
// ////                // backup.
// ////                leftMotor.setPower(-power);
// ////                rightMotor.setPower(-power);
// ////
// ////                sleep(500);
// ////
// ////                // stop.
// ////                leftMotor.setPower(0);
// ////                rightMotor.setPower(0);
// ////
// ////                // turn 90 degrees right.
// ////                if (touched || aButton) rotate(-90, power);
// ////
// ////                // turn 90 degrees left.
// ////                if (bButton) rotate(90, power);
// ////            }
// ////        }
// ////
// ////        // turn the motors off.
// ////        rightMotor.setPower(0);
// ////        leftMotor.setPower(0);
// ////    }
// ////
// ////    /**
// ////     * Resets the cumulative angle tracking to zero.
// ////     */
// ////    private void resetAngle() {
// ////        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
// ////
// ////        globalAngle = 0;
// ////    }
// ////
// ////    /**
// ////     * Get current cumulative angle rotation from last reset.
// ////     *
// ////     * @return Angle in degrees. + = left, - = right from zero point.
// ////     */
// ////    private double getAngle() {
// ////        // We experimentally determined the Z axis is the axis we want to use for heading angle.
// ////        // We have to process the angle because the imu works in euler angles so the Z axis is
// ////        // returned as 0 to +180 or 0 to -180 rolling back to -179 or +179 when rotation passes
// ////        // 180 degrees. We detect this transition and track the total cumulative angle of rotation.
// ////
// ////        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
// ////
// ////        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;
// ////
// ////        if (deltaAngle < -180)
// ////            deltaAngle += 360;
// ////        else if (deltaAngle > 180)
// ////            deltaAngle -= 360;
// ////
// ////        globalAngle += deltaAngle;
// ////
// ////        lastAngles = angles;
// ////
// ////        return globalAngle;
// ////    }
// ////
// ////    /**
// ////     * Rotate left or right the number of degrees. Does not support turning more than 359 degrees.
// ////     *
// ////     * @param degrees Degrees to turn, + is left - is right
// ////     */
// ////    private void rotate(int degrees, double power) {
// ////        // restart imu angle tracking.
// ////        resetAngle();
// ////
// ////        // if degrees > 359 we cap at 359 with same sign as original degrees.
// ////        if (Math.abs(degrees) > 359) degrees = (int) Math.copySign(359, degrees);
// ////
// ////        // start pid controller. PID controller will monitor the turn angle with respect to the
// ////        // target angle and reduce power as we approach the target angle. This is to prevent the
// ////        // robots momentum from overshooting the turn after we turn off the power. The PID controller
// ////        // reports onTarget() = true when the difference between turn angle and target angle is within
// ////        // 1% of target (tolerance) which is about 1 degree. This helps prevent overshoot. Overshoot is
// ////        // dependant on the motor and gearing configuration, starting power, weight of the robot and the
// ////        // on target tolerance. If the controller overshoots, it will reverse the sign of the output
// ////        // turning the robot back toward the setpoint value.
// ////
// ////        pidRotate.reset();
// ////        pidRotate.setSetpoint(degrees);
// ////        pidRotate.setInputRange(0, degrees);
// ////        pidRotate.setOutputRange(0, power);
// ////        pidRotate.setTolerance(1);
// ////        pidRotate.enable();
// ////
// ////        // getAngle() returns + when rotating counter clockwise (left) and - when rotating
// ////        // clockwise (right).
// ////
// ////        // rotate until turn is completed.
// ////
// ////        if (degrees < 0) {
// ////            // On right turn we have to get off zero first.
// ////            while (opModeIsActive() && getAngle() == 0) {
// ////                leftMotor.setPower(power);
// ////                rightMotor.setPower(-power);
// ////                sleep(100);
// ////            }
// ////
// ////            do {
// ////                power = pidRotate.performPID(getAngle()); // power will be - on right turn.
// ////                leftMotor.setPower(-power);
// ////                rightMotor.setPower(power);
// ////            } while (opModeIsActive() && !pidRotate.onTarget());
// ////        } else    // left turn.
// ////            do {
// ////                power = pidRotate.performPID(getAngle()); // power will be + on left turn.
// ////                leftMotor.setPower(-power);
// ////                rightMotor.setPower(power);
// ////            } while (opModeIsActive() && !pidRotate.onTarget());
// ////
// ////        // turn the motors off.
// ////        rightMotor.setPower(0);
// ////        leftMotor.setPower(0);
// ////
// ////        rotation = getAngle();
// ////
// ////        // wait for rotation to stop.
// ////        sleep(500);
// ////
// ////        // reset angle tracking on new heading.
// ////        resetAngle();
// ////    }
// ////}
// //
// //


// import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
// import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
// import com.qualcomm.robotcore.hardware.DcMotor;
// import com.qualcomm.robotcore.hardware.DcMotorEx;
// import com.qualcomm.robotcore.hardware.PIDCoefficients;
// import com.qualcomm.robotcore.hardware.PIDFCoefficients;
// import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

// /**
//  * Created by tom on 9/26/17.
//  * This assumes that you are using a REV Robotics Expansion Hub
//  * as your DC motor controller.  This op mode uses the extended/enhanced
//  * PID-related functions of the DcMotorEx class.  The REV Robotics Expansion Hub
//  * supports the extended motor functions, but other controllers (such as the
//  * Modern Robotics and Hitechnic DC Motor Controllers) do not.
//  */

// @Autonomous(name = "Concept: Change PID", group = "Concept")
// public class Unu extends LinearOpMode {

//     // our DC motor.
//     DcMotorEx lf;
//     DcMotorEx rf;
//     DcMotorEx lr;
//     DcMotorEx rr;

//     public static final double NEW_P = 2.5;
//     public static final double NEW_I = 0.1;
//     public static final double NEW_D = 0.2;

//     @Override
//     public void runOpMode() throws InterruptedException {
//         // get reference to DC motor.
//         // since we are using the Expansion Hub,
//         // cast this motor to a DcMotorEx object.
//         lf = hardwareMap.get(DcMotorEx.class, "front_left_motor");
//         rf = hardwareMap.get(DcMotorEx.class, "front_right_motor");
//         lr = hardwareMap.get(DcMotorEx.class, "back_left_motor");
//         rr = hardwareMap.get(DcMotorEx.class, "back_right_motor");

//         // wait for start command.
//         waitForStart();
//         PIDFCoefficients pidOriglf = lf.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
//         PIDFCoefficients pidOrigrf = rf.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
//         PIDFCoefficients pidOriglr = lr.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
//         PIDFCoefficients pidOrigrr = rr.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);


//         // change coefficients using methods included with DcMotorEx class.
//         PIDFCoefficients pidNew = new PIDFCoefficients(NEW_P, NEW_I, NEW_D);
//         lf.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidNew);
//         rf.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidNew);
//         lr.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidNew);
//         rr.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidNew);

//         // re-read coefficients and verify change.
//         PIDCoefficients pidModifiedlf = lf.getPIDCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
//         PIDCoefficients pidModifiedrf = rf.getPIDCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
//         PIDCoefficients pidModifiedlr = lr.getPIDCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
//         PIDCoefficients pidModifiedrr = rr.getPIDCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);

//         // display info to user.
//         while (opModeIsActive()) {
//             telemetry.addData("Runtime", "%.03f", getRuntime());
//             telemetry.addData("P,I,D (orig) for lf:", "%.04f, %.04f, %.0f",
//                     pidOriglf.p, pidOriglf.i, pidOriglf.d);
//             telemetry.addData("P,I,D (orig) for rf:", "%.04f, %.04f, %.0f",
//                     pidOrigrf.p, pidOrigrf.i, pidOrigrf.d);
//             telemetry.addData("P,I,D (orig) for lr:", "%.04f, %.04f, %.0f",
//                     pidOriglr.p, pidOriglr.i, pidOriglr.d);
//             telemetry.addData("P,I,D (orig) for rr:", "%.04f, %.04f, %.0f",
//                     pidOrigrr.p, pidOrigrr.i, pidOrigrr.d);

//             telemetry.addData("P,I,D (modified) for lf:", "%.04f, %.04f, %.04f",
//                     pidModifiedlf.p, pidModifiedlf.i, pidModifiedlf.d);
//             telemetry.addData("P,I,D (modified) for rf:", "%.04f, %.04f, %.04f",
//                     pidModifiedrf.p, pidModifiedrf.i, pidModifiedrf.d);
//             telemetry.addData("P,I,D (modified) for lr:", "%.04f, %.04f, %.04f",
//                     pidModifiedlr.p, pidModifiedlr.i, pidModifiedlr.d);
//             telemetry.addData("P,I,D (modified) for rr:", "%.04f, %.04f, %.04f",
//                     pidModifiedrr.p, pidModifiedrr.i, pidModifiedrr.d);
//             telemetry.update();
//         }
//     }
// }



