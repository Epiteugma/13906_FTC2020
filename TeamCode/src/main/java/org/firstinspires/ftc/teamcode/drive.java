package org.firstinspires.ftc.teamcode;



import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import java.util.*;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import java.lang.Runnable;
import java.util.Locale;


@TeleOp(name="Basic: Iterative OpMode", group="FTC_Cyprus_2020-21")



public class drive extends LinearOpMode {


    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor fl = null;
    private DcMotor bl = null;
    private DcMotor br = null;
    private DcMotor fr = null;
    private Servo shooterLoader = null;
    private double globalpowerfactor = 0.8;
    private DcMotor collector = null;
    private DcMotor shooter = null;
    private final static double SLHOME=0.0; //starting pos
    private final static double SLMINR=0.0; //smallest pos
    private final static double SLMAXR=1.0; //bigger pos
    BNO055IMU imu;
    Orientation angles;
    Acceleration gravity;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void runOpMode() {
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        collector = hardwareMap.get(DcMotor.class, "collector");
        shooterLoader = hardwareMap.get(Servo.class, "shooterLoader");
        shooterLoader.setPosition(SLHOME);
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
        telemetry.addData("Status", "Initialized");
        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        fl = hardwareMap.get(DcMotor.class, "front_left");
        fr = hardwareMap.get(DcMotor.class, "front_right");
        bl = hardwareMap.get(DcMotor.class, "back_left");
        br = hardwareMap.get(DcMotor.class, "back_right");
        shooter = hardwareMap.get(DcMotor.class, "shooter");
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        collector.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
//        leftDrive.setDirection(DcMotor.Direction.FORWARD);
//        rightDrive.setDirection(DcMotor.Direction.REVERSE);

        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");

        waitForStart();
        //Thread imuthread = new Thread(imuRead);
        //imuthread.start();
        Thread shooterThread = new Thread(shooterRun);
        Thread collectorThread = new Thread(collectRun);
        Thread driveThread = new Thread(driverRun);
        Thread shooterLoaderThread = new Thread(shooterLoaderRun);
        driveThread.start();
        collectorThread.start();
        shooterThread.start();
        shooterLoaderThread.start();
        while (opModeIsActive()) {


            /*
             * Code to run ONCE after the driver hits STOP.
             */

        }
    }


    Runnable driverRun = new Runnable() {
        @Override
        public void run() {
            long prevTime = 0;
            while (opModeIsActive()) {

                // Read controller variables.
                double forwardpower = Math.sin(gamepad1.left_stick_y * Math.PI / 2) * globalpowerfactor;
                double sidepower = Math.sin(-gamepad1.left_stick_x * Math.PI / 2) * globalpowerfactor;
                double turnpower = Math.sin(-gamepad1.right_stick_x * Math.PI / 2) * globalpowerfactor;

                // Calculate DC Motor Power.
                fl.setPower((forwardpower + sidepower + turnpower));
                bl.setPower((forwardpower - sidepower + turnpower));
                fr.setPower(-(forwardpower - sidepower - turnpower));
                br.setPower(-(forwardpower + sidepower - turnpower));

                // Do gearbox increase.
                if (gamepad1.y && (System.currentTimeMillis() - 500 > prevTime)) {
                    prevTime = System.currentTimeMillis();
                    if ((globalpowerfactor + 0.1) < 1) {
                        globalpowerfactor += 0.1;
                    }
                }


                if (gamepad1.a && (System.currentTimeMillis() - 500 > prevTime)) {
                    prevTime = System.currentTimeMillis();
                    if ((globalpowerfactor - 0.1) > 0) {
                        globalpowerfactor -= 0.1;
                    }
                }
            }
        }


    };

    /*Runnable imuRead = new Runnable() {
        @Override
        public void run() {                            // Make function
            while (opModeIsActive()) {
                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                telemetry.addLine() .addData("Z Coordinate (Rotation)", formatAngle(AngleUnit.DEGREES, angles.firstAngle));
                telemetry.update();
            }
        }
    };*/

    Runnable collectRun = new Runnable() {
        @Override
        public void run() {
            boolean collectorIsEnabled = false;
            CollectorDirection collectorDirection = null;
            long prevTime = 0;
            while (opModeIsActive()) {
                // Set collector power.
                if (collectorIsEnabled) {
                    if (collectorDirection == CollectorDirection.BACK) {
                        collector.setPower(1);
//                        long prevpostime = System.currentTimeMillis();
//                        int prevpos = collector.getCurrentPosition();
//                        if (collector.getCurrentPosition() > prevpos) {
//                            double collectrpower = calculateshooterpower(prevpostime, System.currentTimeMillis(), 288, 70, 0.1, 0.5);
//                            collector.setPower(collectrpower);
//                            prevpos = collector.getCurrentPosition();
//                            prevpostime = System.currentTimeMillis();
//                        }
                    } else if (collectorDirection == CollectorDirection.FORWARD) {
                        collector.setPower(-1);
                    } else {
                        collector.setPower(0);
                    }
                } else {
                    collector.setPower(0);
                }

                // Do collector toggles.
                if (gamepad2.x && (System.currentTimeMillis() - 500 > prevTime)) {
                    prevTime = System.currentTimeMillis();
                    if (collectorDirection == CollectorDirection.BACK) {
                        if (collectorIsEnabled) {
                            collectorIsEnabled = false;
                        } else {
                            collectorIsEnabled = true;
                        }
                    } else {
                        collectorDirection = CollectorDirection.BACK;
                        collectorIsEnabled = true;
                    }
                }

                if (gamepad2.b && (System.currentTimeMillis() - 500 > prevTime)) {
                    prevTime = System.currentTimeMillis();
                    if (collectorDirection == CollectorDirection.FORWARD) {
                        if (collectorIsEnabled) {
                            collectorIsEnabled = false;
                        } else {
                            collectorIsEnabled = true;
                        }
                    } else {
                        collectorDirection = CollectorDirection.FORWARD;
                        collectorIsEnabled = true;
                    }

                }
            }
        }
    };

    Runnable shooterRun = new Runnable() {
        @Override
        public void run() {
            long prevTime = 0;
            boolean shooterIsEnabled = false;
            while (opModeIsActive()) {
                if (shooterIsEnabled) {
                    shooter.setPower(1);
                } else {
                    shooter.setPower(0);
                }

                if (gamepad2.left_bumper && (System.currentTimeMillis() - 500 > prevTime)) {
                    prevTime = System.currentTimeMillis();
                    shooterIsEnabled = false;
                }

                if (gamepad2.right_bumper && (System.currentTimeMillis() - 500 > prevTime)) {
                    prevTime = System.currentTimeMillis();
                    shooterIsEnabled = true;
                }
            }
        }
    };
    Runnable shooterLoaderRun = new Runnable() {
        @Override
        public void run() {
            long prevTime = 0;
            while (opModeIsActive()) {
                if (gamepad2.y && ((System.currentTimeMillis() - 400) > prevTime)) {
                    shooterLoader.setPosition(1);
                    prevTime = System.currentTimeMillis();
                    while ((System.currentTimeMillis() - 800) < prevTime) {
                    }
                    shooterLoader.setPosition(0);
                }
            }
        }
    };


    /////////////////////////////////////
    //            FORMATTING           //
    /////////////////////////////////////

    String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    String formatDegrees(double degrees) {
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }
        double calculateshooterpower( long timeprevioustick, long currenttick, long ticksprerotation, long targetrpm, double p, double basespeed){
            long timeinterval = currenttick - timeprevioustick;
            long expectedtimeinterval = 60 / (targetrpm * ticksprerotation);
            long error = expectedtimeinterval - timeinterval;
            double connrection = p * error;
            double outputpower = basespeed + connrection;
            return outputpower;
        }
}

