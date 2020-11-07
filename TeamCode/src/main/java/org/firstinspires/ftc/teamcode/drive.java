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
    private double sidepowerfactor = 0.75;
    private double forwardpowerfactor = 0.85;
    private double turnpowerfactor = 0.7;
    private DcMotor collector = null;
    private DcMotor shooter=null;
    BNO055IMU imu;
    Orientation angles;
    Acceleration gravity;
    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void runOpMode() {
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        collector = hardwareMap.get(DcMotor.class, "collector");
        shooterLoader = hardwareMap.get(Servo.class, "servoloader");
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
        Thread drivethread = new Thread(driver);
        Thread imuthread = new Thread(imuRead);
        Thread gearboxYthread = new Thread(gearboxY);
        Thread gearboxAthread = new Thread(gearboxA);
        Thread collectorRunThread = new Thread(runCollector);
        Thread collectorToggleThread = new Thread(toggleCollector);
        Thread shooterRunThread = new Thread(runshooter);
        Thread shooterToggleThread = new Thread(toggleshooter);
        gearboxAthread.start();
        gearboxYthread.start();
        collectorRunThread.start();
        collectorToggleThread.start();
        shooterRunThread.start();
        shooterToggleThread.start();
        imuthread.start();
        drivethread.start();
        while (opModeIsActive()) {

        }

        /*
         * Code to run ONCE after the driver hits STOP
         */

    }

    Runnable driver = new Runnable() {
        @Override
        public void run() {
            while (opModeIsActive()) {

                // Read controller variables
                double forwardpower =  Math.sin(gamepad1.left_stick_y*Math.PI/2) * forwardpowerfactor;
                double sidepower = Math.sin(-gamepad1.left_stick_x*Math.PI/2) * sidepowerfactor;
                double turnpower = Math.sin(-gamepad1.right_stick_x *Math.PI/2) * turnpowerfactor;

                // Calculate DC Motor Power.
                fl.setPower((forwardpower + sidepower + turnpower));
                bl.setPower((forwardpower - sidepower + turnpower));
                fr.setPower(-(forwardpower - sidepower - turnpower));
                br.setPower(-(forwardpower + sidepower - turnpower));
            }
        }


    };

    Runnable imuRead = new Runnable() {
        @Override
        public void run() {
            while (opModeIsActive()) {
                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                telemetry.addLine() .addData("Z Coordinate (Rotation)", formatAngle(AngleUnit.DEGREES, angles.firstAngle));
                telemetry.update();
            }
        }
    };

    Runnable gearboxY = new Runnable() {
        @Override
        public void run() {
            while (opModeIsActive()) {
                if(gamepad1.y) {
                    boolean ranFirstTime = false;
                    if ((forwardpowerfactor + 0.1) < 1) {
                        forwardpowerfactor += 0.1;
                        try {
                            Thread.sleep(500);
                        } catch (InterruptedException e) {

                        }
                    }
                }
            }
        }
    };

    Runnable gearboxA = new Runnable() {
        @Override
        public void run() {
            while (opModeIsActive()) {
                if(gamepad1.a) {
                    boolean ranFirstTime = false;
                    if ((forwardpowerfactor - 0.1) < 1) {
                        forwardpowerfactor -= 0.1;
                        try {
                            Thread.sleep(500);
                        } catch (InterruptedException e) {

                        }
                    }
                }
            }
        }
    };
    boolean collectorIsEnabled = false;
    Runnable runCollector = new Runnable() {
        @Override
        public void run() {
            while(opModeIsActive()) {
                if (collectorIsEnabled) {
                    collector.setPower(-1.0);
                }
                else {
                    collector.setPower(0);
                }
            }
        }
    };

    Runnable toggleCollector = new Runnable() {
        @Override
        public void run() {
            while (opModeIsActive()) {
                if (gamepad2.x) {
                    if (collectorIsEnabled) {
                        collectorIsEnabled = false;
                    } else {
                        collectorIsEnabled = true;
                    }
                    try {
                        Thread.sleep(250);
                    } catch (InterruptedException e) {
                        e.printStackTrace();
                    }
                }
            }
        }
    };
    boolean shooterIsEnabled = false;
    Runnable runshooter = new Runnable() {
        @Override
        public void run() {
            while(opModeIsActive()) {
                if (shooterIsEnabled) {
                    shooter.setPower(-1.0);
                }
                else {
                   shooter.setPower(0);
                }
            }
        }
    };

    Runnable toggleshooter = new Runnable() {
        @Override
        public void run() {
            while (opModeIsActive()) {
                if (gamepad2.y) {
                    if (shooterIsEnabled) {
                        shooterIsEnabled = false;
                    } else {
                        shooterIsEnabled = true;
                    }
                    try {
                        Thread.sleep(250);
                    } catch (InterruptedException e) {
                        e.printStackTrace();
                    }
                }
            }
        }
    };

    Runnable shoot = new Runnable() {
        @Override
        public void run() {
            while (opModeIsActive()) {
                if(gamepad2.a) {
                    shooter.setTargetPosition(shooter.getCurrentPosition()+30);
                    try {
                        Thread.sleep(500);
                    } catch (InterruptedException e) {
                        e.printStackTrace();
                    }
                    shooter.setTargetPosition(shooter.getCurrentPosition()-30);
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

    String formatDegrees(double degrees){
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }
}

