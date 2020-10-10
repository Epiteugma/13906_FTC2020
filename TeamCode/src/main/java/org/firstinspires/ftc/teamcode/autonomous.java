package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name = "Autonomous Test", group = "FTC_Cyprus_2020-21")
public class autonomous extends LinearOpMode {
    BNO055IMU imu;

    @Override
    public void runOpMode() throws InterruptedException {
        // Set up the parameters with which we will use our IMU. Note that integration
        // algorithm here just reports accelerations to the logcat log; it doesn't actually
        // provide positional information.
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
        final DcMotor fl = hardwareMap.get(DcMotor.class, "front_left");
        final DcMotor fr = hardwareMap.get(DcMotor.class, "front_right");
        final DcMotor bl = hardwareMap.get(DcMotor.class, "back_left");
        final DcMotor br = hardwareMap.get(DcMotor.class, "back_right");
        waitForStart();
        Runnable test = new Runnable() {
            @Override
            public void run() {
                fl.setPower(0.5);
                fr.setPower(-0.5);
                bl.setPower(0.5);
                br.setPower(-0.5);
                try {Thread.sleep(1000);}
                catch (Exception e) {}
                fl.setPower(0);
                fr.setPower(0);
                bl.setPower(0);
                br.setPower(0);
            }
        };
        Thread thread1 = new Thread(test);
        thread1.start();
        while(opModeIsActive()) {

        }
    }

}
