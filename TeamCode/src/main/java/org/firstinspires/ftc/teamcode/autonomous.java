package org.firstinspires.ftc.teamcode;

import android.graphics.Path;
import android.graphics.drawable.GradientDrawable;
import android.net.sip.SipSession;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.vuforia.HINT;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.*;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import java.util.Locale;

import org.firstinspires.ftc.robotcore.external.navigation.*;

@Autonomous(name = "Autonomous", group = "FTC_Cyprus_2020-21")
public class autonomous extends LinearOpMode {
    VuforiaLocalizer vuforiaLocalizer;
    VuforiaLocalizer.Parameters vparameters;
    VuforiaTrackables visionTargets;
    VuforiaTrackable target;
    VuforiaTrackableDefaultListener listener;

    float robotX = 0;
    float robotY = 0;
    float robotAngle = 0;

    OpenGLMatrix lastKnownLocation;
    OpenGLMatrix phoneLocation;

    BNO055IMU imu;
    Orientation angles;
    @Override
    public void runOpMode() throws InterruptedException {
        // Set up the parameters with which we will use our IMU. Note that integration
        // algorithm here just reports accelerations to the logcat log; it doesn't actually
        // provide positional information.
        setupVuforia();
        lastKnownLocation = createMatrix(0, 0, 0, 0, 0, 0);
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
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        final DcMotor fl = hardwareMap.get(DcMotor.class, "front_left");
        final DcMotor fr = hardwareMap.get(DcMotor.class, "front_right");
        final DcMotor bl = hardwareMap.get(DcMotor.class, "back_left");
        final DcMotor br = hardwareMap.get(DcMotor.class, "back_right");
        waitForStart();
        visionTargets.activate();
        while(opModeIsActive()) {
            // Ask the listener for the latest information on where the robot is
            OpenGLMatrix latestLocation = listener.getUpdatedRobotLocation();

            // The listener will sometimes return null, so we check for that to prevent errors
            if(latestLocation != null)
                lastKnownLocation = latestLocation;

            float[] coordinates = lastKnownLocation.getTranslation().getData();

            robotX = coordinates[0];
            robotY = coordinates[1];
            robotAngle = Orientation.getOrientation(lastKnownLocation, AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).thirdAngle;

            // Send information about whether the target is visible, and where the robot is
            telemetry.addData("Tracking " + target.getName(), listener.isVisible());
            telemetry.addData("Last Known Location", formatMatrix(lastKnownLocation));

            // Send telemetry and idle to let hardware catch up
            telemetry.update();
            idle();
        }
    }

    /////////////////////////////////////
    //             VUFORIA             //
    /////////////////////////////////////


    private void setupVuforia()
    {
        // Setup parameters to create localizer
        vparameters = new VuforiaLocalizer.Parameters(R.id.cameraMonitorViewId); // To remove the camera view from the screen, remove the R.id.cameraMonitorViewId
        vparameters.vuforiaLicenseKey = "AXMZ1Sj/////AAABmUx9MvSm30+BhSsSC99gi4Ujtkus2hJIZS0gVIZQUeSPEcIITEOnwlBZpbMvw9zEBpF7fu28GyAgYY3vGjDNcFcyeuEhKSKJ1A4URgGWeqJJ5HsA+2K1fJY7zhgBTjNt5it80mzhs7y2Jba1Vsjxe4LtVWSgWyzHalth1+aky0tEE9ALjWeuE/3RV/fUMgO2QPbjYg8UvPEjZLhFwSLVhf/ku23jTE1JDtmfZCgdrCCYGhdvuMCBEzIijVf8HyhxaWWtnhYZi77RcaDQdKQURAcuSpM5HRRygLkoFZ8B9mjEcFxkxQ4rJFZFb4xN2j2XXPurSl70Ht7IAQLIiJDJAAY2LdLzDggUZlNhp2LY8TrI";
        vparameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        vparameters.useExtendedTracking = false;
        vuforiaLocalizer = ClassFactory.createVuforiaLocalizer(vparameters);

        // These are the vision targets that we want to use
        // The string needs to be the name of the appropriate .xml file in the assets folder
        visionTargets = vuforiaLocalizer.loadTrackablesFromAsset("UltimateGoal");
        Vuforia.setHint(HINT.HINT_MAX_SIMULTANEOUS_IMAGE_TARGETS, 4);

        // Setup the target to be tracked
        target = visionTargets.get(0);
        target.setName("BlueTowerGoal");
        target = visionTargets.get(1);
        target.setName("RedTowerGoal");
        target = visionTargets.get(2);
        target.setName("RedAlliance");
        target = visionTargets.get(3);
        target.setName("BlueAlliance");
        target = visionTargets.get(4);
        target.setName("FrontWall");

        target.setLocation(createMatrix(0, 500, 0, 90, 0, 90));

        // Set phone location on robot
        phoneLocation = createMatrix(0, 225, 0, 90, 0, 0);

        // Setup listener and inform it of phone information
        listener = (VuforiaTrackableDefaultListener) target.getListener();
        listener.setPhoneInformation(phoneLocation, vparameters.cameraDirection);
    }

    /////////////////////////////////////
    //            FORMATTING           //
    /////////////////////////////////////
    private OpenGLMatrix createMatrix(float x, float y, float z, float u, float v, float w) {
        return OpenGLMatrix.translation(x, y, z)
                .multiplied(Orientation.getRotationMatrix(
                        AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES, u, v, w));
    }
    private String formatMatrix(OpenGLMatrix matrix) {
        return matrix.formatAsTransform();
    }
    String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }
    String formatDegrees(double degrees){
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }
}
