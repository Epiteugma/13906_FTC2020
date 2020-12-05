package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.vuforia.HINT;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

/*
 * This OpMode was written for the VuforiaDemo Basics video. This demonstrates basic principles of
 * using VuforiaDemo in FTC.
 */
@Autonomous(name = "Vuforia")
public class VuforiaDemo extends LinearOpMode
{
    // Variables to be used for later
    private VuforiaLocalizer vuforiaLocalizer;
    private VuforiaLocalizer.Parameters parameters;
    private VuforiaTrackables visionTargets;
    private VuforiaTrackable target;
    private VuforiaTrackableDefaultListener listener;

    private OpenGLMatrix lastKnownLocation;
    private OpenGLMatrix phoneLocation;

    private static final String VUFORIA_KEY = "AXMZ1Sj/////AAABmUx9MvSm30+BhSsSC99gi4Ujtkus2hJIZS0gVIZQUeSPEcIITEOnwlBZpbMvw9zEBpF7fu28GyAgYY3vGjDNcFcyeuEhKSKJ1A4URgGWeqJJ5HsA+2K1fJY7zhgBTjNt5it80mzhs7y2Jba1Vsjxe4LtVWSgWyzHalth1+aky0tEE9ALjWeuE/3RV/fUMgO2QPbjYg8UvPEjZLhFwSLVhf/ku23jTE1JDtmfZCgdrCCYGhdvuMCBEzIijVf8HyhxaWWtnhYZi77RcaDQdKQURAcuSpM5HRRygLkoFZ8B9mjEcFxkxQ4rJFZFb4xN2j2XXPurSl70Ht7IAQLIiJDJAAY2LdLzDggUZlNhp2LY8TrI"; // Insert your own key here

    private float robotX = 0;
    private float robotY = 0;
    private float robotAngle = 0;

    public void runOpMode() throws InterruptedException
    {
        setupVuforia();

        // We don't know where the robot is, so set it to the origin
        // If we don't include this, it would be null, which would cause errors later on
        lastKnownLocation = createMatrix(0, 0, 0, 0, 0, 0);

        waitForStart();

        // Start tracking the targets
        visionTargets.activate();

        while(opModeIsActive())
        {
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

    private void setupVuforia()
    {
        // Setup parameters to create localizer
        parameters = new VuforiaLocalizer.Parameters(R.id.cameraMonitorViewId); // To remove the camera view from the screen, remove the R.id.cameraMonitorViewId
        parameters.vuforiaLicenseKey = "AXMZ1Sj/////AAABmUx9MvSm30+BhSsSC99gi4Ujtkus2hJIZS0gVIZQUeSPEcIITEOnwlBZpbMvw9zEBpF7fu28GyAgYY3vGjDNcFcyeuEhKSKJ1A4URgGWeqJJ5HsA+2K1fJY7zhgBTjNt5it80mzhs7y2Jba1Vsjxe4LtVWSgWyzHalth1+aky0tEE9ALjWeuE/3RV/fUMgO2QPbjYg8UvPEjZLhFwSLVhf/ku23jTE1JDtmfZCgdrCCYGhdvuMCBEzIijVf8HyhxaWWtnhYZi77RcaDQdKQURAcuSpM5HRRygLkoFZ8B9mjEcFxkxQ4rJFZFb4xN2j2XXPurSl70Ht7IAQLIiJDJAAY2LdLzDggUZlNhp2LY8TrI";
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        parameters.useExtendedTracking = false;
        vuforiaLocalizer = ClassFactory.createVuforiaLocalizer(parameters);

        // These are the vision targets that we want to use
        // The string needs to be the name of the appropriate .xml file in the assets folder
        visionTargets = vuforiaLocalizer.loadTrackablesFromAsset("FTC_2016-17");
        Vuforia.setHint(HINT.HINT_MAX_SIMULTANEOUS_IMAGE_TARGETS, 4);

        // Setup the target to be tracked
        target = visionTargets.get(0); // 0 corresponds to the wheels target
        target.setName("Wheels Target");
        target.setLocation(createMatrix(0, 500, 0, 90, 0, 90));

        // Set phone location on robot
        phoneLocation = createMatrix(0, 225, 0, 90, 0, 0);

        // Setup listener and inform it of phone information
        listener = (VuforiaTrackableDefaultListener) target.getListener();
        listener.setPhoneInformation(phoneLocation, parameters.cameraDirection);
    }

    // Creates a matrix for determining the locations and orientations of objects
    // Units are millimeters for x, y, and z, and degrees for u, v, and w
    private OpenGLMatrix createMatrix(float x, float y, float z, float u, float v, float w)
    {
        return OpenGLMatrix.translation(x, y, z).
                multiplied(Orientation.getRotationMatrix(
                        AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES, u, v, w));
    }

    // Formats a matrix into a readable string
    private String formatMatrix(OpenGLMatrix matrix)
    {
        return matrix.formatAsTransform();
    }
}
