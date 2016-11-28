package org.wheelerschool.robotics.library.vision;

import com.vuforia.HINT;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.teamcode.R;
import org.wheelerschool.robotics.config.Config;

import java.util.HashMap;
import java.util.Map;

/**
 * Created by luciengaitskell on 11/25/16.
 */

public class VuforiaTrackableLocation {
    public static float MM_PER_INCH = 25.4f;

    private OpenGLMatrix phoneLocation;
    private VuforiaTrackables trackables;
    public VuforiaLocalizer.Parameters params;

    public VuforiaTrackableLocation(OpenGLMatrix phoneLocation) {
        this.phoneLocation = phoneLocation;

        this.params = new VuforiaLocalizer.Parameters(R.id.cameraMonitorViewId);
        this.params.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        this.params.vuforiaLicenseKey = Config.VUFORIA_KEY;
        this.params.cameraMonitorFeedback = VuforiaLocalizer.Parameters.CameraMonitorFeedback.AXES;

        VuforiaLocalizer vuforia = ClassFactory.createVuforiaLocalizer(this.params);
        Vuforia.setHint(HINT.HINT_MAX_SIMULTANEOUS_IMAGE_TARGETS, 4);

        this.trackables = vuforia.loadTrackablesFromAsset("FTC_2016-17");
        this.trackables.get(0).setName("wheels");
        this.trackables.get(1).setName("tools");
        this.trackables.get(2).setName("legos");
        this.trackables.get(3).setName("gears");
    }

    public void activate() {
        this.trackables.activate();
    }

    // Trackables:
    public class Trackable {
        public VuforiaTrackable trackable;
        public VuforiaTrackableDefaultListener listener;

        public VectorF translation = null;
        public Orientation orientation = null;
        public Boolean visible = null;

        public Trackable(VuforiaTrackable trackable) {
            this(new OpenGLMatrix(new float[]{0,0,0}), VuforiaLocalizer.CameraDirection.BACK, trackable);
        }

        public Trackable(OpenGLMatrix phoneLocation, VuforiaLocalizer.CameraDirection cameraDirection, VuforiaTrackable trackable) {
            this.trackable = trackable;
            this.listener = (VuforiaTrackableDefaultListener) trackable.getListener();
            this.listener.setPhoneInformation(phoneLocation, cameraDirection);
        }

        public OpenGLMatrix getPose() {
            return this.listener.getPose();
        }

        public void readData() {
            this.visible = this.listener.isVisible();

            OpenGLMatrix pose = getPose();

            this.translation = ((pose == null) ? null : pose.getTranslation());
            this.orientation = ((pose == null) ? null : Orientation.getOrientation(pose, AxesReference.EXTRINSIC,
                                                                                   AxesOrder.XYZ, AngleUnit.RADIANS));
        }
    }


    public Map<String, Trackable> getTrackables() {
        Map<String, Trackable> beacons = new HashMap<>();

        for (VuforiaTrackable track : this.trackables) {
            beacons.put(track.getName(), new Trackable(this.phoneLocation, this.params.cameraDirection, track));
        }
        return beacons;
    }
}
