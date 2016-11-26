package org.wheelerschool.robotics.vuforia;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.wheelerschool.robotics.library.vision.VuforiaTrackableLocation;

import java.util.Map;

/**
 * Created by luciengaitskell on 11/25/16.
 */

@TeleOp
public class VuforiaTrackableLocationTest extends LinearOpMode {
    VuforiaTrackableLocation targetsLocation;

    @Override
    public void runOpMode() throws InterruptedException {

        OpenGLMatrix phoneLocation = OpenGLMatrix
                .translation((15 * VuforiaTrackableLocation.MM_PER_INCH)/2 ,
                        -1.75f * VuforiaTrackableLocation.MM_PER_INCH,
                        4 * VuforiaTrackableLocation.MM_PER_INCH)
                .multiplied(Orientation.getRotationMatrix(
                        AxesReference.EXTRINSIC, AxesOrder.XYZ,
                        AngleUnit.DEGREES, 0, -90, 0));

        targetsLocation = new VuforiaTrackableLocation(phoneLocation);

        waitForStart();
        targetsLocation.activate();

        Map<String, VuforiaTrackableLocation.TrackablesData> trackables = targetsLocation.getTrackables();

        while (opModeIsActive()) {
            for (Map.Entry<String, VuforiaTrackableLocation.TrackablesData> entry : trackables.entrySet()) {
                VuforiaTrackableLocation.TrackablesData track = entry.getValue();
                track.readData();

                telemetry.addData(entry.getKey() + "-Visible", track.visible);

                if (track.visible == Boolean.TRUE) {
                    telemetry.addData(entry.getKey() + "-Translation", track.translation);
                    telemetry.addData(entry.getKey() + "-Orientation", track.orientation);
                }
            }

            telemetry.update();
        }
    }
}
