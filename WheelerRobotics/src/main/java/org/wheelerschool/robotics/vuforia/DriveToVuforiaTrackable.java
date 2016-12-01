package org.wheelerschool.robotics.vuforia;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.wheelerschool.robotics.library.navigation.TranslationMotorNavigation;
import org.wheelerschool.robotics.library.util.DcMotorUtil;
import org.wheelerschool.robotics.library.vision.VuforiaTrackableLocation;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;

/**
 * Created by luciengaitskell on 11/25/16.
 */

@TeleOp
public class DriveToVuforiaTrackable extends LinearOpMode {
    // Motors:
    List<DcMotor> leftMotors;
    List<DcMotor> rightMotors;

    VuforiaTrackableLocation targetsLocation;
    TranslationMotorNavigation navigation;

    @Override
    public void runOpMode() throws InterruptedException {
        // Motors setup:
        leftMotors = new ArrayList<>();
        leftMotors.add(hardwareMap.dcMotor.get("leftBack"));
        leftMotors.add(hardwareMap.dcMotor.get("leftFront"));
        rightMotors = new ArrayList<>();
        rightMotors.add(hardwareMap.dcMotor.get("rightBack"));
        rightMotors.add(hardwareMap.dcMotor.get("rightFront"));

        this.navigation = new TranslationMotorNavigation();

        OpenGLMatrix phoneLocation = OpenGLMatrix
                .translation((15 * VuforiaTrackableLocation.MM_PER_INCH)/2 ,
                        -1.75f * VuforiaTrackableLocation.MM_PER_INCH,
                        4 * VuforiaTrackableLocation.MM_PER_INCH)
                .multiplied(Orientation.getRotationMatrix(
                        AxesReference.EXTRINSIC, AxesOrder.XYZ,
                        AngleUnit.DEGREES, 0, -90, 0));

        targetsLocation = new VuforiaTrackableLocation(phoneLocation);

        targetsLocation.activate();

        waitForStart();

        Map<String, VuforiaTrackableLocation.Trackable> trackables = targetsLocation.getTrackables();

        String trackableName = "gears";

        VuforiaTrackableLocation.Trackable wheels = trackables.get(trackableName);
        while (opModeIsActive()) {
            VuforiaTrackableLocation.Trackable.TrackableData trackableData = wheels.getData();

            telemetry.addData(trackableName + "-Visible", trackableData.visible);

            // If the trackable is visible:
            if (trackableData.visible) {
                VectorF translation = trackableData.translation;
                Orientation orientation = trackableData.orientation;
                telemetry.addData(trackableName + "-Translation", translation);
                telemetry.addData(trackableName + "-Orientation", orientation);

                TranslationMotorNavigation.NavigationData navData =
                        this.navigation.calculateNavigationData(translation.get(2),
                                translation.get(1), orientation.secondAngle);

                telemetry.addData("Left Power", navData.leftMotorPower);
                telemetry.addData("Right Power", navData.rightMotorPower);

                DcMotorUtil.setMotorsPower(this.leftMotors, navData.leftMotorPower);
                DcMotorUtil.setMotorsPower(this.rightMotors, navData.rightMotorPower);
            } else {
                DcMotorUtil.setMotorsPower(this.leftMotors, 0);
                DcMotorUtil.setMotorsPower(this.rightMotors, 0);
            }

            telemetry.update();
            Thread.sleep(50);
        }
    }
}
