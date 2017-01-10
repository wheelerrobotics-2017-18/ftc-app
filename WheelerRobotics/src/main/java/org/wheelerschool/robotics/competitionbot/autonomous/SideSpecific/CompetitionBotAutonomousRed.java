package org.wheelerschool.robotics.competitionbot.autonomous.SideSpecific;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.wheelerschool.robotics.competitionbot.autonomous.CompetitionBotAutonomous;
import org.wheelerschool.robotics.library.vision.VuforiaLocation;

/**
 * Created by luciengaitskell on 12/6/16.
 */

@Autonomous
public class CompetitionBotAutonomousRed extends CompetitionBotAutonomous {
    @Override
    public void runOpMode() throws InterruptedException {
        // Sensor Setup:
        this.AFTER_ENCODER_ROTATE_ANGLE = AngleUnit.RADIANS.fromDegrees(80);
        this.FIRST_BEACON_LOCATION = new VectorF(-56 * VuforiaLocation.MM_PER_INCH, -12 * VuforiaLocation.MM_PER_INCH, 0);
        this.FIRST_BEACON_PRESS_LOCATION = this.FIRST_BEACON_LOCATION.added(new VectorF(-7.5f * VuforiaLocation.MM_PER_INCH, 0, 0));
        Log.d("CompBotAutoBlue", this.FIRST_BEACON_LOCATION.toString());
        this.PRE_WALL_FOLLOW_ANGLE = AngleUnit.RADIANS.fromUnit(AngleUnit.DEGREES, 90);
        this.TOWARDS_BEACON_ANGLE = AngleUnit.RADIANS.fromUnit(AngleUnit.DEGREES, 180);
        this.POST_WALL_FOLLOW_ROTATE_ANGLE = AngleUnit.RADIANS.fromUnit(AngleUnit.DEGREES, 80);
        this.SECOND_BEACON_INITIAL_LOCATION = new VectorF(-58 * VuforiaLocation.MM_PER_INCH, 35f * VuforiaLocation.MM_PER_INCH, 0);
        this.SECOND_BEACON_PRESS_LOCATION = this.SECOND_BEACON_INITIAL_LOCATION.added(new VectorF(-6f * VuforiaLocation.MM_PER_INCH, 0, 0));
        this.sideUltrasonicSensor = hardwareMap.ultrasonicSensor.get("rightUltrasound");
        this.closeMotors = this.leftMotors;
        this.fartherMotors = this.rightMotors;
        this.DESIRED_BEACON_COLOR = new int[] {4, 0, 0};

        super.runOpMode();
    }
}
