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
public class CompetitionBotAutonomousBlue extends CompetitionBotAutonomous {
    @Override
    public void runOpMode() throws InterruptedException {
        // Sensor Setup:
        this.AFTER_ENCODER_ROTATE_ANGLE = AngleUnit.RADIANS.fromDegrees(-80);
        this.FIRST_BEACON_LOCATION = new VectorF(11.5f * VuforiaLocation.MM_PER_INCH, 57 * VuforiaLocation.MM_PER_INCH, 0);
        this.FIRST_BEACON_PRESS_LOCATION = this.FIRST_BEACON_LOCATION.added(new VectorF(0, 5.5f * VuforiaLocation.MM_PER_INCH, 0));
        Log.d("CompBotAutoBlue", this.FIRST_BEACON_LOCATION.toString());
        this.PRE_WALL_FOLLOW_ANGLE = AngleUnit.RADIANS.fromUnit(AngleUnit.DEGREES, 175);
        this.TOWARDS_BEACON_ANGLE = AngleUnit.RADIANS.fromUnit(AngleUnit.DEGREES, 90);
        this.SECOND_BEACON_INITIAL_LOCATION = new VectorF(-35f * VuforiaLocation.MM_PER_INCH, 58 * VuforiaLocation.MM_PER_INCH, 0);
        this.POST_WALL_FOLLOW_ROTATE_ANGLE = AngleUnit.RADIANS.fromUnit(AngleUnit.DEGREES, -90);
        this.SECOND_BEACON_PRESS_LOCATION = this.SECOND_BEACON_INITIAL_LOCATION.added(new VectorF(0, 4.75f * VuforiaLocation.MM_PER_INCH, 0));
        this.sideUltrasonicSensor = hardwareMap.ultrasonicSensor.get("leftUltrasound");
        this.closeMotors = this.rightMotors;
        this.fartherMotors = this.leftMotors;
        this.DESIRED_BEACON_COLOR = new int[] {0, 0, 4};

        super.runOpMode();
    }
}
