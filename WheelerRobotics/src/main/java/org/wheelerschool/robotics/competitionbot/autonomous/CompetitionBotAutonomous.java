package org.wheelerschool.robotics.competitionbot.autonomous;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.UltrasonicSensor;

import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.wheelerschool.robotics.competitionbot.CompetitionBotConfig;
import org.wheelerschool.robotics.library.navigation.TranslationMotorNavigation;
import org.wheelerschool.robotics.library.util.LinearOpModeActiveCallable;
import org.wheelerschool.robotics.library.util.LinearOpModeUtil;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.Callable;

/**
 * An abstract OpMode which allows for autonomous navigation of the competition bot.
 * It is made for implementation and setting of the un-initialized variables.
 *
 * @author luciengaitskell
 * @since 161124
 */

public abstract class CompetitionBotAutonomous extends LinearOpMode {
    /**
     * Make sure to set all configuration variables below:
     */
    //      Wall Follow Motors:
    public List<DcMotor> closeMotors;
    public List<DcMotor> fartherMotors;
    //      Sensors:
    public UltrasonicSensor sideUltrasonicSensor;
    //      Other:
    public double AFTER_ENCODER_ROTATE_ANGLE; // The relative angle to turn after the initial drive with encoders
    public VectorF FIRST_BEACON_LOCATION;
    public VectorF FIRST_BEACON_PRESS_LOCATION;
    public double TOWARDS_BEACON_ANGLE;
    public double PRE_WALL_FOLLOW_ANGLE; // The angle to turn to before following the wall (radians)
    public double POST_WALL_FOLLOW_ROTATE_ANGLE; // (relative)
    public VectorF SECOND_BEACON_INITIAL_LOCATION;
    public VectorF SECOND_BEACON_PRESS_LOCATION;
    public int[] DESIRED_BEACON_COLOR;


    // Hardware Setup:
    //      Robot:
    public CompetitionBotConfig robot;
    //      Motor:
    public List<DcMotor> leftMotors = new ArrayList<>();
    public List<DcMotor> rightMotors = new ArrayList<>();


    // Config:
    public static String AUTO_FULL_LOG_TAG = "Comp Bot Auto Full";
    public static String AUTO_STATE_LOG_TAG = "Comp Bot Auto State";
    private static long MAX_TIME_TIMEOUT = 200; // MAX time until TIMEOUT when running OpMode (millis)
    private static double NO_BEACON_ROTATE_SPEED = 0.25;
    private static double MINIMUM_ROTATION_DIFF = AngleUnit.RADIANS.fromUnit(AngleUnit.DEGREES, 5);
    private static double ROBOT_ROTATION_GAIN = 1.5;
    private static long MINIMUM_ENCODER_DRIVE_VALUE = 800;
    private static long ENCODER_DRIVE_RAMP_DOWN_VALUE = 3000;
    private static double WALL_FOLLOW_FRONT_SPEED = 0.4;
    private static double NOMINAL_DISTANCE = 19;
    private static double MAXIMUM_VALUE_DIFF = 50;
    private double MIN_LINE_REFLECT_AMT = 0.4; // TODO: UPDATE THIS VALUE


    // OpMode:
    public void runOpMode() throws InterruptedException {
        // Hardware Setup:
        //      Robot:
        robot = new CompetitionBotConfig(hardwareMap, telemetry, false, new LinearOpModeActiveCallable(this));
        robot.setUpIMU();

        //      Motors (for reference in extending an class to set the closer and farther motors):
        this.leftMotors.addAll(robot.leftMotors);
        this.rightMotors.addAll(robot.rightMotors);

        // Wait for start button to be pushed:
        LinearOpModeUtil.runWhileWait(this, new Callable<Void>() {
            @Override
            public Void call() throws Exception {
                robot.vuforia.readData();
                telemetry.addData("Trackables", robot.vuforia.lastTrackableData);
                telemetry.update();
                return null;
            }
        });
        waitForStart();

        // Autonomous Sections:
        // Drive forward by encoder counts:
        Log.i(AUTO_STATE_LOG_TAG, "Initial drive by encoder");
        robot.driveForwardByEncoder(1, 1, 7000);

        // Rotate robot to angle towards beacon
        Log.i(AUTO_STATE_LOG_TAG, "Rotate after initial drive");
        robot.rotateRobotIMU(AFTER_ENCODER_ROTATE_ANGLE, 1);

        //      Drive to the wall:
        Log.i(AUTO_STATE_LOG_TAG, "Drive to initial first beacon");
        Double robotRot = robot.driveToPosition(FIRST_BEACON_LOCATION, 1.5);
        // Log final robot angle:
        Log.i(AUTO_STATE_LOG_TAG, "Robot Angle (after first beacon drive): " + robotRot);

        //      Rotate to wall (using Vuforia):
        Log.i(AUTO_STATE_LOG_TAG, "Towards First Beacon Angle: " + TOWARDS_BEACON_ANGLE);
        Log.i(AUTO_STATE_LOG_TAG, "Turning...");
        robotRot = robot.rotateRobotVision(TOWARDS_BEACON_ANGLE, ROBOT_ROTATION_GAIN);
        Log.i(AUTO_STATE_LOG_TAG, "SHOULD BE FACING FIRST BEACON");
        // Log final robot angle:
        Log.i(AUTO_STATE_LOG_TAG, "Robot Angle (after first beacon alignment): " + robotRot);

        // Drive in to press beacon:
        Log.i(AUTO_STATE_LOG_TAG, "Drive to first beacon press position");
        robotRot = robot.driveToPosition(FIRST_BEACON_PRESS_LOCATION, 1.5);

        Log.i(AUTO_STATE_LOG_TAG, "Towards First Beacon Angle: " + TOWARDS_BEACON_ANGLE);
        Log.i(AUTO_STATE_LOG_TAG, "Turning...");
        robotRot = robot.rotateRobotVision(TOWARDS_BEACON_ANGLE, ROBOT_ROTATION_GAIN);

        robot.idleMotors();

        Log.i(AUTO_STATE_LOG_TAG, "CLICK BEACON ONE HERE!");
        robot.pushBeacon(DESIRED_BEACON_COLOR);

        Log.i(AUTO_STATE_LOG_TAG, "Reverse from first beacon, to allow for rotation room");
        robot.driveForwardByEncoder(0.8, 1, -1500);

        // Sleep to break between rotate towards wall and rotate away
        Thread.sleep(500);

        //      Rotate to follow wall:
        // Log the needed angle:
        Log.i(AUTO_STATE_LOG_TAG, "Needed angle for wall follow: " + PRE_WALL_FOLLOW_ANGLE);

        // Only continue if wasn't interrupted:
        if (robotRot != null) {
            // Calculate needed relative rotation:
            double rotationAngle = TranslationMotorNavigation.angleDifference(PRE_WALL_FOLLOW_ANGLE, robotRot);
            // Log the relative rotation:
            Log.i(AUTO_STATE_LOG_TAG, "Relative rotation for wall follow: " + rotationAngle);
            // Start IMU based robot rotation:
            Log.i(AUTO_STATE_LOG_TAG, "Rotate to wall follow angle by IMU");
            robot.rotateRobotIMU(rotationAngle, ROBOT_ROTATION_GAIN);

            // Follow the wall:
            Log.i(AUTO_STATE_LOG_TAG, "\"Follow\" the wall! (use IMU to drive straight)");
            robot.driveForwardByIMU(0, ROBOT_ROTATION_GAIN, 0.8, 5000, 0.4);
            Log.i(AUTO_STATE_LOG_TAG, "DETECTED LINE (HOPEFULLY THE SECOND BEACON'S)!");

            Thread.sleep(100);

            // Rotate to second beacon:
            Log.i(AUTO_STATE_LOG_TAG, "Relative rotation for second beacon alignment: " + POST_WALL_FOLLOW_ROTATE_ANGLE);
            Log.i(AUTO_STATE_LOG_TAG, "(Mostly) Align with second beacon, by rotate with IMU");
            robot.rotateRobotIMU(POST_WALL_FOLLOW_ROTATE_ANGLE, 1);

            // Drive to press beacon:
            Log.i(AUTO_STATE_LOG_TAG, "Drive to second beacon initial location");
            robot.driveToPosition(SECOND_BEACON_INITIAL_LOCATION, 2);

            Log.i(AUTO_STATE_LOG_TAG, "Desired angle for beacon alignment: " + TOWARDS_BEACON_ANGLE);
            Log.i(AUTO_STATE_LOG_TAG, "Rotate to align with beacon");
            robot.rotateRobotVision(TOWARDS_BEACON_ANGLE, ROBOT_ROTATION_GAIN);

            Log.i(AUTO_STATE_LOG_TAG, "Drive to second beacon push location");
            robot.driveToPosition(SECOND_BEACON_PRESS_LOCATION, 1);
            Log.i(AUTO_STATE_LOG_TAG, "CLICK BEACON TWO HERE!");

            Log.i(AUTO_STATE_LOG_TAG, "Desired angle for beacon alignment: " + TOWARDS_BEACON_ANGLE);
            Log.i(AUTO_STATE_LOG_TAG, "Rotate to align with beacon (second time)");
            robot.rotateRobotVision(TOWARDS_BEACON_ANGLE, ROBOT_ROTATION_GAIN);

            robot.pushBeacon(DESIRED_BEACON_COLOR);
        } else {  // This means that the drive to position was interrupted:
            Log.e(AUTO_STATE_LOG_TAG, "Final Robot angle was 'null' (interrupted). ENDING!");
        }
    }
}
