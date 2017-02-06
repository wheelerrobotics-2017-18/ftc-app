package org.wheelerschool.robotics.competitionbot;

import android.util.Log;

import com.qualcomm.hardware.adafruit.BNO055IMU;
import com.qualcomm.hardware.adafruit.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.UltrasonicSensor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.wheelerschool.robotics.library.navigation.ConstantDistanceMotorNavigation;
import org.wheelerschool.robotics.library.navigation.TranslationMotorNavigation;
import org.wheelerschool.robotics.library.util.DcMotorUtil;
import org.wheelerschool.robotics.library.vision.VuforiaLocation;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.Callable;

/**
 * Configuration values for the competition bot.
 *
 * @author luciengaitskell
 * @since 161216
 */

public class CompetitionBotConfig {
    private Callable<Boolean> getIsRunning;
    private HardwareMap hardwareMap;
    Telemetry telemetry;

    public class AdvancedServo {
        public Servo servo;
        private double retractedPos;
        private double extendedPos;

        public AdvancedServo(Servo servo) {
            this.servo = servo;
        }

        public void activatePusher(boolean extend) {
            if (extend) {
                servo.setPosition(extendedPos);
            } else {
                servo.setPosition(retractedPos);
            }
        }
    }

    // IMU:
    public BNO055IMU imu;

    // Autonomous Left/Right:
    public AdvancedServo pusherLeft;
    public AdvancedServo pusherRight;

    public static boolean defaultRobotForwards = true;
    public boolean robotForwards = defaultRobotForwards;
    public List<DcMotor> leftMotors = new ArrayList<>();
    public List<DcMotor> rightMotors = new ArrayList<>();

    public List<DcMotor> launcherMotors = new ArrayList<>();
    public enum LauncherMotorsState {
        DISABLE, IDLE, LAUNCH
    }

    public CRServo feederServo;

    public OpticalDistanceSensor feedDetector;

    public ColorSensor colorRight;
    public ColorSensor colorLeft;

    public OpticalDistanceSensor groundReflectSensor;

    //      Phone Location
    public OpenGLMatrix phoneLocation = OpenGLMatrix
            .translation(3.5f * VuforiaLocation.MM_PER_INCH,
                    2.25f * VuforiaLocation.MM_PER_INCH,
                    7f * VuforiaLocation.MM_PER_INCH)
            .multiplied(Orientation.getRotationMatrix(
                    AxesReference.INTRINSIC, AxesOrder.XYZ,
                    AngleUnit.DEGREES, 0, -90, 180));
    //      Vuforia Target Setup:
    public VuforiaLocation vuforia;

    public void setUpIMU() {
        //  Sensors:
        //      IMU:
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "AdafruitIMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        //          Retrieve and initialize the IMU:
        this.imu = hardwareMap.get(BNO055IMU.class, "imu");
        this.imu.initialize(parameters);
    }

    public CompetitionBotConfig(HardwareMap hardwareMap, Telemetry telemetry, Callable<Boolean> getIsRunning) {
        this(hardwareMap, telemetry, defaultRobotForwards, getIsRunning);
    }

    public CompetitionBotConfig(HardwareMap hardwareMap, Telemetry telemetry, boolean robotForwards, Callable<Boolean> getIsRunning) {
        this.vuforia = new VuforiaLocation(phoneLocation);

        this.getIsRunning = getIsRunning;
        this.telemetry = telemetry;
        this.hardwareMap = hardwareMap;


        // Pusher Servos:
        this.pusherLeft = new AdvancedServo(hardwareMap.servo.get("pusherLeft"));
        this.pusherLeft.retractedPos = 0;
        this.pusherLeft.extendedPos = 1;
        this.pusherLeft.activatePusher(false);
        this.pusherRight = new AdvancedServo(hardwareMap.servo.get("pusherRight"));
        this.pusherRight.servo.setDirection(Servo.Direction.REVERSE);
        this.pusherRight.retractedPos = 0;
        this.pusherRight.extendedPos = 1;
        this.pusherRight.activatePusher(false);

        // Drive Motors:
        this.leftMotors.add(this.hardwareMap.dcMotor.get("backLeft"));
        this.rightMotors.add(this.hardwareMap.dcMotor.get("backRight"));
        // Set robot direction:
        setRobotDirection(robotForwards);

        DcMotorUtil.setMotorsRunMode(this.leftMotors, DcMotor.RunMode.RUN_USING_ENCODER);
        DcMotorUtil.setMotorsRunMode(this.rightMotors, DcMotor.RunMode.RUN_USING_ENCODER);
        this.idleMotors();

        // Launcher Motors:
        launcherMotors.add(hardwareMap.dcMotor.get("launcherLeft"));
        launcherMotors.add(hardwareMap.dcMotor.get("launcherRight"));
        DcMotorUtil.setMotorsDirection(launcherMotors, DcMotorSimple.Direction.REVERSE);
        this.setLauncherState(LauncherMotorsState.DISABLE);

        // Launcher Feed Servo:
        feederServo = hardwareMap.crservo.get("feeder");
        feederServo.setPower(0);

        feedDetector = hardwareMap.opticalDistanceSensor.get("feedDetector");

        // Beacon Sensors:
        /// NOTE: Color sensors are flipped, because they are on a side referenced to the front
        ///     of the robot:
        colorLeft = hardwareMap.colorSensor.get("colorLeft");
        colorLeft.setI2cAddress(I2cAddr.create7bit(0x1e));
        colorLeft.enableLed(false);
        colorRight = hardwareMap.colorSensor.get("colorRight");
        colorRight.setI2cAddress(I2cAddr.create7bit(0x1f));
        colorRight.enableLed(false);

        // Line Detector:
        groundReflectSensor = hardwareMap.opticalDistanceSensor.get("groundODS");
    }

    public void setRobotDirection(boolean forwards) {
        if (robotForwards != forwards) {
            robotForwards = forwards;
            List<DcMotor> newRightMotors = this.leftMotors;
            List<DcMotor> newLeftMotors = this.rightMotors;
            this.leftMotors = newLeftMotors;
            this.rightMotors = newRightMotors;
        }

        DcMotorUtil.setMotorsDirection(this.leftMotors, DcMotorSimple.Direction.FORWARD);
        DcMotorUtil.setMotorsDirection(this.rightMotors, DcMotorSimple.Direction.REVERSE);
    }

    public double setLauncherState(LauncherMotorsState state) {
        double launcherMotorIdlePower = 0.1;
        double launcherMotorLaunchPower = 0.2;

        // Default launcher speed:
        double launcherPower = 0;
        if (state == LauncherMotorsState.IDLE){
            // Set the launcher power to idle mode:
            launcherPower = launcherMotorIdlePower;
        } else if (state == LauncherMotorsState.LAUNCH) {
            // Set the launcher power to launch mode:
            launcherPower = launcherMotorLaunchPower;
        }

        // Set the launcher Motors:
        DcMotorUtil.setMotorsPower(this.launcherMotors, launcherPower);

        return launcherPower;
    }


    public void idleMotors() {
        DcMotorUtil.setMotorsPower(this.leftMotors, 0);
        DcMotorUtil.setMotorsPower(this.rightMotors, 0);
    }

    public void resetEncoders() throws InterruptedException {
        // Reset Encoders:
        DcMotorUtil.setMotorsRunMode(this.leftMotors, DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        DcMotorUtil.setMotorsRunMode(this.rightMotors, DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Thread.sleep(50);  // Wait for reset to occur...
        // Change mode to run using the encoders:
        DcMotorUtil.setMotorsRunMode(this.leftMotors, DcMotor.RunMode.RUN_USING_ENCODER);
        DcMotorUtil.setMotorsRunMode(this.rightMotors, DcMotor.RunMode.RUN_USING_ENCODER);
    }

    private Boolean __runBooleanCallableIgnoreException(Callable<Boolean> func) {
        try {
            return func.call();
        } catch (Exception e) {
            return null;
        }
    }

    public static String AUTO_FULL_LOG_TAG = "Comp Bot Auto Full";
    public static String AUTO_STATE_LOG_TAG = "Comp Bot Auto State";

    private static long MAX_TIME_TIMEOUT = 200; // MAX time until TIMEOUT when running OpMode (millis)
    private static double NO_BEACON_ROTATE_SPEED = 0.25;
    private static double MINIMUM_ROTATION_DIFF = AngleUnit.RADIANS.fromUnit(AngleUnit.DEGREES, 3);
    private static long MINIMUM_ENCODER_DRIVE_VALUE = 800;
    private static long ENCODER_DRIVE_RAMP_DOWN_VALUE = 5000;
    private static double WALL_FOLLOW_FRONT_SPEED = 0.4;
    private static double NOMINAL_DISTANCE = 19;
    private static double MAXIMUM_VALUE_DIFF = 50;
    private double MIN_LINE_REFLECT_AMT = 0.3; // TODO: UPDATE THIS VALUE
    private double FEEDER_POWER = 0.8;
    private long BALL_DISPENCE_DELAY = 2500;
    private double FEED_DETECTOR_BALL_VALUE = 0.03;


    public void noTargetSearchRotate() throws InterruptedException {
        // Log data and add to telemetry for debug:
        telemetry.addData("ERROR", "Target has not been seen in " + MAX_TIME_TIMEOUT + "ms");
        Log.d("Vuforia Data", "Lost beacon (" + MAX_TIME_TIMEOUT + "ms). Rotating...");

        // Set motors to rotate:
        DcMotorUtil.setMotorsPower(this.leftMotors, NO_BEACON_ROTATE_SPEED);
        DcMotorUtil.setMotorsPower(this.rightMotors, -NO_BEACON_ROTATE_SPEED);

        // Sleep to allow for rotation:
        Thread.sleep(200);

        // Stop motors:
        DcMotorUtil.setMotorsPower(this.leftMotors, 0);
        DcMotorUtil.setMotorsPower(this.rightMotors, 0);
        Log.d("Vuforia Data", "Ended Rotation");

        // Wait to allow camera to adjust and acquire a lock on a target
        Thread.sleep(500);
    }

    /*------------------------------------AUTONOMOUS SECTIONS-------------------------------------*/

    public double __calculateEncoderDriveMotorGain(long encoderChange) {
        double motorGain = 1 * Math.signum(encoderChange);
        if (encoderChange < ENCODER_DRIVE_RAMP_DOWN_VALUE) {
            Log.d(AUTO_FULL_LOG_TAG, "Encoder Ramp Down!");
            motorGain = motorGain * Math.abs(encoderChange) / ENCODER_DRIVE_RAMP_DOWN_VALUE;
        }

        return motorGain;
    }

    public void driveForwardByEncoder(double motorPower, double differentialGain, long encoderVal) throws InterruptedException {
        resetEncoders();

        // Log the info:
        Log.i(AUTO_FULL_LOG_TAG, "RESET LEFT/RIGHT MOTOR ENCODERS");
        Log.i(AUTO_FULL_LOG_TAG, "Left Encoders Average: " + DcMotorUtil.getMotorsPosition(this.leftMotors));
        Log.i(AUTO_FULL_LOG_TAG, "Right Encoders Average: " + DcMotorUtil.getMotorsPosition(this.rightMotors));

        while (__runBooleanCallableIgnoreException(getIsRunning)) {
            // Get encoder values:
            Long leftEncoder = DcMotorUtil.getMotorsPosition(this.leftMotors);
            Long rightEncoder = DcMotorUtil.getMotorsPosition(this.rightMotors);

            // Check if encoder value was returned:
            if (leftEncoder != null && rightEncoder != null) {
                telemetry.addData("Left Encoders", leftEncoder);
                Log.d(AUTO_FULL_LOG_TAG, "Left Encoders Average: " + leftEncoder);
                telemetry.addData("Right Encoders", rightEncoder);
                Log.d(AUTO_FULL_LOG_TAG, "Right Encoders Average: " + rightEncoder);

                long leftChange = encoderVal - leftEncoder;
                long rightChange = encoderVal - rightEncoder;

                telemetry.addData("Left Encoders Change", leftChange);
                Log.d(AUTO_FULL_LOG_TAG, "Left Encoders Change: " + leftChange);
                telemetry.addData("Right Encoders Change", rightChange);
                Log.d(AUTO_FULL_LOG_TAG, "Right Encoders Change: " + rightChange);

                // Break if both sides at final position:
                if (Math.abs(leftChange) < MINIMUM_ENCODER_DRIVE_VALUE
                        && Math.abs(rightChange) < MINIMUM_ENCODER_DRIVE_VALUE) {
                    Log.d(AUTO_FULL_LOG_TAG, "ENCODERS AT FINAL POSITION!");
                    break;
                }

                double leftMotorGain = __calculateEncoderDriveMotorGain(leftChange);
                double rightMotorGain = __calculateEncoderDriveMotorGain(rightChange);

                // Drive motors by designated motor power:
                DcMotorUtil.setMotorsPower(this.leftMotors, differentialGain * motorPower * leftMotorGain);
                DcMotorUtil.setMotorsPower(this.rightMotors, (1/differentialGain) * motorPower * rightMotorGain);
            } else {  // This means that there was no encoder data:
                Log.w(AUTO_FULL_LOG_TAG, "ENCODER DRIVE: NO ENCODER DATA!");
                break;
            }

            telemetry.update();
        }

        // Stop motors:
        idleMotors();
    }


    public Double driveToPosition(VectorF targetLocation, double motorGain) throws InterruptedException {
        /*---------------------------------DRIVE TO INITIAL POINT---------------------------------*/
        // Translation Navigation Setup:
        TranslationMotorNavigation translationNavigation = new TranslationMotorNavigation();
        translationNavigation.ROTATION_IGNORE_DISTANCE = 60;
        translationNavigation.MIN_DRIVE_DISTANCE = 40;
        translationNavigation.DEFAULT_ROTATION_GAIN = 0.8;
        translationNavigation.IGNORED_ROTATION_GAIN = 0.3;

        long time = System.currentTimeMillis();
        while (__runBooleanCallableIgnoreException(getIsRunning)) {
            // Set 'Phase' in the telemetry:
            telemetry.addData("Phase", "Drive to Initial Position");

            // Read and get robot location data using vuforia:
            vuforia.readData();
            VectorF lastLocation = vuforia.lastLocationXYZ;
            Orientation lastRotation = vuforia.lastRotationXYZ;

            // If robot location is known:
            if (lastLocation != null && lastRotation != null) {
                time = System.currentTimeMillis();

                // Get specific values from robot location data:
                float x = lastLocation.get(0);
                float y = lastLocation.get(1);
                float robotRot = lastRotation.thirdAngle;

                // Log data for debug:
                Log.d(AUTO_FULL_LOG_TAG, String.format("X: %.3f, Y: %.3f, Rot: " + lastRotation.toString(), x, y));

                // Calculate translation required to get to target:
                VectorF translation = targetLocation.subtracted(lastLocation);

                // Calculate motor power for navigation to target:
                TranslationMotorNavigation.NavigationData calculationData =
                        translationNavigation.calculateNavigationData(translation.get(0), translation.get(1), robotRot);

                // Break if on target (and return robot rotation):
                if (calculationData.onTarget) {
                    idleMotors();
                    return (double) robotRot;
                }

                calculationData.leftMotorPower = calculationData.leftMotorPower * motorGain;
                calculationData.rightMotorPower = calculationData.rightMotorPower * motorGain;

                // Add data to telemetry for debug:
                telemetry.addData("On Target", calculationData.onTarget);
                telemetry.addData("Left Power", calculationData.leftMotorPower);
                telemetry.addData("Right Power", calculationData.rightMotorPower);
                telemetry.addData("Needed Distance", calculationData.translationDistance);
                telemetry.addData("Needed Angle", calculationData.translationAngle);
                telemetry.addData("Robot Angle", robotRot);
                telemetry.addData("Needed Angle Change", calculationData.rotationAmount);
                telemetry.addData("Forward Power", calculationData.forwardPower);
                telemetry.addData("Rotation Power", calculationData.rotationPower);
                telemetry.addData("Robot X", x);
                telemetry.addData("Robot Y", y);

                // Drive Motors:
                DcMotorUtil.setMotorsPower(this.leftMotors, calculationData.leftMotorPower);
                DcMotorUtil.setMotorsPower(this.rightMotors, calculationData.rightMotorPower);
            }

            // If target are not seen after some amount of time, rotate to find one:
            if (System.currentTimeMillis() - time > MAX_TIME_TIMEOUT) {
                noTargetSearchRotate();
            }

            // Update telemetry:
            telemetry.update();

            // Sleep to control loop:
            Thread.sleep(50);
        }

        idleMotors();

        // Return null if interrupted:
        return null;
    }


    public boolean robotRotation(double rotationAngle, double rotationGain) {
        /**
         * Drive the motors to rotate the robot based on a target rotation angle and a rotation gain.
         *
         * Returns: (boolean) If robot is on the target or not.
         */

        // Calculate rotation power to be applied to motors:
        double rotationPower = (rotationAngle / Math.PI);
        rotationPower = (rotationPower * rotationGain);
        telemetry.addData("rotationPower", rotationPower);

        // Break if rotation angle is smaller than minimum rotation difference
        //  (on target rotation):
        if (Math.abs(rotationAngle) < MINIMUM_ROTATION_DIFF) {
            Log.d(AUTO_FULL_LOG_TAG, "Rotation Angle: " + rotationAngle + " < Minimum Rot: " + MINIMUM_ROTATION_DIFF);
            return true;
        } else {
            Log.d(AUTO_FULL_LOG_TAG, "Rotation Angle: " + rotationAngle + " > Minimum Rot: " + MINIMUM_ROTATION_DIFF);
        }

        // Calculate left motors power and set motors:
        double leftPower = Range.clip(rotationPower, -1, 1);
        telemetry.addData("Left Motor Power", leftPower);
        DcMotorUtil.setMotorsPower(this.leftMotors, leftPower);

        // Calculate right motors power and set motors:
        double rightPower = -Range.clip(rotationPower, -1, 1);
        telemetry.addData("Right Motor Power", rightPower);
        DcMotorUtil.setMotorsPower(this.rightMotors, rightPower);

        return false;
    }


    public Double rotateRobotVision(double directAngle, double rotationGain) throws InterruptedException {
        // Initiate variable for no target timeout:
        long timeSinceLastData = System.currentTimeMillis();

        while (__runBooleanCallableIgnoreException(getIsRunning)) {
            // Read vuforia data:
            vuforia.readData();
            Orientation lastRotation = vuforia.lastRotationXYZ;

            if (lastRotation != null) {
                // Update last target read time:
                timeSinceLastData = System.currentTimeMillis();

                // Get the robot rotation from Orientation object
                double robotRotation = lastRotation.thirdAngle;

                // Calculate the needed angle of rotation to get to target:
                double rotationAngle = TranslationMotorNavigation.angleDifference(robotRotation,
                        directAngle);
                telemetry.addData("rotationAmount", rotationAngle);

                // Rotate robot using rotation angle and gain:
                boolean finishedRotation = robotRotation(rotationAngle, rotationGain);
                if (finishedRotation) {  // Break from loop if the rotation has been ended:
                    idleMotors();
                    return robotRotation;
                }
            } if (System.currentTimeMillis() - timeSinceLastData > MAX_TIME_TIMEOUT) {
                // Run a target search if no targets are in sight:
                noTargetSearchRotate();
            }

            // Update telemetry:
            telemetry.update();
        }

        idleMotors();
        return null;
    }


    public Orientation getIMUOrientation() {
        /**
         * Get the IMU orientation with the designated settings.
         */
        return this.imu.getAngularOrientation().toAngleUnit(AngleUnit.RADIANS)
                .toAxesReference(AxesReference.INTRINSIC).toAxesOrder(AxesOrder.ZYX);
    }

    public void driveForwardByIMU(long encoderLimit, boolean lineDetect, double angle, double rotationGain, double forwardPower) throws InterruptedException {
        driveForwardByIMU(encoderLimit, lineDetect, angle, rotationGain, forwardPower, 0, 0);
    }

    public void driveForwardByIMU(long encoderLimit, boolean lineDetect, double angle, double rotationGain, double forwardPower,
                                  long encoderRampDistance, double rampPower) throws InterruptedException {
        /**
         * Rotate the robot by a certain degree angle using the IMU.
         */

        resetEncoders();

        if (encoderRampDistance != 0) {
            resetEncoders();
        }

        long lineDetectWaitTime = 1000;
        long startTime = System.currentTimeMillis();
        double _forwardPower = forwardPower;

        // Record initial angle:
        Orientation initialAngle = getIMUOrientation();
        // Get robot heading:
        double initialRotation = initialAngle.firstAngle;
        // Add rotation to initial angle and make sure that it is a -180 - 180 value:
        double targetAngle = TranslationMotorNavigation.angleDifference(angle + initialRotation, 0);
        Log.d(AUTO_FULL_LOG_TAG, "angle: " + angle);
        Log.d(AUTO_FULL_LOG_TAG, "targetAngle: " + targetAngle);

        // Loop while OpMode is active
        while (__runBooleanCallableIgnoreException(getIsRunning)) {
            Long encoderAverage = (DcMotorUtil.getMotorsPosition(this.leftMotors) +
                    DcMotorUtil.getMotorsPosition(this.rightMotors))/2;

            if (encoderAverage != 0) {
                if (Math.abs(encoderAverage) > Math.abs(encoderLimit)) {
                    break;
                }
            }

            if (encoderRampDistance != 0) {
                if (Math.abs(encoderAverage) > encoderRampDistance) {
                    _forwardPower = rampPower;
                } else {
                    _forwardPower = forwardPower;
                }
            }

            telemetry.addData("Phase", "Drive forward with angle correction");
            // Get current robot angle:
            double robotAngle = getIMUOrientation().firstAngle;
            Log.d(AUTO_FULL_LOG_TAG, "robotAngle: " + robotAngle);

            // Calculate the needed angle of rotation to get to target:
            double rotationAngle = TranslationMotorNavigation.angleDifference(robotAngle, targetAngle);
            telemetry.addData("rotationAmount", rotationAngle);
            Log.d(AUTO_FULL_LOG_TAG, "rotationAmount: " + rotationAngle);
            // Calculate rotation power to be applied to motors:
            double rotationPower = (rotationAngle / Math.PI);
            rotationPower = (rotationPower * rotationGain);
            telemetry.addData("rotationPower", rotationPower);
            Log.d(AUTO_FULL_LOG_TAG, "rotationPower: " + rotationPower);

            // Calculate left motors power and set motors:
            double leftPower = _forwardPower + Range.clip(rotationPower, -1, 1);
            telemetry.addData("Left Motor Power", leftPower);
            Log.d(AUTO_FULL_LOG_TAG, "Left Motor Power: " + leftPower);
            DcMotorUtil.setMotorsPower(this.leftMotors, leftPower);

            // Calculate right motors power and set motors:
            double rightPower = _forwardPower - Range.clip(rotationPower, -1, 1);
            telemetry.addData("Right Motor Power", rightPower);
            Log.d(AUTO_FULL_LOG_TAG, "Right Motor Power: " + rightPower);
            DcMotorUtil.setMotorsPower(this.rightMotors, rightPower);


            if (lineDetect) {
                // Check if on beacon line:
                double groundReflect = this.groundReflectSensor.getLightDetected();
                telemetry.addData("Ground Reflect", groundReflect);

                if (groundReflect > this.MIN_LINE_REFLECT_AMT
                        && (System.currentTimeMillis() - startTime) > lineDetectWaitTime) {
                    Log.d(AUTO_FULL_LOG_TAG, "Ground Reflect: " + groundReflect + " > " + this.MIN_LINE_REFLECT_AMT);
                    idleMotors();
                    break;
                }
            }

            telemetry.update();
        }

        idleMotors();
    }

    public void rotateRobotIMU(double angle, double rotationGain) {
        /**
         * Rotate the robot by a certain degree angle using the IMU.
         */
        // Record initial angle:
        Orientation initialAngle = getIMUOrientation();
        // Get robot heading:
        double initialRotation = initialAngle.firstAngle;
        // Add rotation to initial angle and make sure that it is a -180 - 180 value:
        double targetAngle = TranslationMotorNavigation.angleDifference(angle + initialRotation, 0);
        Log.d(AUTO_FULL_LOG_TAG, "angle: " + angle);
        Log.d(AUTO_FULL_LOG_TAG, "targetAngle: " + targetAngle);

        // Loop while OpMode is active
        while (__runBooleanCallableIgnoreException(getIsRunning)) {
            telemetry.addData("Phase", "Rotate to Angle");
            // Get current robot angle:
            double robotAngle = getIMUOrientation().firstAngle;
            Log.d(AUTO_FULL_LOG_TAG, "robotAngle: " + robotAngle);

            // Calculate the needed angle of rotation to get to target:
            double rotationAngle = TranslationMotorNavigation.angleDifference(robotAngle, targetAngle);
            telemetry.addData("rotationAmount", rotationAngle);
            Log.d(AUTO_FULL_LOG_TAG, "rotationAmount: " + rotationAngle);
            // Calculate rotation power to be applied to motors:
            double rotationPower = (rotationAngle / Math.PI);
            rotationPower = (rotationPower * rotationGain);
            telemetry.addData("rotationPower", rotationPower);
            Log.d(AUTO_FULL_LOG_TAG, "rotationPower: " + rotationPower);

            // Break if rotation angle is smaller than minimum rotation difference
            //  (on target rotation):
            if (Math.abs(rotationAngle) < MINIMUM_ROTATION_DIFF) {
                Log.d(AUTO_FULL_LOG_TAG, "Rotation Angle: " + rotationAngle + " < Minimum Rot: " + MINIMUM_ROTATION_DIFF);
                break;
            }

            // Calculate left motors power and set motors:
            double leftPower = Range.clip(rotationPower, -1, 1);
            telemetry.addData("Left Motor Power", leftPower);
            Log.d(AUTO_FULL_LOG_TAG, "Left Motor Power: " + leftPower);
            DcMotorUtil.setMotorsPower(this.leftMotors, leftPower);

            // Calculate right motors power and set motors:
            double rightPower = -Range.clip(rotationPower, -1, 1);
            telemetry.addData("Right Motor Power", rightPower);
            Log.d(AUTO_FULL_LOG_TAG, "Right Motor Power: " + rightPower);
            DcMotorUtil.setMotorsPower(this.rightMotors, rightPower);

            // Update telemetry:
            telemetry.update();
        }

        idleMotors();
    }


    public void followWall(List<DcMotor> closeMotors, List<DcMotor> fartherMotors, UltrasonicSensor sideUltrasonicSensor) { // TODO: IMPLEMENT 'maxEncoderTurns'
        /*--------------------------------------FOLLOW THE WALL-----------------------------------*/
        ConstantDistanceMotorNavigation constantDistanceNavigation = new ConstantDistanceMotorNavigation(NOMINAL_DISTANCE, MAXIMUM_VALUE_DIFF);

        long lineDetectWaitTime = 1000;
        long startTime = System.currentTimeMillis();

        while (__runBooleanCallableIgnoreException(getIsRunning)) {
            telemetry.addData("Phase", "Following Wall");

            double sideUltrasonicLevel = sideUltrasonicSensor.getUltrasonicLevel();
            telemetry.addData("Side Ultrasonic Value", sideUltrasonicLevel);

            // Get motor power:
            ConstantDistanceMotorNavigation.NavigationData navigationData =
                    constantDistanceNavigation.calculateNavigationData(WALL_FOLLOW_FRONT_SPEED, sideUltrasonicLevel);
            // Set motors power:
            DcMotorUtil.setMotorsPower(closeMotors, navigationData.closerMotorPower);
            DcMotorUtil.setMotorsPower(fartherMotors, navigationData.fartherMotorPower);

            // Add motors powers to telemetry:
            telemetry.addData("Closer Motor Power", navigationData.closerMotorPower);
            telemetry.addData("Farther Motor Power", navigationData.fartherMotorPower);

            telemetry.addData("Rotation Power", navigationData.rotationPower);

            // Check if on beacon line:
            double groundReflect = this.groundReflectSensor.getLightDetected();
            telemetry.addData("Ground Reflect", groundReflect);

            // Update telemetry:
            telemetry.update();

            if (groundReflect > this.MIN_LINE_REFLECT_AMT
                    && (System.currentTimeMillis()-startTime) > lineDetectWaitTime) {
                Log.d(AUTO_FULL_LOG_TAG, "Ground Reflect: " + groundReflect + " > " + this.MIN_LINE_REFLECT_AMT);
                idleMotors();
                break;
            }
        }
    }

    private void __logColorSensorValue(String colorSensorName, String valueName, int value) {
        Log.d(AUTO_FULL_LOG_TAG, colorSensorName + " Color Sensor " + valueName + ": " + value);
    }


    // TODO: PLEASE FIND BETTER WAY (more fool-proof):
    private int __calculateColorSensorDisparity(int[] DESIRED_BEACON_COLOR, ColorSensor colorSensor, String colorSensorName) {
        int disparity = 0;
        int red = colorSensor.red();
        __logColorSensorValue(colorSensorName, "red", red);
        int blue = colorSensor.blue();
        __logColorSensorValue(colorSensorName, "blue", blue);

        if (DESIRED_BEACON_COLOR[0] > 0) {  // Wanted red
            if (red <= blue) {
                disparity += 10;
            }
        } else if (DESIRED_BEACON_COLOR[2] > 0) {  // Wanted blue
            if (blue <= red) {
                disparity += 10;
            }
        }

        Log.d(AUTO_FULL_LOG_TAG, colorSensorName + " Color Sensor Disparity: " + disparity);
        return disparity;
    }

    private void __pushBeaconAndWait(CompetitionBotConfig.AdvancedServo advancedServo)
            throws InterruptedException {
        advancedServo.activatePusher(true);
        Thread.sleep(1000);
        advancedServo.activatePusher(false);
    }

    // Color Detection / Beacon Push:
    public void pushBeacon(int[] DESIRED_BEACON_COLOR) throws InterruptedException {
        // Calculate the disparity between the color sensor disparity:
        ///  NOTE: Negative means desired color is more likely on the left, positive is the opposite
        int disparityDisparity = __calculateColorSensorDisparity(DESIRED_BEACON_COLOR, this.colorLeft, "Left")
                - __calculateColorSensorDisparity(DESIRED_BEACON_COLOR, this.colorRight, "Right");

        Log.d(AUTO_STATE_LOG_TAG, "Disparity Disparity: " + disparityDisparity);
        if (Integer.signum(disparityDisparity) == -1) {
            Log.d(AUTO_STATE_LOG_TAG, "Desired Color on Left");
            __pushBeaconAndWait(this.pusherLeft);
        } else if (Integer.signum(disparityDisparity) == 1){
            Log.d(AUTO_STATE_LOG_TAG, "Desired Color on Right");
            __pushBeaconAndWait(this.pusherRight);
        } else {
            Log.d(AUTO_STATE_LOG_TAG, "Equal Desired Color -- Skipping!");
        }
    }

    public void dispenceBalls(int ballQuantity) {
        boolean feederNotDisabled = true;
        double feederSpeed;

        long initialTime = System.currentTimeMillis();
        while (__runBooleanCallableIgnoreException(getIsRunning) && ballQuantity > 0) {
            if ((System.currentTimeMillis()-initialTime) > BALL_DISPENCE_DELAY && !feederNotDisabled) {
                feederNotDisabled = true;
            }

            if (feederNotDisabled){
                double feedDetectorValue = this.feedDetector.getLightDetected();

                //  Default feeder speed:
                feederSpeed = 0;
                //  Disable the feeder, if the feed detector is above the desired value:
                if (feedDetectorValue > FEED_DETECTOR_BALL_VALUE) {
                    initialTime = System.currentTimeMillis();
                    feederNotDisabled = false;
                    ballQuantity--;
                } else {
                    feederSpeed = FEEDER_POWER;
                }
                //  Set the feeder speed:
                this.feederServo.setPower(feederSpeed);
            }
        }
    }
}
