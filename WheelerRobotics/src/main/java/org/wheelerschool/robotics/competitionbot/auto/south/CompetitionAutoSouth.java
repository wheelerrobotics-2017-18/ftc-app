package org.wheelerschool.robotics.competitionbot.auto.south;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuMarkInstanceId;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.wheelerschool.robotics.competitionbot.library.CompetitionBot;
import org.wheelerschool.robotics.config.Config;

/**
 * Created by luciengaitskell on 1/11/18.
 */


public abstract class CompetitionAutoSouth extends LinearOpMode {
    CompetitionBot cb;

    VuforiaLocalizer vuforia;

    public abstract int getSideScale();

    final float ROT_POWER = 0.5f;

    final int CRYPTO_COLUMN_WIDTH = 1020;

    final int BALANCE_OFFSET = -100; //-150
    final int JEWEL_ENC = 300;
    final int BALANCE_REPOS_ENC = 250;
    final int BALANCE_EXIT_ROT_ENC = 3000;
    final int BALANCE_EXIT_FWD_ENC = 5000; //5200 -- w/ -150 Balance for RED GOOD


    final int GLYPH_FWD_ENC = 1200;
    final int GLYPH_CLEAR_ENC = -700;
    final int GLYPH_SND_FWD_ENC = 600;

    private void sleep(int time) {
        long ts = System.currentTimeMillis();
        while (time>(System.currentTimeMillis()-ts) && opModeIsActive()) {
            try {
                Thread.sleep(1);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }
    }
    private void allEncDrive(int encL, int encR, float power, boolean rel, int time) {
        leftMotorEncDrive(encL, power, rel);
        rightMotorEncDrive(encR, power, rel);

        sleep(time);
    }

    private static void motorEncDrive(DcMotor m, int enc, float power, boolean rel) {
        if (rel) {
            enc += m.getCurrentPosition();
        }
        m.setTargetPosition(enc);
        m.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        m.setPower(power);
    }

    private static void motorEncStop(DcMotor m) {
        m.setPower(0);
        m.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    private void allMotorEncStop() {
        motorEncStop(cb.driveMotors.bLeft);
        motorEncStop(cb.driveMotors.bRight);
        motorEncStop(cb.driveMotors.fLeft);
        motorEncStop(cb.driveMotors.fRight);
    }

    private void leftMotorEncDrive(int enc, float power, boolean rel) {
        motorEncDrive(cb.driveMotors.bLeft, enc, power, rel);
        motorEncDrive(cb.driveMotors.fLeft, enc, power, rel);
    }

    private void rightMotorEncDrive(int enc, float power, boolean rel) {
        motorEncDrive(cb.driveMotors.bRight, enc, power, rel);
        motorEncDrive(cb.driveMotors.fRight, enc, power, rel);
    }

    private void jewelMove() {
        cb.jewelWrist.setPosition(CompetitionBot.JewelWristPositions.EXT);

        long start = System.currentTimeMillis();

        double b = 0;
        while ((System.currentTimeMillis() - start < 1000) && opModeIsActive()) {
            // Positive is red, negative is blue
            b = (cb.jewelColor.red() - cb.jewelColor.blue()) / 10240.;

            telemetry.addData("balance", b);
            telemetry.update();
        }

        // TODO: FIX THIS
        //if (b < 0.000005) {
        //    b = 0;
        //}

        int sign = getSideScale();

        if (b > 0) {
            telemetry.addData("Level", "Red");
        } else if (b < 0) {
            sign = -sign;
            telemetry.addData("Level", "Blue");
        } else {
            sign = 0;
            telemetry.addData("Level", "IMPARTIAL");
        }

        telemetry.update();
        sleep(500);

        allEncDrive(JEWEL_ENC*sign, JEWEL_ENC*-sign, ROT_POWER, false, 1500);

        cb.jewelWrist.setPosition(CompetitionBot.JewelWristPositions.CLEAR);

        allEncDrive(0, 0, ROT_POWER, false, 1500);
    }

    private void preExitBalancingStone() {
        allEncDrive(BALANCE_REPOS_ENC,BALANCE_REPOS_ENC,ROT_POWER, true, 1200);

        allEncDrive(BALANCE_EXIT_ROT_ENC * getSideScale(), -BALANCE_EXIT_ROT_ENC * getSideScale(), ROT_POWER, true, 3000);
    }

    private void exitBalancingStone(int distance) {
        allEncDrive(distance, distance, ROT_POWER, true, 5000);


        allEncDrive(BALANCE_EXIT_ROT_ENC*getSideScale(), -BALANCE_EXIT_ROT_ENC*getSideScale(), ROT_POWER, true, 3000);
    }

    private void deposit() {
        allEncDrive(GLYPH_FWD_ENC, GLYPH_FWD_ENC, 1, true, 1000);

        cb.glyphGrabber.setState(true);
        sleep(500);

        allEncDrive(GLYPH_CLEAR_ENC, GLYPH_CLEAR_ENC, 1, true, 1000);

        // ENSURE:
        allEncDrive(GLYPH_SND_FWD_ENC, GLYPH_SND_FWD_ENC, 1f, true, 1000);
        allEncDrive(-GLYPH_SND_FWD_ENC, -GLYPH_SND_FWD_ENC, 1f, true, 1000);
        //allEncDrive(GLYPH_CLEAR_ENC, GLYPH_CLEAR_ENC, 1f, true, 1000);
    }

    @Override
    public void runOpMode() throws InterruptedException {
        cb = new CompetitionBot(hardwareMap);
        cb.glyphGrabber.setState(false);
        cb.glyphtDrive.moveToEnc(-10000, 0.5);

        VuforiaLocalizer.Parameters parameters;
        if (true) {
            int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
            parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        } else {
            parameters = new VuforiaLocalizer.Parameters();
        }

        parameters.vuforiaLicenseKey = Config.VUFORIA_KEY;

        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);

        VuforiaTrackables relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        VuforiaTrackable relicTemplate = relicTrackables.get(0);
        relicTemplate.setName("relicVuMarkTemplate"); // can help in debugging; otherwise not necessary

        relicTrackables.activate();





        // ------------------------- INIT STOP
        waitForStart();



        RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.UNKNOWN;
        for (int i=0; i<10; i++) {
            vuMark = RelicRecoveryVuMark.from(relicTemplate);
        }

        int mainDist = BALANCE_EXIT_FWD_ENC + BALANCE_OFFSET*getSideScale();

        if (vuMark == RelicRecoveryVuMark.CENTER) {
            telemetry.addData("VU", "CENTER");
        } else if (vuMark == RelicRecoveryVuMark.LEFT) {
            mainDist += CRYPTO_COLUMN_WIDTH*getSideScale();
            telemetry.addData("VU", "LEFT");
        } else if (vuMark == RelicRecoveryVuMark.RIGHT) {
            mainDist -= CRYPTO_COLUMN_WIDTH*getSideScale();
            telemetry.addData("VU", "RIGHT");
        } else {
            telemetry.addData("VU", "NONE");
        }

        telemetry.update();

        jewelMove();
        preExitBalancingStone();
        exitBalancingStone(mainDist);
        deposit();
    }
}