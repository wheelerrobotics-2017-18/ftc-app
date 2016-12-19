package org.wheelerschool.robotics.competitionbot.controltests;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import java.util.ArrayList;
import java.util.List;

/**
 * Created by luciengaitskell on 12/16/16.
 */

@TeleOp
public class CompetitionBotLauncherControl extends OpMode {
    private String LOG_TAG = "Comp Bot TeleOp";

    private double launcherMotorsGain = 0.6;

    private List<DcMotor> launcherMotors;

    @Override
    public void init() {
        launcherMotors = new ArrayList<>();
        launcherMotors.add(hardwareMap.dcMotor.get("launcherLeft"));
        launcherMotors.add(hardwareMap.dcMotor.get("launcherRight"));
    }

    @Override
    public void loop() {
        // Launcher Speed:
        double launcherSpeed = gamepad2.left_stick_y * launcherMotorsGain;
        for (DcMotor mtr : launcherMotors) {
            Log.d(LOG_TAG, "Launcher Speed: " + launcherSpeed);
            mtr.setPower(launcherSpeed);
        }
    }
}
