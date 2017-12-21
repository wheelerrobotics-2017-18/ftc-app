package org.wheelerschool.robotics.competitionbot;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.wheelerschool.robotics.competitionbot.library.CompetitionBot;

import java.util.Arrays;

/**
 * Created by luciengaitskell on 12/20/17.
 */

@TeleOp()
public class DriveForward extends OpMode {
    CompetitionBot cb;

    @Override
    public void init() {
        cb = new CompetitionBot(hardwareMap);
    }

    @Override
    public void start() {
        //cb.driveMotors.updateMotors(1.f, 0, 0);
    }

    @Override
    public void loop() {
        telemetry.addData("m", Arrays.toString(cb.driveMotors.updateMotors(0, -0.5f, 0)));
        /*cb.driveMotors.fLeft.setPower(0.1f);
        cb.driveMotors.bLeft.setPower(0.1f);
        cb.driveMotors.fRight.setPower(0.1f);
        cb.driveMotors.bRight.setPower(0.1f);*/
    }

    @Override
    public void stop() {
        //cb.driveMotors.updateMotors(0, 0, 0);
    }
}
