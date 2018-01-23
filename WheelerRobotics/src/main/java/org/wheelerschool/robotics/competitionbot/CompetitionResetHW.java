package org.wheelerschool.robotics.competitionbot;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.wheelerschool.robotics.competitionbot.library.CompetitionBot;

/**
 * Created by luciengaitskell on 1/11/18.
 */

@TeleOp(name="RESET")
public class CompetitionResetHW extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        CompetitionBot cb = new CompetitionBot(hardwareMap);
        waitForStart();
        cb.resetDevices();
    }
}
