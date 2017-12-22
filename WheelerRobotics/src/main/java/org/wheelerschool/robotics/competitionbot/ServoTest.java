package org.wheelerschool.robotics.competitionbot;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.wheelerschool.robotics.competitionbot.library.CompetitionBot;

/**
 * Created by Pratul on 12/22/2017.
 */
@TeleOp (name = "Comp bot Servo Test")
public class ServoTest extends OpMode {
    CompetitionBot cb;

    @Override
    public void init() {
        cb = new CompetitionBot(hardwareMap);
    }


}
