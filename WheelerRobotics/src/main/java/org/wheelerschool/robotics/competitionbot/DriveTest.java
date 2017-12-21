package org.wheelerschool.robotics.competitionbot;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.wheelerschool.robotics.competitionbot.library.CompetitionBot;

/**
 * Created by luciengaitskell on 12/20/17.
 */

@TeleOp(name = "Comp Bot Drive Test")
public class DriveTest extends OpMode {
    CompetitionBot cb;

    @Override
    public void init() {
        cb = new CompetitionBot(hardwareMap);
    }

    @Override
    public void loop() {
        cb.driveMotors.updateMotors(gamepad1.left_stick_x, gamepad1.left_stick_y,
                gamepad1.right_stick_x);
    }
}
