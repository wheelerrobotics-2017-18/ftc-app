package org.wheelerschool.robotics.competitionbot;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.wheelerschool.robotics.competitionbot.library.CompetitionBot;
import org.wheelerschool.robotics.library.util.joystick.JoystickButtonUpdated;

import java.util.concurrent.Callable;

/**
 * Created by luciengaitskell on 12/20/17.
 */

@TeleOp(name = "Competition TeleOp")
public class CompetitionTeleOp extends OpMode {
    CompetitionBot cb;

    JoystickButtonUpdated glyphRaiseButton;

    @Override
    public void init() {
        cb = new CompetitionBot(hardwareMap);
        glyphRaiseButton = new JoystickButtonUpdated(new Callable<Boolean>() {
            @Override
            public Boolean call() throws Exception {
                return gamepad1.a;
            }
        });
    }

    @Override
    public void loop() {
        cb.driveMotors.updateMotors(gamepad1.left_stick_x, -gamepad1.left_stick_y,
                gamepad1.right_stick_x);
        telemetry.addData("Front Left", cb.driveMotors.fLeft.getCurrentPosition());
        telemetry.addData("Front Right", cb.driveMotors.fRight.getCurrentPosition());
        telemetry.addData("Back Left", cb.driveMotors.bLeft.getCurrentPosition());
        telemetry.addData("Back Right", cb.driveMotors.bRight.getCurrentPosition());

        cb.setGlyphRaiseState(glyphRaiseButton.getValueIgnoreException().flipStateValue);
        cb.setGlyphIntakeState(gamepad1.right_bumper);
    }
}
