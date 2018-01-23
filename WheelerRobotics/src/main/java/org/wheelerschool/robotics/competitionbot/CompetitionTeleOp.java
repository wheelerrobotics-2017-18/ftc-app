package org.wheelerschool.robotics.competitionbot;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.wheelerschool.robotics.competitionbot.library.CompetitionBot;
import org.wheelerschool.robotics.library.util.joystick.JoystickButtonUpdated;

import java.util.Arrays;
import java.util.concurrent.Callable;

/**
 * Created by luciengaitskell on 12/20/17.
 */

@TeleOp(name = "Competition TeleOp")
public class CompetitionTeleOp extends OpMode {
    CompetitionBot cb;

    JoystickButtonUpdated glyphRaiseButton;

    JoystickButtonUpdated glyphtUp;
    JoystickButtonUpdated glyphtDown;

    JoystickButtonUpdated relicOut;
    JoystickButtonUpdated relicIn;

    JoystickButtonUpdated relicWrist;
    JoystickButtonUpdated relicGrab;

    JoystickButtonUpdated glyphGrab;

    @Override
    public void init() {
        cb = new CompetitionBot(hardwareMap);
        glyphRaiseButton = new JoystickButtonUpdated(new Callable<Boolean>() {
            @Override
            public Boolean call() throws Exception {
                return gamepad1.b;
            }
        });
        glyphGrab = new JoystickButtonUpdated(new Callable<Boolean>() {
            @Override
            public Boolean call() throws Exception {
                return gamepad2.y;
            }
        });

        glyphtUp = new JoystickButtonUpdated(new Callable<Boolean>() {
            @Override
            public Boolean call() throws Exception {
                return gamepad2.dpad_up;
            }
        });
        glyphtDown = new JoystickButtonUpdated(new Callable<Boolean>() {
            @Override
            public Boolean call() throws Exception {
                return gamepad2.dpad_down;
            }
        });

        relicIn = new JoystickButtonUpdated(new Callable<Boolean>() {
            @Override
            public Boolean call() throws Exception {
                return gamepad1.dpad_down;
            }
        });
        relicOut = new JoystickButtonUpdated(new Callable<Boolean>() {
            @Override
            public Boolean call() throws Exception {
                return gamepad1.dpad_up;
            }
        });

        relicWrist = new JoystickButtonUpdated(new Callable<Boolean>() {
            @Override
            public Boolean call() throws Exception {
                return gamepad1.left_bumper;
            }
        }, true);
        relicGrab = new JoystickButtonUpdated(new Callable<Boolean>() {
            @Override
            public Boolean call() throws Exception {
                return gamepad1.left_trigger>0.8f;
            }
        }, true);

    }

    private float scaleJoy(float j, float exp) {
        return (float) Math.copySign(Math.pow(Math.abs(j), exp), j);
    }

    @Override
    public void loop() {
        telemetry.addData("Drive Motor",
                Arrays.toString(cb.driveMotors.updateMotors(scaleJoy(gamepad1.left_stick_x, 1), scaleJoy(-gamepad1.left_stick_y,3),
                scaleJoy(gamepad1.right_stick_x, 3))));
        telemetry.addData("Front Left", cb.driveMotors.fLeft.getCurrentPosition());
        telemetry.addData("Front Right", cb.driveMotors.fRight.getCurrentPosition());
        telemetry.addData("Back Left", cb.driveMotors.bLeft.getCurrentPosition());
        telemetry.addData("Back Right", cb.driveMotors.bRight.getCurrentPosition());

        cb.setGlyphRaiseState(glyphRaiseButton.getValueIgnoreException().flipStateValue);
        if (gamepad1.right_bumper) {
            cb.setGlyphIntake(cb.glyphIntakePower);
        } else if (gamepad1.right_trigger > 0.5) {
            cb.setGlyphIntake(-cb.glyphIntakePower);
        } else {
            cb.setGlyphIntake(0);
        }


        telemetry.addData("Glypht Enc", cb.glyphtDrive.dcMotor.getCurrentPosition());
        if (!cb.glyphtDrive.manualOverride(-gamepad2.left_stick_y, gamepad2.x)) {
            JoystickButtonUpdated.JoystickButtonData upD = glyphtUp.getValueIgnoreException();
            if (upD.buttonState && upD.isButtonStateNew) {
                cb.glyphtDrive.moveRel(1, 1);
            }

            JoystickButtonUpdated.JoystickButtonData downD = glyphtDown.getValueIgnoreException();
            if (downD.buttonState && downD.isButtonStateNew) {
                cb.glyphtDrive.moveRel(-1, 1);
            }

            //telemetry.addData("Idx", cb.glyphtDrive.currentIdx);
            if (gamepad2.a) {
                cb.glyphtDrive.stop();
            }
        }

        if (false) {
            JoystickButtonUpdated.JoystickButtonData outD = relicOut.getValueIgnoreException();
            if (outD.buttonState && outD.isButtonStateNew) {
                cb.relicExtension.moveRel(1, 1);
            }

            JoystickButtonUpdated.JoystickButtonData inD = relicIn.getValueIgnoreException();
            if (inD.buttonState && inD.isButtonStateNew) {
                cb.relicExtension.moveRel(-1, 1);
            }
        } else {
            if (relicOut.getValueIgnoreException().buttonState) {
                cb.relicExtension.dcMotor.setPower(1);
            } else if (relicIn.getValueIgnoreException().buttonState) {
                cb.relicExtension.dcMotor.setPower(-1);
            } else {
                cb.relicExtension.dcMotor.setPower(0);
            }
        }
        telemetry.addData("Relic Pos", cb.relicExtension.dcMotor.getCurrentPosition());

        cb.glyphGrabber.setState(glyphGrab.getValueIgnoreException().flipStateValue);

        cb.relicWrist.setState(relicWrist.getValueIgnoreException().flipStateValue);
        cb.relicGrabber.setState(relicGrab.getValueIgnoreException().flipStateValue);
    }
}
