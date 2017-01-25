package org.wheelerschool.robotics.library.util;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import java.util.concurrent.Callable;

/**
 * Created by luciengaitskell on 1/25/17.
 */

public class LinearOpModeActiveCallable implements Callable<Boolean> {
    LinearOpMode linearOpMode;

    public LinearOpModeActiveCallable(LinearOpMode linearOpMode) {
        this.linearOpMode = linearOpMode;
    }

    @Override
    public Boolean call() {
        return this.linearOpMode.opModeIsActive();
    }
}
