package org.wheelerschool.robotics.competitionbot.auto.south.sidespecific;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.wheelerschool.robotics.competitionbot.auto.south.CompetitionAutoSouth;

/**
 * Created by luciengaitskell on 2/2/18.
 */

@Autonomous
public class CompetitionAutoSouthBlue extends CompetitionAutoSouth {
    @Override
    public int getSideScale() {
        return -1;
    }
}
