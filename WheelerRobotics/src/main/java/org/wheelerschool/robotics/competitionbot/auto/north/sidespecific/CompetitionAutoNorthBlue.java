package org.wheelerschool.robotics.competitionbot.auto.north.sidespecific;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.wheelerschool.robotics.competitionbot.auto.north.CompetitionAutoNorth;

/**
 * Created by luciengaitskell on 2/2/18.
 */

@Autonomous
public class CompetitionAutoNorthBlue extends CompetitionAutoNorth {
    @Override
    public int getSideScale() {
        return -1;
    }
}
