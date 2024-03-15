package org.firstinspires.ftc.teamcode.util;/*use camera to grab stuff if needed
 * make flowchart on
 */

import org.firstinspires.ftc.teamcode.util.MotionLibrary.util.Pose2D;

public class FiniteState {
    public enum robotState {
        intake,
        outtake,
        idle
    }

    enum armState {
        extended,
        retracted
    }

    //TODO set to array
    public Pose2D runStuff() {
        if(robotState == robotState.intake) {
            return new Pose2D(1);
        }
        if(robotState.outtake) {
            return new Pose2D(2);
        }
    }
}