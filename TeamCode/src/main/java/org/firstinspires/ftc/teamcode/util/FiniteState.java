package org.firstinspires.ftc.teamcode.util;
/*use camera to grab stuff if needed
 * make flowchart
 */
import org.firstinspires.ftc.teamcode.util.MotionLibrary.util.Pose2D;

public class FiniteState {
    public enum robotState {
        start,
        intake,
        outtake,
        idle
    }

    enum armState {
        extended,
        retracted
    }
    robotState state = robotState.start;

    //TODO set to array
    public Pose2D runStuff() {
        switch(state) {
            case intake:
                return new Pose2D();
                break;
            case outtake:
                return new Pose2D();
                break;
        }
    }
}