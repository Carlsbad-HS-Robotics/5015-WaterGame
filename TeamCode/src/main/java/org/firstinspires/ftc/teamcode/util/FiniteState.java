/*use camera to grab stuff if needed
 * make flowchart on
 */

import org.firstinspires.ftc.teamcode.util.MotionLibrary.util.Pose2D;

public class FiniteState {
    enum state {
        intake,
        outtake,
        idle
    }

    enum armState {
        extended,
        retracted
    }

    //TODO set to array
    pubic Pose2D runStuff() {
        if(state.intake) {
        return new Pose2D(1);
        }
        if(state.outtake) {
            return new Pose2D(2);
        }
    }

    
}