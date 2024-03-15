package org.firstinspires.ftc.teamcode.CurrentSeason.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.roboctopi.cuttlefish.controller.Waypoint;
import com.roboctopi.cuttlefish.queue.DelayTask;
import com.roboctopi.cuttlefish.queue.PointTask;
import com.roboctopi.cuttlefish.queue.ServoTask;
import com.roboctopi.cuttlefish.queue.TaskQueue;
import com.roboctopi.cuttlefish.utils.Pose;

@Autonomous(name="Auto")
public class Auto extends AutoConfig {

    TaskQueue queue = new TaskQueue();

    public void onInit() {
        super.onInit();

        ptpController.setTranslational_PD_ctrlr(new PID( 
            0.02,  // Proportional
            0.0,   // Integral
            0.002, // Derivative
            0.0,   // Initial value (should be zero)
            1.0    // Maximum integral power (to prevent integral windup)
        ));
        ptpController.setRotational_PID_ctrlr(new PID(
            3,  // Proportional
            0.0,   // Integral
            0.2, // Derivative
            0.0,   // Initial value (should be zero)
            1.0    // Maximum integral power (to prevent integral windup)
        ));
        ptpController.getAntistallParams().setMovePowerAntistallThreshold(0.2); // Maxmimum translational power where the bot is still stalled
        ptpController.getAntistallParams().setRotatePowerAntistallThreshold(0.2); // Maxmimum rotation power where the bot is still stalled
        ptpController.getAntistallParams().setMoveSpeedAntistallThreshold(0.015);  // Maximum speed in m/s for the bot to be considered stalled
        ptpController.getAntistallParams().setRotateSpeedAntistallThreshold(0.3); // Maximum rotation speed in rad/s for the bot to be considered stalled
    }

    public void main()
    {
        super.main();

        // Go forward 1000mm
        queue.addTask(new PointTask(
                new Waypoint(
                        new Pose(1000,0,0),
                        0.5
                ),
                ptpController
        ));

        // Go sideways 1000mm and turn 0.5PI Radians (90 degrees)
        queue.addTask(new PointTask(
                new Waypoint(
                        new Pose(1000,1000,Math.PI/2),
                        0.5
                ),
                ptpController
        ));

        //Delay to make sure the servo has time to move
        queue.addTask(new DelayTask(400));

        //Drive back to the starting position
        queue.addTask(new PointTask(
                new Waypoint(
                        new Pose(0,0,0),
                        0.5
                ),
                ptpController
        ));

    }
    public void mainLoop()
    {
        super.mainLoop();
    }
}
