package org.firstinspires.ftc.teamcode.CurrentSeason.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.roboctopi.cuttlefish.controller.Waypoint;
import com.roboctopi.cuttlefish.queue.DelayTask;
import com.roboctopi.cuttlefish.queue.PointTask;
import com.roboctopi.cuttlefish.queue.ServoTask;
import com.roboctopi.cuttlefish.queue.TaskQueue;
import com.roboctopi.cuttlefish.utils.Pose;

@Autonomous(name="cuttle test")
public class AutoOp extends Auto{
    TaskQueue queue = new TaskQueue();
    public void onInit()
    {
        super.onInit();
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
