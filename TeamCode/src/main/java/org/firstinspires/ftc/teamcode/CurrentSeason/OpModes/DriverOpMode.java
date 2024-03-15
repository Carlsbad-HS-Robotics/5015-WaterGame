package org.firstinspires.ftc.teamcode.CurrentSeason.OpModes;

import org.firstinspires.ftc.teamcode.CurrentSeason.Config.OpModeConfig;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.roboctopi.cuttlefish.controller.MecanumController;
import com.roboctopi.cuttlefish.utils.Pose;

@TeleOp(name="Teleop")
public abstract class DriverOpMode extends OpModeConfig {
    MecanumController chassis;
    public void mainLoop(){
        super.mainLoop();
        chassis.setVec(new Pose(gamepad1.left_stick_x,-gamepad1.left_stick_y,-gamepad1.right_stick_x));
    }
}
