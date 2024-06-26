package org.firstinspires.ftc.teamcode.CurrentSeason.Config;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.linearOpMode;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.roboctopi.cuttlefishftcbridge.opmodeTypes.GamepadOpMode;
import com.roboctopi.cuttlefish.localizer.ThreeEncoderLocalizer;
import com.roboctopi.cuttlefishftcbridge.devices.CuttleEncoder;
import com.roboctopi.cuttlefishftcbridge.devices.CuttleRevHub;
import com.roboctopi.cuttlefishftcbridge.devices.CuttleMotor;
import com.roboctopi.cuttlefish.controller.MecanumController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.roboctopi.cuttlefish.controller.PTPController;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.roboctopi.cuttlefish.controller.Waypoint;
import com.roboctopi.cuttlefish.components.Motor;
import com.roboctopi.cuttlefish.queue.PointTask;
import com.roboctopi.cuttlefish.utils.Direction;
import com.roboctopi.cuttlefish.utils.Pose;

public class AutoConfig extends GamepadOpMode {
    CuttleRevHub ctrlHub = new CuttleRevHub(hardwareMap,CuttleRevHub.HubTypes.CONTROL_HUB);
    CuttleRevHub expHub = new CuttleRevHub(hardwareMap,CuttleRevHub.HubTypes.EXPANSION_HUB);
    CuttleEncoder leftEncoder ;
    CuttleEncoder sideEncoder ;
    CuttleEncoder rightEncoder;

    ThreeEncoderLocalizer encoderLocalizer;
    public CuttleMotor leftFrontMotor;
    public CuttleMotor rightFrontMotor;
    public CuttleMotor rightBackMotor ;
    public CuttleMotor leftBackMotor  ;
    MecanumController quinton;
    PTPController ptpController;

    @Override
    public void onInit()
    {
        leftFrontMotor  = ctrlHub.getMotor(3);
        rightFrontMotor = ctrlHub.getMotor(2);
        rightBackMotor  = expHub .getMotor(2);
        leftBackMotor   = expHub .getMotor(3);

        leftBackMotor .setDirection(Direction.REVERSE);
        leftFrontMotor.setDirection(Direction.REVERSE);


        leftEncoder.setDirection(Direction.REVERSE);

        encoderLocalizer = new ThreeEncoderLocalizer(
                leftEncoder  ,
                sideEncoder  ,
                rightEncoder ,
                29, // Radius of the wheel in mm
                130.5, // Distance between the two forward facing wheels in mm
                1.0 //Calibration constant (see below)
        );

        quinton = new MecanumController(rightFrontMotor,rightBackMotor,leftFrontMotor,leftBackMotor);
        ptpController = new PTPController(quinton, encoderLocalizer);
 }

    @Override
    public void main() {

    }

    @Override
    public void runOpMode() throws InterruptedException {

    }
    @Override
    public void mainLoop()
    {
        encoderLocalizer.update();
        System.out.println(encoderLocalizer.getPos());
        telemetry.addData("Localizer X:",encoderLocalizer.getPos().getX());
        telemetry.addData("Localizer Y:",encoderLocalizer.getPos().getY());
        telemetry.addData("Localizer R:",encoderLocalizer.getPos().getR());
        telemetry.update();
    }
}
