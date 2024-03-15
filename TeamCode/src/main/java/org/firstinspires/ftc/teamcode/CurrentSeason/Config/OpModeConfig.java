package org.firstinspires.ftc.teamcode.CurrentSeason.Config;

import com.roboctopi.cuttlefishftcbridge.opmodeTypes.GamepadOpMode;
import com.roboctopi.cuttlefish.localizer.ThreeEncoderLocalizer;
import com.roboctopi.cuttlefishftcbridge.devices.CuttleEncoder;
import com.roboctopi.cuttlefishftcbridge.devices.CuttleRevHub;
import com.roboctopi.cuttlefishftcbridge.devices.CuttleMotor;
import com.roboctopi.cuttlefish.controller.MecanumController;
import com.roboctopi.cuttlefish.controller.PTPController;

import org.firstinspires.ftc.teamcode.util.FiniteState;

import com.roboctopi.cuttlefish.utils.Direction;

public class OpModeConfig extends GamepadOpMode {
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
    PTPController ptpController;
    MecanumController chassis;

    FiniteState fsm;

    @Override
    public void onInit()
    {
        leftFrontMotor  = ctrlHub.getMotor(3);
        rightFrontMotor = ctrlHub.getMotor(2);
        rightBackMotor  = expHub .getMotor(2);
        leftBackMotor   = expHub .getMotor(3);

        leftBackMotor .setDirection(Direction.REVERSE);
        leftFrontMotor.setDirection(Direction.REVERSE);

        leftEncoder  = expHub .getEncoder(3,720*4);
        sideEncoder  = ctrlHub.getEncoder(0,720*4);
        rightEncoder = ctrlHub.getEncoder(3,720*4);
        leftEncoder.setDirection(Direction.REVERSE);

        encoderLocalizer = new ThreeEncoderLocalizer(
                leftEncoder  ,
                sideEncoder  ,
                rightEncoder ,
                29, // Radius of the wheel in mm
                130.5, // Distance between the two forward facing wheels in mm
                1.0 //Calibration constant (see below)
        );

        chassis = new MecanumController(rightFrontMotor,rightBackMotor,leftFrontMotor,leftBackMotor);
        ptpController = new PTPController(chassis, encoderLocalizer);

        fsm = new FiniteState();
    }

    @Override
    public void main() {

    }

    @Override
    public void mainLoop()
    {
        if (gamepad1.a) {
            fsm.state = fsm.state.inatke;
        }
        //Make this move along this vector
        //fsm.runStuff();



        encoderLocalizer.update();
        //telemetry.addData("State: ", fsm.state);
        telemetry.addData("Localizer X: ",encoderLocalizer.getPos().getX());
        telemetry.addData("Localizer Y: ",encoderLocalizer.getPos().getY());
        telemetry.update();
    }
}
