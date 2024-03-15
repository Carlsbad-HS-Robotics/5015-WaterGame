//TODO imports

@TeleOp(name="Teleop")
public abstract class TeleOp extends OpModeConfig {
    public void mainLoop(){
        super.mainLoop();
        chassis.setVec(new Pose(gamepad1.left_stick_x,-gamepad1.left_stick_y,-gamepad1.right_stick_x));
    }
}
