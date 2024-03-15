//TODO imports

public class TeleOp {
    @TeleOp(name="Driver Opmode")
    public abstract class DriverOpMode extends Tele {
        @Override
        public void onInit(){
            super.onInit();
        }

        @Override
        public void main() {
            super.main();
        }

        public void mainLoop(){
            super.mainLoop();
            chassis.setVec(new Pose(gamepad1.left_stick_x,-gamepad1.left_stick_y,-gamepad1.right_stick_x));
        }
    }
}