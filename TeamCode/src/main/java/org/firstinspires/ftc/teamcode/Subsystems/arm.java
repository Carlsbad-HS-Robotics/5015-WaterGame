//TODO write arm code using finite state machine and APID
import org.firstinspires.ftc.teamcode.util.FiniteState;
import com.roboctopi.cuttlefish.queue.ForkTask;
import com.roboctopi.cuttlefish.queue.*;

public class arm {
    
    FiniteState fsm;

    public void init() {
        fsm = new FiniteState();

        if(fms.armState == extended) {
            //TODO find how to use forktasks
            //queue.addTask(new ForkTask)
        }
    }
}