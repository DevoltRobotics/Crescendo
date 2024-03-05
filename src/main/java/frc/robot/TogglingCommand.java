package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;

public class TogglingCommand extends Command {
    
    Command a;
    Command b;

    boolean state = true;

    public TogglingCommand(Command a, Command b) {
        this.a = a;
        this.b = b;
    }

    @Override
    public void initialize() {
        state = !state;
    }

    @Override
    public void execute() {
        if(state) {
            if(b.isScheduled()) {
                b.cancel();
            }
            a.schedule();
        } else {
            if(a.isScheduled()) {
                a.cancel();
            }
            
            b.schedule();
        }

        cancel();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
