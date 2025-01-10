package org.firstinspires.ftc.teamcode.intake;

/* System includes */
import java.util.LinkedHashMap;
import java.util.Map;

/* Qualcomm includes */
import com.qualcomm.robotcore.hardware.HardwareMap;

/* FTC Controller includes */
import org.firstinspires.ftc.robotcore.external.Telemetry;

/* Configuration includes */
import org.firstinspires.ftc.teamcode.configurations.Configuration;
import org.firstinspires.ftc.teamcode.configurations.ConfServo;

/* Component includes */
import org.firstinspires.ftc.teamcode.components.ServoComponent;
import org.firstinspires.ftc.teamcode.components.ServoMock;
import org.firstinspires.ftc.teamcode.components.ServoCoupled;
import org.firstinspires.ftc.teamcode.components.ServoSingle;

public class IntakeClaw {

    public enum Position {
        OPEN,
        MICRORELEASED,
        CLOSED
    };

    private static final Map<String, Position> sConfToPosition = Map.of(
        "open",  Position.OPEN,
        "microrelease", Position.MICRORELEASED,
        "closed", Position.CLOSED
    );

    Telemetry             mLogger;

    boolean               mReady;

    Position              mPosition;
    ServoComponent        mServo;
    Map<Position, Double> mPositions = new LinkedHashMap<>();

    public Position getPosition() { return mPosition; }

    public void setHW(Configuration config, HardwareMap hwm, Telemetry logger) {

        mLogger = logger;
        mReady = true;

        String status = "";

        // Get configuration
        ConfServo move  = config.getServo("intake-claw");
        if(move == null)  { mReady = false; status += " CONF";}
        else {

            // Configure servo
            if (move.shallMock()) { mServo = new ServoMock("intake-claw"); }
            else if (move.getHw().size() == 1) { mServo = new ServoSingle(move, hwm, "intake-claw", logger); }
            else if (move.getHw().size() == 2) { mServo = new ServoCoupled(move, hwm, "intake-claw", logger); }

            mPositions.clear();
            Map<String, Double> confPosition = move.getPositions();
            for (Map.Entry<String, Double> pos : confPosition.entrySet()) {
                if(sConfToPosition.containsKey(pos.getKey())) {
                    mPositions.put(sConfToPosition.get(pos.getKey()), pos.getValue());
                }
            }

            if (!mServo.isReady()) { mReady = false; status += " HW";}
        }

        // Log status
        if (mReady) { logger.addLine("==>  IN CLW : OK"); }
        else        { logger.addLine("==>  IN CLW : KO : " + status); }

        // Initialize position
        this.setPosition(Position.OPEN);
    }

    public void setPosition(Position position) {

        if( mPositions.containsKey(position) && mReady) {
            mServo.setPosition(mPositions.get(position));
            mPosition = position;
        }

    }

    public void switchPosition() {
        if( mPosition == Position.OPEN) { this.setPosition(Position.CLOSED); }
        else                            { this.setPosition(Position.OPEN);   }
    }

}


