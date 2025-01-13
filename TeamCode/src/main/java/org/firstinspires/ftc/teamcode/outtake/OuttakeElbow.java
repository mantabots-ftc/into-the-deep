package org.firstinspires.ftc.teamcode.outtake;

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

/* Utils includes */
import org.firstinspires.ftc.teamcode.utils.SmartTimer;

public class OuttakeElbow {

    public enum Position {
        TRANSFER,
        DROP,
        OFF
    };

    private static final Map<String, Position> sConfToPosition = Map.of(
            "transfer", Position.TRANSFER,
            "drop",Position.DROP,
            "off",Position.OFF
    );

    private static int sTimeOut = 100;

    Telemetry               mLogger;

    boolean                 mReady;
    SmartTimer              mTimer;

    Position                mPosition;
    ServoComponent          mServo;
    Map<Position, Double>   mPositions   = new LinkedHashMap<>();

    public boolean isMoving() { return mTimer.isArmed();}

    public Position getPosition() { return mPosition; }

    public void setHW(Configuration config, HardwareMap hwm, Telemetry logger) {

        mLogger = logger;
        mReady = true;
        mTimer = new SmartTimer(mLogger);

        String status = "";

        // Get configuration
        ConfServo pitch  = config.getServo("outtake-elbow-pitch");
        if(pitch == null)  { mReady = false; status += " CONF";}
        else {

            // Configure servo
            if (pitch.shallMock()) { mServo = new ServoMock("outtake-elbow-pitch"); }
            else if (pitch.getHw().size() == 1) { mServo = new ServoSingle(pitch, hwm, "outtake-elbow-pitch", logger); }
            else if (pitch.getHw().size() == 2) { mServo = new ServoCoupled(pitch, hwm, "outtake-elbow-pitch", logger); }

            mPositions.clear();
            Map<String, Double> confPosition = pitch.getPositions();
            for (Map.Entry<String, Double> pos : confPosition.entrySet()) {
                if(sConfToPosition.containsKey(pos.getKey())) {
                    mPositions.put(sConfToPosition.get(pos.getKey()), pos.getValue());
                }
            }

            if (!mServo.isReady()) { mReady = false; status += " HW";}
        }

        // Log status
        if (mReady) { logger.addLine("==>  OUT ELB : OK"); }
        else        { logger.addLine("==>  OUT ELB : KO : " + status); }

        // Initialize position
        this.setPosition(Position.OFF);

    }

    public void setPosition(Position position) {

        if( mPositions.containsKey(position) && mReady && !this.isMoving()) {
            mServo.setPosition(mPositions.get(position));
            mPosition = position;
            mTimer.arm(sTimeOut);
        }
    }

    public void moveUp() {
        if(mPosition == Position.OFF)              { this.setPosition(Position.TRANSFER);          }
        else if(mPosition == Position.TRANSFER)    { this.setPosition(Position.DROP); }
    }

    public void moveDown() {
        if(mPosition == Position.DROP)          { this.setPosition(Position.TRANSFER);         }
        else if(mPosition == Position.TRANSFER) { this.setPosition(Position.OFF);          }
    }
}


