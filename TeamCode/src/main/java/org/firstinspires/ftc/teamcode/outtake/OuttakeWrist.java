package org.firstinspires.ftc.teamcode.outtake;

/* System includes */
import static java.lang.Math.max;
import static java.lang.Math.min;

import java.util.LinkedHashMap;
import java.util.Map;

/* Qualcomm includes */
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

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
import org.firstinspires.ftc.teamcode.intake.IntakeWrist;

public class OuttakeWrist {

    public enum Position {
        MINUS_TWO,
        MINUS_ONE,
        NULL,
        ONE,
        TWO,
        THREE,
        FOUR,
        FIVE,
        SIX
    };
    private static final Map<String, Position> sConfToPosition = Map.of(
            "-2", Position.MINUS_TWO,
            "-1", Position.MINUS_ONE,
            "0", Position.NULL,
            "1", Position.ONE,
            "2", Position.TWO,
            "3", Position.THREE,
            "4", Position.FOUR,
            "5", Position.FIVE,
            "6", Position.SIX
    );

    public static final double sIncrementRatio = 0.01;

    Telemetry             mLogger;

    boolean               mReady;

    Position              mPosition;
    double                mDeltaPosition = 0;
    ServoComponent        mServo;
    Map<Position, Double> mPositions = new LinkedHashMap<>();

    public Position getPosition() { return mPosition; }

    public void setHW(Configuration config, HardwareMap hwm, Telemetry logger) {

        mLogger = logger;
        mReady = true;

        String status = "";

        // Get configuration
        ConfServo roll  = config.getServo("outtake-wrist-roll");
        if(roll == null)  { mReady = false; status += " CONF";}
        else {

            // Configure servo
            if (roll.shallMock()) { mServo = new ServoMock("outtake-wrist-roll"); }
            else if (roll.getHw().size() == 1) { mServo = new ServoSingle(roll, hwm, "outtake-wrist-roll", logger); }
            else if (roll.getHw().size() == 2) { mServo = new ServoCoupled(roll, hwm, "outtake-wrist-roll", logger); }

            mPositions.clear();
            Map<String, Double> confPosition = roll.getPositions();
            for (Map.Entry<String, Double> pos : confPosition.entrySet()) {
                if(sConfToPosition.containsKey(pos.getKey())) {
                    mPositions.put(sConfToPosition.get(pos.getKey()), pos.getValue());
                }
            }

            if (!mServo.isReady()) { mReady = false; status += " HW";}
        }

        // Log status
        if (mReady) { logger.addLine("==>  OUT WRS : OK"); }
        else        { logger.addLine("==>  OUT WRS : KO : " + status); }

        // Initialize position
        this.setPosition(Position.NULL);
    }

    public void setPosition(Position position) {

        if( mPositions.containsKey(position) && mReady) {
            mLogger.addLine(" ==> OUT WRS POS : " + mPositions.get(position));
            mServo.setPosition(mPositions.get(position));
            mPosition = position;
            mDeltaPosition = 0;
        }
    }

    public void rotateUp() {
        if(mPosition      == Position.MINUS_TWO)    { this.setPosition(Position.MINUS_ONE); }
        else if(mPosition == Position.MINUS_ONE)    { this.setPosition(Position.NULL);      }
        else if(mPosition == Position.NULL)         { this.setPosition(Position.ONE);       }
        else if(mPosition == Position.ONE)          { this.setPosition(Position.TWO);       }
        else if(mPosition == Position.TWO)          { this.setPosition(Position.THREE);     }
        else if(mPosition == Position.THREE)        { this.setPosition(Position.FOUR);      }
        else if(mPosition == Position.FOUR)         { this.setPosition(Position.FIVE);      }
        else if(mPosition == Position.FIVE)         { this.setPosition(Position.SIX);       }
    }

    public void rotateDown() {
        if(mPosition      == Position.MINUS_ONE)    { this.setPosition(Position.MINUS_TWO);  }
        else if(mPosition == Position.NULL)         { this.setPosition(Position.MINUS_ONE);  }
        else if(mPosition == Position.ONE)          { this.setPosition(Position.NULL);       }
        else if(mPosition == Position.TWO)          { this.setPosition(Position.ONE);        }
        else if(mPosition == Position.THREE)        { this.setPosition(Position.TWO);        }
        else if(mPosition == Position.FOUR)         { this.setPosition(Position.THREE);      }
        else if(mPosition == Position.FIVE)         { this.setPosition(Position.FOUR);       }
        else if(mPosition == Position.SIX)          { this.setPosition(Position.FIVE);       }
    }
}

