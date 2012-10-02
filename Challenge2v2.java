import lejos.nxt.*;
import java.io.*;
import lejos.robotics.RegulatedMotor;
import lejos.robotics.navigation.DifferentialPilot;
import lejos.util.Delay;
import lejos.nxt.comm.RConsole;
/*
 * Chanllenge2 with getCD method.
 * 
 * code is not perfect.There are a lot useless statement inside.
 * don't get confused , if that statement seems useless.
 */
public class Challenge2v2 {

    private final float wheelDiameter;
    private final float trackWidth;
    private final RegulatedMotor leftMotor;
    private final RegulatedMotor rightMotor;
    private final DifferentialPilot pilot;
    private final SensorPort ultrasonicPort;
    private final UltrasonicSensor ultrasonicSensor;
    private final SensorPort lightPort;
    private final LightSensor lightSensor;
    private final TouchSensor touchSensor;
    private final SensorPort touchPort;

    public static void main(String[] args) {

        final Challenge2v2 robot;
        robot = new Challenge2v2();

        robot.run();
    }

    // ///////////////constructor don't need to read/////////////////////
    public Challenge2v2() {
        ultrasonicPort = SensorPort.S2;
        ultrasonicSensor = new UltrasonicSensor(ultrasonicPort);
        lightPort = SensorPort.S3;
        lightSensor = new LightSensor(lightPort);
        lightSensor.setFloodlight(true);
        wheelDiameter = 56.0f;
        // 114 mm (measured between the centre of both wheels)
        trackWidth = 114.0f;
        leftMotor = Motor.A;
        rightMotor = Motor.C;
        pilot = new DifferentialPilot(wheelDiameter, trackWidth, leftMotor,
                rightMotor);
        touchPort = SensorPort.S1;
        touchSensor = new TouchSensor(touchPort);
    }

    // ////////////robot start to run from here//////////////////////////
    public void run() {
        boolean door = false;
        //getItBaby();
        pilot.setTravelSpeed(200);
        //pilot.rotate(90);
        findWall();// /find the nearest wall and go toward it then turn
        // right.

        /*
         * no problem about part , some things wrong below
         */
        /*
         * every time the robot finish one side of the wall ,then turn right
         * ,and repeat the while loop
         */

        while (!door) {
            /*
             * after the robot turn 90 degree, first check the wall color if it
             * is white
             */
            // >400 means white, <400 means black
            if (lightSensor.readNormalizedValue() > 400) {
                door = firstDoor();// this door checking is different from
                                   // others, see the methord
                if (door == true) {
                    break;// if found door then break the loop
                }
                pilot.forward();
                /*
                 * the next while loop check the distanse and color,untill the
                 * robot leave the door(find the black again) or too close to
                 * the wall
                 */
                while (ultrasonicSensor.getDistance() > 10
                        && lightSensor.readNormalizedValue() > 400) {
                }
            }

            /* after check the first door , check the whole wall */

            pilot.forward();

            // this while loop gonna repeat until the end of the wall//
            while (ultrasonicSensor.getDistance() > 10) {
                // if find the door,check it.
                if (lightSensor.readNormalizedValue() > 400) {
                    pilot.travel(155);
                    door = isDoor();
                    if (door == true) {
                        break;
                    }
                    pilot.forward();
                    while (ultrasonicSensor.getDistance() > 10
                            && lightSensor.readNormalizedValue() > 400) {
                    }
                }
            }
            // //////////////////////////////////////////////////////////

            if (door == true) {
                break;
            }

            pilot.stop();
            pilot.rotate(93);
        }

        pilot.travel(200);
    }

    // ////////check the door////////
    public boolean isDoor() {
        pilot.rotate(-90);
        pilot.travel(400);
        pilot.travel(-70);
        pilot.rotate(90);
        return false;
        /*
         * if (touchSensor.isPressed()) { pilot.travel(-70); pilot.rotate(90);
         * return false; } else { return true; }
         */

    }

    public void findWall() {
        pilot.setTravelSpeed(150);
        int ds;
        double[] angle = new double[10];
        int cl = 255;
        int j = 0;
        pilot.steer(-200, 360, true);
        while (pilot.isMoving()) {
            ds = ultrasonicSensor.getDistance();
            if (ds < cl) {
                cl = ds;
                j = 0;
                angle[j] = pilot.getAngleIncrement();
            }
            if (cl == ds) {
                angle[j] = pilot.getAngleIncrement();
                j++;
            }
        }
        pilot.steer(-200, angle[0] + (angle[j - 1] - angle[0]) / 2);
        pilot.forward();
        while (pilot.isMoving()) {
            ds = ultrasonicSensor.getDistance();
            if (ds <= 10) {
                pilot.rotate(90);
            }
        }
    }

    // /// right after the robot turn ,if it's a door ,use this to check////
    public boolean firstDoor() {
        pilot.forward();
        // //go pass the door////
        while (lightSensor.readNormalizedValue() > 400) {
            if (ultrasonicSensor.getDistance() < 10) {
                return false;
            }
        }
        // //move back to check////
        pilot.travel(-50);
        return isDoor();
    }

    public void getItBaby() {
        int track;
        double[] angle = new double[10];
        int ds;
        findWall();
        pilot.rotate(-90);
        while (ultrasonicSensor.getDistance() > 10) {
        }
        pilot.stop();
        pilot.travel(400);
        pilot.travel(-70);
        pilot.rotate(92);
        int shortwall = 80;// ////need test
        double speed;
        speed = pilot.getRotateSpeed();

        pilot.forward();
        while (ultrasonicSensor.getDistance() < shortwall) {
            pilot.forward();
            while (ultrasonicSensor.getDistance() > 10) {
            }
            pilot.stop();
            pilot.travel(400);
            pilot.travel(-70);
            pilot.rotate(90);
            System.out.println(ultrasonicSensor.getDistance());
            // Button.waitForAnyPress();
        }
        pilot.travel(ultrasonicSensor.getDistance() / 2 * 10 - 20);

        pilot.rotate(-90);
        pilot.travel(100);

        Motor.B.setSpeed(60);
        Motor.B.rotate(110);
        pilot.travel(-110);
        Motor.B.rotate(-90);
        pilot.travel(35);
        pilot.setRotateSpeed(speed);

    }

}