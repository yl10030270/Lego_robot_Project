import lejos.nxt.*;
import java.io.*;
import lejos.robotics.RegulatedMotor;
import lejos.robotics.navigation.DifferentialPilot;
import lejos.util.Delay;
import lejos.nxt.comm.RConsole;

/* Changes from version 5.17
 * 2.put extra lightsensor to deal with the corner
 * Planning change
 * 
 * 2.put alternate beginning condition-check to avoid miss the key
 */

public class Challenge3v3 {
    private final float wheelDiameter;
    private final float trackWidth;
    private final RegulatedMotor leftMotor;
    private final RegulatedMotor rightMotor;
    private final DifferentialPilot pilot;
    private final SensorPort ultrasonicPort;
    private final UltrasonicSensor ultrasonicSensor;
    private final SensorPort lightPort;
    private final LightSensor lightSensor;
    private final SensorPort lightPort2;
    private final LightSensor lightSensor2;
    // private final TouchSensor touchSensor;
    // private final SensorPort touchPort;
    private final int black;
    private final int white;
    private final int maxspeed;
    private final double turnrate;

    public static void main(String[] args) {
        final Challenge3v3 robot;
        robot = new Challenge3v3();
        robot.run();
    }

    public Challenge3v3() {
        ultrasonicPort = SensorPort.S2;
        ultrasonicSensor = new UltrasonicSensor(ultrasonicPort);
        lightPort = SensorPort.S4;
        lightSensor = new LightSensor(lightPort);
        lightSensor.setFloodlight(true);
        lightPort2 = SensorPort.S3;
        lightSensor2 = new LightSensor(lightPort2);
        lightSensor2.setFloodlight(true);
        wheelDiameter = 56.0f;
        // 114 mm (measured between the centre of both wheels)
        trackWidth = 114.0f;
        leftMotor = Motor.A;
        rightMotor = Motor.C;
        pilot = new DifferentialPilot(wheelDiameter, trackWidth, leftMotor,
                rightMotor);
        black = 32;// need test***********************set up
        white = 48;// need test***********************set up
        maxspeed = 120;// need test********************set up
        turnrate = 34;
    }

    public void run() {
        double s;
        int light;
        int light2;
        int testlight;
        boolean key = false;
        boolean end = true;

        light = lightSensor.readValue();
        System.out.println("" + light);
        pilot.setRotateSpeed(85);
        Button.waitForAnyPress();
        double kp; // run off the line increase Kp. oscillates wildly then
                   // decrease Kp.
        int offset;
        double turn;
        int error;
        offset = (white + black) / 2;
        kp = 200 / (white - offset) * 0.08;
        pilot.setTravelSpeed(maxspeed);

        // ////////////////////////////////////////////////////////////////////////////////////
        begin();
        // ///////////////////////////////////////////////////////////////////////////////////
        while (ultrasonicSensor.getDistance() > 22) {// need test //
            // *******************
            // ultrasonicSensor.reset();
            // for (int q = 1; q <= 3; q++) {
            // if (ultrasonicSensor.getDistance() >= 24) {
            // end = false;
            // // Delay.msDelay(100);
            // }
            // }
            // if (end) {
            // break;
            // }
            // end = true;
            light = lightSensor.readValue();
            light2 = lightSensor2.readValue();
            if (light < 35 || light > 45 || light2 < 40) {// || light2 < 40
                if (light2 < 40) {
                    pilot.rotate(-45);
                    light = lightSensor.readValue();
                }
                if (light < 35) {// || light2 < 40
                    // s = pilot.getRotateSpeed();
                    // pilot.setRotateSpeed(200);
                    // for (int q = 1; q <= 3; q++) {
                    // if (ultrasonicSensor.getDistance() >= 24) {
                    // end = false;
                    // Delay.msDelay(100);
                    // }
                    // }
                    if (ultrasonicSensor.getDistance() <= 24) {
                        System.out.println("" + ultrasonicSensor.getDistance());
                        System.out.println("inside,outside,already");
                        pilot.stop();
                        // Button.waitForAnyPress();
                        // findWall();
                        pilot.travel(ultrasonicSensor.getDistance() * 10);
                        Motor.B.setSpeed(180);
                        Motor.B.rotate(90);
                        // pilot.setRotateSpeed(s);
                        return;
                    }
                    // end = true;
                    pilot.rotate(70);

                    // for (int q = 1; q <= 3; q++) {
                    // if (ultrasonicSensor.getDistance() >= 24) {
                    // end = false;
                    // Delay.msDelay(100);
                    // }
                    // }
                    // s = ultrasonicSensor.getDistance();
                    if (ultrasonicSensor.getDistance() <= 24) {
                        System.out.println("" + ultrasonicSensor.getDistance());
                        System.out.println("inside,already");
                        pilot.stop();

                        // Button.waitForAnyPress();
                        // findWall();
                        pilot.travel(ultrasonicSensor.getDistance() * 10);
                        Motor.B.setSpeed(180);
                        Motor.B.rotate(90);
                        // pilot.setRotateSpeed(s);
                        return;
                    }
                    end = true;
                    pilot.rotate(-70);
                    pilot.rotate(-200, true);
                    while (pilot.isMoving()) {
                        light = lightSensor.readValue();
                        // if (ultrasonicSensor.getDistance() <= 23) {
                        // pilot.stop();
                        // //Button.waitForAnyPress();
                        // //pilot.rotate(-20);
                        // findWall();
                        // pilot.travel(ultrasonicSensor.getDistance() * 10);
                        // Motor.B.setSpeed(180);
                        // Motor.B.rotate(90);
                        // return;
                        // }
                        if (light > 40) {
                            // pilot.stop();
                            break;
                        }
                    }
                } else {
                    pilot.steer(turnrate);
                    while (pilot.isMoving()) {
                        light = lightSensor.readValue();
                        light2 = lightSensor2.readValue();
                        if (ultrasonicSensor.getDistance() <= 23) {
                            break;
                        }
                        if (light2 <= 36 || (light >= 35 && light <= 45)) {
                            if (light2 <= 36) {
                                pilot.stop();
                                // Button.waitForAnyPress();
                                pilot.rotate(-45);
                                pilot.rotate(-200, true);
                                while (pilot.isMoving()) {
                                    light = lightSensor.readValue();

                                    if (light > 40) {
                                        // pilot.stop();
                                        break;
                                    }
                                }
                            }
                            break;
                        }
                    }
                    // //////need test/////////////////////
                }

            }

            error = light - offset;
            turn = kp * error;
            pilot.steer(turn);
        }
        // System.out.println("" + ultrasonicSensor.getDistance());
        System.out.println("out,already");
        // pilot.stop();
        // Button.waitForAnyPress();
        // pilot.rotate(10);
        // Button.waitForAnyPress();
        pilot.travel(ultrasonicSensor.getDistance() * 10);
        Motor.B.setSpeed(180);
        Motor.B.rotate(90);
    }

    public void getKey() {
        pilot.setRotateSpeed(85);
        // int ds;
        // double[] turntrack = new double[2];
        // double[] angle = new double[10];
        int cl = ultrasonicSensor.getDistance();
        // int j = 0;
        // pilot.rotate(10);
        // if ( cl <= 20 ) {
        pilot.travel(cl * 10);
        Motor.B.setSpeed(180);
        Motor.B.rotate(-100);

    }

    public void begin() {
        boolean end = true;
        pilot.travel(290);
        pilot.rotate(90);
        if (ultrasonicSensor.getDistance() <= 22) {

            getKey();

        } else {
            pilot.rotate(-90);
            if (ultrasonicSensor.getDistance() <= 22) {
                getKey();
                pilot.travelArc(-100, 150);
                pilot.travelArc(-100, -150);
                Delay.msDelay(500);
                pilot.travel(-150);
                pilot.rotate(-180);
            } else {
                pilot.rotate(-90);
                getKey();
                pilot.travelArc(-100, 150);
                pilot.travelArc(-100, -150);
                Delay.msDelay(500);
                pilot.travel(-180);
                pilot.rotate(90);
                // pilot.travel(-cl * 10);
                // pilot.rotate(90);
            }
        }
    }
}