import lejos.nxt.Motor;
import lejos.robotics.RegulatedMotor;
import lejos.robotics.navigation.DifferentialPilot;
import lejos.util.Delay;
import lejos.nxt.SensorPort;
import lejos.nxt.UltrasonicSensor;

public class Challenge1 {
    private final float wheelDiameter;
    private final float trackWidth;
    private final RegulatedMotor leftMotor;
    private final RegulatedMotor rightMotor;
    private final DifferentialPilot pilot;
    private final SensorPort ultrasonicPort;
    private final UltrasonicSensor ultrasonicSensor;

    public static void main(String[] args) {
        final Challenge1 robot;
        robot = new Challenge1();
        robot.run();      
    }
    public Challenge1(){
        ultrasonicPort = SensorPort.S2;
        ultrasonicSensor = new UltrasonicSensor(ultrasonicPort);
     // 56 mm (written on the side of the tire)
        wheelDiameter = 56.0f;
        // 114 mm (measured between the centre of both wheels)
        trackWidth = 114.0f;
        leftMotor = Motor.A;
        rightMotor = Motor.C;
        pilot = new DifferentialPilot(wheelDiameter, trackWidth, leftMotor,
                rightMotor);
    }
    public void run(){
        pilot.forward();
        waitForDistance(30);
        pilot.steer(58, 180);
        pilot.forward();
        waitForDistance(30);
        pilot.steer(-50, -180);
        pilot.forward();
        waitForDistance(30);
    }
    private int waitForDistance(final int max) {
        int distance;
        
        do {
            Delay.msDelay(100);
            distance = ultrasonicSensor.getDistance();
        } while (distance > max);
        return (distance);
    }
}