package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "goToArmCode")

public class goToArmCode extends LinearOpMode {

    // Debug turns on all of the telemetry that is unnecessary
    private boolean debug = true;

    private DcMotor LSlide;
    private DcMotor RSlide;
    private CRServo tilt;       // Continuous rotation servo for tilt mechanism
    private CRServo intake;
    private Servo goodtilt;     // Added goodtilt as a standard servo

    private double[] offsets = {0, 0};

    private double armGoal;
    private double armGoalDeadzone = 100;
    private double positon;

    private double top = 2000;

    private boolean slidesInPos = false;

    // Tilt control thresholds
    private double positionThreshold = 1500; // Slide position at which the tilt activates
    private boolean tiltMoved = false;       // To track if tilt has already moved

    // Tilt simulation variables
    private double tiltMoveDuration = 0.20; // Approximate time (in seconds) to reach position
    private long tiltStartTime;

    public void moveSlides(boolean input, double endPoint) {

        int LPos = LSlide.getCurrentPosition();
        int RPos = RSlide.getCurrentPosition();

        positon = ((LSlide.getCurrentPosition() + offsets[0]) + (RSlide.getCurrentPosition() + offsets[1])) / 2;

        if (input) {
            double up = 0;

            if (gamepad2.dpad_up) {
                up = 25;
            } else if (gamepad2.dpad_down) {
                up = -25;
            }

            armGoal += up;

            if (armGoal > top) {
                armGoal = top;
            }
            if (armGoal < 0) {
                armGoal = 0;
            }
        } else {
            armGoal = endPoint;
        }

        // Calculate the error (distance to goal)
        double error = armGoal - positon;

        // Use proportional control with speed reduction near the target
        double out = error * 0.1;

        // Clamp power to a maximum and minimum range
        double minPower = 0.1; // Prevent stalling near the target
        double maxPower = 1.0;

        // Scale the power down as it gets closer
        out = Math.signum(out) * Math.max(minPower, Math.min(Math.abs(out), maxPower));
 
        // Deadzone check
        boolean insideDeadzone = Math.abs(error) <= armGoalDeadzone;

        if (!insideDeadzone) {
            LSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            RSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            LSlide.setPower(out);
            RSlide.setPower(out);
        } else {
            LSlide.setTargetPosition((int) (armGoal + offsets[0]));
            RSlide.setTargetPosition((int) (armGoal + offsets[1]));

            LSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            RSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            LSlide.setPower(0.3); // Low power for holding position
            RSlide.setPower(0.3);
        }

        // Adjust tilt servo based on position
        adjustTiltBasedOnPosition();

        // Debug telemetry
        if (debug) {
            telemetry.addData("Speed on Arm: ", out);
            telemetry.addData("Arm Goal: ", armGoal);
            telemetry.addData("Position: ", positon);
            telemetry.addData("Tilt Moved: ", tiltMoved);
            telemetry.update();
        }

        // Check if slides are in position
        slidesInPos = !input && insideDeadzone;
    }

    // Function to simulate moving the tilt CRServo to a "position"
    private void adjustTiltBasedOnPosition() {
        // Move tilt forward if slides are above the threshold
        if (positon > positionThreshold && !tiltMoved) {
            tilt.setPower(0.8); // Run servo forward
            tiltStartTime = System.currentTimeMillis();
            tiltMoved = true; // Mark tilt as moved
        }

        // Stop forward tilt after the move duration
        if (tiltMoved && tilt.getPower() > 0) {
            long elapsedTime = System.currentTimeMillis() - tiltStartTime;
            if (elapsedTime > (tiltMoveDuration * 1000)) {
                tilt.setPower(0.0); // Stop the servo
            }
        }

        // Reverse tilt if slides go back below the threshold
        if (positon < positionThreshold && tiltMoved) {
            tilt.setPower(-0.8); // Reverse servo to simulate reset
            tiltStartTime = System.currentTimeMillis();
            tiltMoved = false; // Reset the flag after initiating reverse movement
        }

        // Stop reverse tilt after the move duration
        if (!tiltMoved && tilt.getPower() < 0) {
            long elapsedTime = System.currentTimeMillis() - tiltStartTime;
            if (elapsedTime > (tiltMoveDuration * 1000)) {
                tilt.setPower(0.0); // Stop the servo
            }
        }
    }

    @Override
    public void runOpMode() {
        // Initialization for motors
        LSlide = hardwareMap.get(DcMotor.class, "LSlide");
        RSlide = hardwareMap.get(DcMotor.class, "RSlide");
        tilt = hardwareMap.get(CRServo.class, "tilt");
        goodtilt = hardwareMap.get(Servo.class, "goodtilt");
        intake = hardwareMap.get(CRServo.class, "LIntake");

        // Set motors to use encoders
        LSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        LSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        offsets[0] = -LSlide.getCurrentPosition();
        offsets[1] = -RSlide.getCurrentPosition();

        LSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        positon = 0;
        armGoal = 0;

        positon = ((LSlide.getCurrentPosition() + offsets[0]) + (RSlide.getCurrentPosition() + offsets[1])) / 2;

        LSlide.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();

        while (opModeIsActive()) {
            moveSlides(true, 0);

            if (gamepad2.a) {
                tilt.setPower(0.8);
                goodtilt.setPosition(1.0); // Set goodtilt to max position
            }
            if (gamepad2.b) {
                tilt.setPower(-0.8);
                goodtilt.setPosition(0.0); // Set goodtilt to min position
            }

            if (gamepad2.left_bumper) {
                intake.setPower(0.2);
            } else if (gamepad2.right_bumper) {
                intake.setPower(-0.2);
            } else {
                intake.setPower(0);
            }
        }
    }
}
