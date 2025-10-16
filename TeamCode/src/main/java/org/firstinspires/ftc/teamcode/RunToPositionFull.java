package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcontroller.GoBildaPinpointDriver;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

@TeleOp(name="RunToPositionFull", group="Linear OpMode")
public class RunToPositionFull extends LinearOpMode {

    GoBildaPinpointDriver odo;

    // Dead zone threshold (increased)
    double positionThreshold = 15; // mm
    double rotationThreshold = 0.3;  // degrees

    // Proportional control constants
    double kpPosition = 0.005;
    double kpRotation = 0.1; // Added proportional control for rotation

    double wheelLF, wheelRF, wheelLB, wheelRB;

    private DcMotor RBMotor, RFMotor, LFMotor, LBMotor;

    double[] goal = {0, 0, 0}; // Goal position: {X, Y, Rotation in degrees}

    public void movement(double X, double Y, double rotation) {
        Y = Y * -1;


        // Mecanum wheel calculation
        wheelLF = Y + X - rotation;
        wheelRF = Y - X + rotation;
        wheelLB = Y - X - rotation;
        wheelRB = Y + X + rotation;

        // Set motor powers
        LFMotor.setPower(wheelLF);
        RFMotor.setPower(wheelRF);
        LBMotor.setPower(wheelLB);
        RBMotor.setPower(wheelRB);
    }

    @Override
    public void runOpMode() {
        // Motor initialization
        RBMotor = hardwareMap.get(DcMotor.class, "RBMotor");
        RFMotor = hardwareMap.get(DcMotor.class, "RFMotor");
        LFMotor = hardwareMap.get(DcMotor.class, "LFMotor");
        LBMotor = hardwareMap.get(DcMotor.class, "LBMotor");

        RBMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RFMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LBMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LFMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        RBMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        RFMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        odo = hardwareMap.get(GoBildaPinpointDriver.class, "odo");
        odo.setOffsets(-96, -137.25);
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);
        odo.resetPosAndIMU();

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        resetRuntime();

        while (opModeIsActive()) {
            odo.update();
            Pose2D pos = odo.getPosition();

            double y = -gamepad1.left_stick_y * 10; // Invert y to match forward movement
            double x = gamepad1.left_stick_x * 10;
            double z = gamepad1.right_stick_x;

            if (z < -180){
                z = 180;
            }
            if (z > 180){
                z = -180;
            }
            goal[0] += x;
            goal[1] += y;
            goal[2] += z;

            if (goal[2] < -180){
                goal[2] = 180;
            }
            if (goal[2] > 180){
                goal[2] = -180;
            }

            double currentX = pos.getX(DistanceUnit.MM);
            double currentY = pos.getY(DistanceUnit.MM);
            double currentRotation = 10 * odo.getHeading(); // Get current IMU heading

            // Calculate position and rotation errors, go builda sensors are messed up so swap x and y
            double errorX = goal[0] - currentY;
            double errorY = goal[1] - currentX;
            double errorRotation = goal[2] - currentRotation;

            // Wrap rotation error between -180 and 180 degrees
            errorRotation = ((errorRotation + 180) % 360 + 360) % 360 - 180;

            // Apply proportional control
            double moveX = errorX * kpPosition;
            double moveY = errorY * kpPosition;
            double turn = errorRotation * kpRotation;

            // Dead zone: stop movement if close to goal
            if (Math.abs(errorX) < positionThreshold && Math.abs(errorY) < positionThreshold) {
                moveX = 0;
                moveY = 0;
            }

            if (Math.abs(errorRotation) < rotationThreshold) {
                turn = 0;
            }



            // Feed movement values
            movement(moveX, moveY, -turn);

            // Telemetry
            telemetry.addData("Goal X", goal[0]);
            telemetry.addData("Goal Y", goal[1]);
            telemetry.addData("Goal Rotation", goal[2]);
            telemetry.addData("Current X", currentX);
            telemetry.addData("Current Y", currentY);
            telemetry.addData("Current Rotation", currentRotation);
            telemetry.addData("Error X", errorX);
            telemetry.addData("Error Y", errorY);
            telemetry.addData("Error Rotation", errorRotation);
            telemetry.update();

            // Reset position and IMU with gamepad inputs
            if (gamepad1.a) {
                odo.resetPosAndIMU();
            }

            if (gamepad1.b) {
                odo.recalibrateIMU();
            }

            if (gamepad1.y) {
                goal[0] = currentY;
                goal[1] = currentX;
                goal[2] = currentRotation;
            }
        }
    }
}
