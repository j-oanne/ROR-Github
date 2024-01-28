package org.firstinspires.ftc.teamcode;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad2;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.opMode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "TeleOp")
@Config

/*
    Action:               Controls:                                              Gamepad ID:
    General Movement ---- Left Stick & Right Stick                               [gamepad 1]
    Linear Slide -------- Right Bumper (up) & Left Bumper (down)                 [gamepad 2]
    Intake -------------- Toggled; B (reverse) & X (forward)                     [gamepad 2]
    Roller Height ------- Right Trigger (down) & Left Trigger (up)               [gamepad 2]
    Outtake ------------- A                                                      [gamepad 2]
    Drone --------------- DPad Left                                              [gamepad 1]
    Hanging ------------- DPad Up and Down                                       [gamepad 2]
*/

public class Teleop extends LinearOpMode {

    // Configurations
    public static int SLIDE_RESET_TIME = 800;
    public static double SLIDE_POW = 0.65;
    public static double POW_MULTIPLIER = 0.75;
    public static double SLOW_MULTIPLIER = 0.3;
    public static double HANG_POW = 0.5;

    //RESET used to be .15
    public static double RESET = 0.15;

    // Mecanum Wheels
    public DcMotor leftBack;
    public DcMotor leftFront;
    public DcMotor rightBack;
    public DcMotor rightFront;

    // Intake to Outtake
    public TouchSensor touch;
    public DcMotor linearSlide;
    public DcMotor intakeRoller;
    public Servo outtake;

    //Miscellaneous
    public Servo drone;

    public Servo liftRelease;
    public DcMotor lift;

    // Counters
    public int countSlow = 0;
    public int countIntake = 1;
    public int countHang = 0;

    public void runOpMode() {

        // Initialize Mecanum wheels
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        leftBack = hardwareMap.get(DcMotor.class, "leftBack");
        rightBack = hardwareMap.get(DcMotor.class, "rightBack");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        rightBack.setDirection(DcMotor.Direction.REVERSE);

        // Initialize other motors and servos
        intakeRoller = hardwareMap.get(DcMotor.class, "intake");
        touch = hardwareMap.touchSensor.get("touchSensor");
        linearSlide = hardwareMap.get(DcMotor.class, "linearSlide");
        outtake = hardwareMap.get(Servo.class, "yury");
        drone = hardwareMap.get(Servo.class, "drone");
        lift = hardwareMap.get(DcMotor.class, "kent");
        liftRelease = hardwareMap.get(Servo.class, "liftRelease");

        // Reset initial servo positions
        liftRelease.setPosition(1);
        drone.setPosition(1);
        outtake.setPosition(1);
        lift.setPower(0);

        waitForStart();
        if (isStopRequested()) {
            return;
        }

        double hang_power = 0.1;

        while (opModeIsActive()) {

            // GAMEPAD 1 ACTIONS *****************************
            // Mecanum wheels
            move();

            // Release drone
            if (gamepad1.dpad_left) {
                drone.setPosition(0);
            }


            // GAMEPAD 2 ACTIONS ****************************
            // Turn intake on
            intakeRoller.setPower(gamepad2.right_trigger);
            // Reverse roller direction
            if (gamepad2.x) {
                intakeRoller.setPower(-1);
            }

            // Rotate outtake servo
            if (gamepad2.a) {
                outtake.setPosition(0);
                sleep(200);
                outtake.setPosition(1);
                sleep(200);
            }

            // Hanging mechanism
            if (gamepad2.dpad_up) {
                countHang++;
                while (gamepad2.dpad_up) {
                    lift.setPower(HANG_POW);
                }
            }
            if (gamepad2.dpad_down) {
                countHang = 0;
                lift.setPower(-0.1);
                while (gamepad2.dpad_down) {
                    lift.setPower(-0.3);
                }
            }

            if(gamepad2.dpad_right){
                liftRelease.setPosition(.5);
            }
            if (countHang != 0) { // Keep suspended in air
                lift.setPower(0.1); // 0.25 what is minimum power to keep suspended??
            }
            else {
                lift.setPower(0);
            }

            // Raise the cascading slide
            if (gamepad2.right_bumper) {
                while (!touch.isPressed()) {
                    linearSlide.setPower(SLIDE_POW);
                    move();
                    if (gamepad2.y) {
                        break;
                    }
                }
                while (touch.isPressed()) {
                    linearSlide.setPower(-.1);
                    if (gamepad2.y) {
                        break;
                    }
                }
                linearSlide.setPower(0);
            }

            // Lower the cascading slide
            if (gamepad2.left_bumper) {
                while (!touch.isPressed()) {
                    linearSlide.setPower(-.4);
                    move();
                    if (gamepad2.y) {
                        break;
                    }
                }
            }
            while (touch.isPressed()) {
                linearSlide.setPower(RESET);
                if (gamepad2.y) {
                    break;
                }
            }
            linearSlide.setPower(0);
        }
    }

    private void move() {

        double max;

        // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
        double axial   = -gamepad1.left_stick_y;  // Note: pushing stick forward gives negative value
        double lateral =  gamepad1.left_stick_x;
        double yaw     =  gamepad1.right_stick_x;

        // Combine the joystick requests for each axis-motion to determine each wheel's power.
        // Set up a variable for each drive wheel to save the power level for telemetry.

        double leftFrontPower  = axial + lateral + yaw;
        double rightFrontPower = axial - lateral - yaw;
        double leftBackPower   = axial - lateral + yaw;
        double rightBackPower  = axial + lateral - yaw;

        // Normalize the values so no wheel power exceeds 100%
        // This ensures that the robot maintains the desired motion.
        max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
        max = Math.max(max, Math.abs(leftBackPower));
        max = Math.max(max, Math.abs(rightBackPower));

        if (max > 1.0) {
            leftFrontPower  /= max;
            rightFrontPower /= max;
            leftBackPower   /= max;
            rightBackPower  /= max;
        }

        // Send calculated power to wheels
        if (countSlow % 2 == 0) {
            leftFront.setPower(leftFrontPower * POW_MULTIPLIER);
            rightFront.setPower(rightFrontPower * POW_MULTIPLIER);
            leftBack.setPower(leftBackPower * POW_MULTIPLIER);
            rightBack.setPower(rightBackPower * POW_MULTIPLIER);
        }
        else {
            leftFront.setPower(leftFrontPower * SLOW_MULTIPLIER);
            rightFront.setPower(rightFrontPower * SLOW_MULTIPLIER);
            leftBack.setPower(leftBackPower * SLOW_MULTIPLIER);
            rightBack.setPower(rightBackPower * SLOW_MULTIPLIER);
        }

        if (gamepad1.a) {
            countSlow++;
            sleep(300);
        }
    }
}