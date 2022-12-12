package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name = "drive (Blocks to Java)")
public class Jdrive extends LinearOpMode {

    private ColorSensor colorsensor_REV_ColorRangeSensor;
    private DcMotor backright;
    private DcMotor frontleft;
    private DcMotor backleft;
    private DcMotor frontright;
    private TouchSensor digital_touch;
    private DcMotor armmotor;
    private Servo graber;

    /**
     * Describe this function...
     */
    private void update_colors() {
        telemetry.addData("Color value: ", JavaUtil.colorToText(colorsensor_REV_ColorRangeSensor.argb()));
        telemetry.addData("Red value: ", JavaUtil.colorToText(colorsensor_REV_ColorRangeSensor.red()));
        telemetry.addData("Blue value: ", JavaUtil.colorToText(colorsensor_REV_ColorRangeSensor.blue()));
        telemetry.addData("Green value:", JavaUtil.colorToText(colorsensor_REV_ColorRangeSensor.green()));
        telemetry.addData("Color distance: ", ((DistanceSensor) colorsensor_REV_ColorRangeSensor).getDistance(DistanceUnit.CM));
        telemetry.addData("Read Color Function Output", read_color());
    }

    /**
     * Describe this function...
     */
    private String read_color() {
        int red_value;
        String color_result;
        int blue_value;
        int green_value;

        red_value = colorsensor_REV_ColorRangeSensor.red();
        blue_value = colorsensor_REV_ColorRangeSensor.blue();
        green_value = colorsensor_REV_ColorRangeSensor.green();
        if (blue_value == JavaUtil.maxOfList(JavaUtil.createListWith(red_value, blue_value, green_value))) {
            color_result = "blue";
        } else if (red_value == JavaUtil.maxOfList(JavaUtil.createListWith(red_value, blue_value, green_value))) {
            color_result = "red";
        } else {
            color_result = "green";
        }
        return color_result;
    }

    /**
     * This function is executed when this Op Mode is selected from the Driver Station.
     */
    @Override
    public void runOpMode() {
        double SpeedAdapter;
        double BackLeftDrive;
        double BackLeftCrabWalk;
        float BackLeftTurn;
        double BackRightDrive;
        double BackRightCrabWalk;
        float BackRightTurn;
        double FrontLeftDrive;
        double FrontLeftCrabWalk;
        float FrontLeftTurn;
        double FrontRightDrive;
        double FrontRightCrabWalk;
        float FrontRightTurn;


        colorsensor_REV_ColorRangeSensor = hardwareMap.get(ColorSensor.class, "color sensor");
        backright = hardwareMap.get(DcMotor.class, "back right");
        frontleft = hardwareMap.get(DcMotor.class, "front left");
        backleft = hardwareMap.get(DcMotor.class, "back left");
        frontright = hardwareMap.get(DcMotor.class, "front right");
        digital_touch = hardwareMap.get(TouchSensor.class, "digital_touch");
        armmotor = hardwareMap.get(DcMotor.class, "arm motor");
        graber = hardwareMap.get(Servo.class, "graber");

        waitForStart();
        while (opModeIsActive()) {
            update_colors();
            backright.setDirection(DcMotorSimple.Direction.REVERSE);
            frontleft.setDirection(DcMotorSimple.Direction.REVERSE);
            if (Math.abs(gamepad1.left_stick_x) + Math.abs(gamepad1.left_stick_y) == 0) {
                SpeedAdapter = 0;
            } else {
                SpeedAdapter = Math.sqrt(Math.pow(gamepad1.left_stick_x, 2) + Math.pow(gamepad1.left_stick_y, 2)) / (Math.abs(gamepad1.left_stick_x) + Math.abs(gamepad1.left_stick_y));
            }
            BackLeftDrive = gamepad1.left_stick_y * (SpeedAdapter / 2);
            BackLeftCrabWalk = gamepad1.left_stick_x * (SpeedAdapter / 2);
            BackLeftTurn = gamepad1.right_stick_x / -2;
            backleft.setPower(BackLeftDrive + BackLeftCrabWalk + BackLeftTurn);

            BackRightDrive = gamepad1.left_stick_y * (SpeedAdapter / 2);
            BackRightCrabWalk = gamepad1.left_stick_x * (SpeedAdapter / -2);
            BackRightTurn = gamepad1.right_stick_x / 2;
            backright.setPower(BackRightDrive + BackRightCrabWalk + BackRightTurn);

            FrontLeftDrive = gamepad1.left_stick_y * (SpeedAdapter / 2);
            FrontLeftCrabWalk = gamepad1.left_stick_x * (SpeedAdapter / -2);
            FrontLeftTurn = gamepad1.right_stick_x / -2;
            frontleft.setPower(FrontLeftDrive + FrontLeftCrabWalk + FrontLeftTurn);

            FrontRightDrive = gamepad1.left_stick_y * (SpeedAdapter / 2);
            FrontRightCrabWalk = gamepad1.left_stick_x * (SpeedAdapter / 2);
            FrontRightTurn = gamepad1.right_stick_x / 2;
            frontright.setPower(FrontRightDrive + FrontRightCrabWalk + FrontRightTurn);

            if (digital_touch.isPressed()) {
                armmotor.setPower(0);
            } else {
                armmotor.setPower(gamepad2.right_trigger + -1 * gamepad2.left_trigger);
            }
            if (gamepad2.dpad_left) {
                graber.setPosition(1);
            } else if (gamepad2.dpad_right) {
                graber.setPosition(-0.5);
            }
            if (gamepad2.left_bumper) {
                graber.setPosition(1);
            } else if (gamepad2.right_bumper) {
                graber.setPosition(-0.5);
            }
            telemetry.update();
        }
    }
}