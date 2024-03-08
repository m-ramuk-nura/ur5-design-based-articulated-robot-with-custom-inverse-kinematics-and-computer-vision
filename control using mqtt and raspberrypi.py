import RPi.GPIO as GPIO
import time
import paho.mqtt.client as mqtt

broker_address = "broker.hivemq.com"
port = 1883
J1_DIR_PIN = 17
J1_PULSE_PIN = 27
J1_limit_switch_pin = 4
J2_DIR_PIN = 13
J2_PULSE_PIN = 6
J2_limit_switch_pin = 22
J3_DIR_PIN = 21
J3_PULSE_PIN = 20
J3_limit_switch_pin = 5
GPIO.setmode(GPIO.BCM)

GPIO.setup(J1_DIR_PIN, GPIO.OUT)
GPIO.setup(J1_PULSE_PIN, GPIO.OUT)
GPIO.setup(J1_limit_switch_pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(J2_DIR_PIN, GPIO.OUT)
GPIO.setup(J2_PULSE_PIN, GPIO.OUT)
GPIO.setup(J2_limit_switch_pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(J3_DIR_PIN, GPIO.OUT)
GPIO.setup(J3_PULSE_PIN, GPIO.OUT)
GPIO.setup(J3_limit_switch_pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)
pub = 1
topic = "moveit"


def on_connect(client, userdata, flags, rc):
    global pub

    if rc == 0:
        print("Connected to MQTT broker")

        # Subscribe to the specified topic
        if pub == 1:
            client.publish("finisheddone", 0)
            pub = 0
            client.subscribe(topic)
    else:
        print("Connection failed with error code " + str(rc))


def on_message(client, userdata, message):
    received_topic = message.topic
    received_message = message.payload.decode("utf-8")

    print(f"Received message on topic '{received_topic}': {received_message}")

    # Split the received message into four variables using spaces
    parts = received_message.split(" ")
    if len(parts) == 4:
        var1, var2, var3, var4 = parts
        J1_check = 0
        J1_check_2 = 0
        J2_check = 0
        J2_check_2 = 0
        J3_check = 0
        J3_check_2 = 0
        J4_check = 0
        J4_check_2 = 0

        J5_check = 0
        J5_check_2 = 0
        J6_check = 0
        J6_check_2 = 0
        step_j1 = abs(int(var1))
        step_j2 = abs(int(var2))
        step_j3 = abs(int(var3))
        joint1_side = var4
        if joint1_side == "1":
            for _ in range(2000):

                if J1_check_2 <= (step_j1 - 1):
                    GPIO.output(J1_DIR_PIN, 1)

                    GPIO.output(J1_PULSE_PIN, 1)
                    time.sleep(0.008)
                    GPIO.output(J1_PULSE_PIN, 0)
                    time.sleep(0.008)
                    J1_check_2 = J1_check_2 + 1
                    if J1_check_2 == step_j1:
                        GPIO.output(J1_DIR_PIN, 0)
                        GPIO.output(J1_PULSE_PIN, 0)
        if joint1_side == "0":
            for _ in range(2000):

                if J1_check_2 <= (step_j1 - 1):
                    GPIO.output(J1_DIR_PIN, 0)

                    GPIO.output(J1_PULSE_PIN, 1)
                    time.sleep(0.008)
                    GPIO.output(J1_PULSE_PIN, 0)
                    time.sleep(0.008)
                    J1_check_2 = J1_check_2 + 1
                    if J1_check_2 == step_j1:
                        GPIO.output(J1_DIR_PIN, 0)
                        GPIO.output(J1_PULSE_PIN, 0)
        for _ in range(3000):

            if J2_check_2 <= (step_j2 - 1):
                GPIO.output(J2_DIR_PIN, 1)

                GPIO.output(J2_PULSE_PIN, 1)
                time.sleep(0.008)
                GPIO.output(J2_PULSE_PIN, 0)
                time.sleep(0.008)
                J2_check_2 = J2_check_2 + 1
                print(J2_check_2)
                if J2_check_2 == step_j2:
                    GPIO.output(J2_DIR_PIN, 0)
                    GPIO.output(J2_PULSE_PIN, 0)
            if J3_check_2 <= (step_j3 - 1):

                GPIO.output(J3_DIR_PIN, 1)

                # Generate a pulse
                GPIO.output(J3_PULSE_PIN, 1)
                time.sleep(0.005)
                GPIO.output(J3_PULSE_PIN, 0)
                time.sleep(0.005)
                J3_check_2 = J3_check_2 + 1
                print()
                if J3_check_2 == step_j3:
                    GPIO.output(J3_DIR_PIN, 0)
                    GPIO.output(J3_PULSE_PIN, 0)
        J2_check_2 = 0
        J3_check_2 = 0
        J1_check_2 = 0
        for _ in range(3000):

            if J2_check_2 <= (step_j2 - 1):

                GPIO.output(J2_DIR_PIN, 0)

                GPIO.output(J2_PULSE_PIN, 1)
                time.sleep(0.008)
                GPIO.output(J2_PULSE_PIN, 0)
                time.sleep(0.008)
                J2_check_2 = J2_check_2 + 1
                print(J2_check_2)
                if J2_check_2 == step_j2:
                    GPIO.output(J2_DIR_PIN, 0)
                    GPIO.output(J2_PULSE_PIN, 0)
            if J3_check_2 <= (step_j3 - 1):

                GPIO.output(J3_DIR_PIN, 0)

                # Generate a pulse
                GPIO.output(J3_PULSE_PIN, 1)
                time.sleep(0.005)
                GPIO.output(J3_PULSE_PIN, 0)
                time.sleep(0.005)
                J3_check_2 = J3_check_2 + 1
                print()
                if J3_check_2 == step_j3:
                    GPIO.output(J3_DIR_PIN, 0)
                    GPIO.output(J3_PULSE_PIN, 0)
        if joint1_side == "1":
            for _ in range(1500):

                if J1_check_2 <= (step_j1 + 999):
                    GPIO.output(J1_DIR_PIN, 0)

                    GPIO.output(J1_PULSE_PIN, 1)
                    time.sleep(0.008)
                    GPIO.output(J1_PULSE_PIN, 0)
                    time.sleep(0.008)
                    J1_check_2 = J1_check_2 + 1
                    if J1_check_2 == step_j1 + 1000:
                        GPIO.output(J1_DIR_PIN, 0)
                        GPIO.output(J1_PULSE_PIN, 0)
        if joint1_side == "0":
            for _ in range(2000):

                if J1_check_2 <= (step_j1 - 999) * -1:
                    GPIO.output(J1_DIR_PIN, 0)

                    GPIO.output(J1_PULSE_PIN, 1)
                    time.sleep(0.008)
                    GPIO.output(J1_PULSE_PIN, 0)
                    time.sleep(0.008)
                    J1_check_2 = J1_check_2 + 1
                    if J1_check_2 == (step_j1 - 1000):
                        GPIO.output(J1_DIR_PIN, 0)
                        GPIO.output(J1_PULSE_PIN, 0)
        J3_check_2 = 0
        J1_check_2 = 0
        for _ in range(2000):

            if J1_check_2 <= (999):
                GPIO.output(J1_DIR_PIN, 1)

                GPIO.output(J1_PULSE_PIN, 1)
                time.sleep(0.008)
                GPIO.output(J1_PULSE_PIN, 0)
                time.sleep(0.008)
                J1_check_2 = J1_check_2 + 1
                if J1_check_2 == 1000:
                    GPIO.output(J1_DIR_PIN, 0)
                    GPIO.output(J1_PULSE_PIN, 0)
        global pub
        pub = 1

        client.connect(broker_address, port)
        client = mqtt.Client()

        # Set the callback functions
        client.on_connect = on_connect

        # Connect to the MQTT broker


client = mqtt.Client()

# Set the callback functions
client.on_connect = on_connect
client.on_message = on_message

# Connect to the MQTT broker
client.connect(broker_address, port)
client.loop_start()

# Start the MQTT client loop


