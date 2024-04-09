#include <Arduino.h>

const byte LEFT_MOTOR_PIN1 = 5;
const byte LEFT_MOTOR_PIN2 = 6;
const byte RIGHT_MOTOR_PIN1 = 7;
const byte RIGHT_MOTOR_PIN2 = 8;
const byte LEFT_MOTOR_EN = 9;
const byte RIGHT_MOTOR_EN = 11;

// Định nghĩa các chân kết nối với cảm biến
const int sensorPins[] = {A0, A1, A2, A3, A4};
const int MAX_SPEED = 255;
const int MIN_SPEED = 65;
const int DEFAULT_SPEED = 105;

int caclulatePID(int error);
int calculeError();
void moveBackward();
void moveForward(int speed);
void turnRight(int speed);
void turnLeft(int speed);
void solve(int pid);
void turnLeft90(int speed);

float p = 0;
float i = 0;
float d = 0;
int lastError = 0;

int sensorValues[5];
const int numSensors = 5;
int readSensors();
void setup()
{
    Serial.begin(9600);
    pinMode(LEFT_MOTOR_PIN1, OUTPUT);
    pinMode(LEFT_MOTOR_PIN2, OUTPUT);
    pinMode(RIGHT_MOTOR_PIN1, OUTPUT);
    pinMode(RIGHT_MOTOR_PIN2, OUTPUT);
    pinMode(LEFT_MOTOR_EN, OUTPUT);
    pinMode(RIGHT_MOTOR_EN, OUTPUT);

    digitalWrite(LEFT_MOTOR_PIN1, HIGH);
    digitalWrite(LEFT_MOTOR_PIN2, LOW);
    digitalWrite(RIGHT_MOTOR_PIN1, HIGH);
    digitalWrite(RIGHT_MOTOR_PIN2, LOW);

    for (int i = 0; i < numSensors; i++)
    {
        pinMode(sensorPins[i], INPUT);
    }
}

void loop()
{

    // in giá trị cảm biến
    readSensors();
    // for (int i = 0; i < numSensors; i++) {
    //     Serial.print(sensorValues[i]);
    //     Serial.println(" ");
    // }
    int error = calculeError();
    Serial.print("Error: ");
    Serial.println(error);
    int pid = caclulatePID(error);
    Serial.print("PID: ");
    Serial.println(pid);

    solve(pid);
    delay(50);
    // delay(2000);
}

// duyệt các giá trị của cảm biến viết hàm đưa các giá trị >1000 về 1 và <1000 về 0

int readSensors()
{
    for (int i = 0; i < numSensors; i++)
    {
        sensorValues[i] = analogRead(sensorPins[i]);
        sensorValues[i] = (sensorValues[i] > 200) ? 1 : 0;
    }
    return 0;
}

int calculeError()
{
    int values[5] = {-4, -2, 0, 2, 4};
    int error = 0;
    for (int i = 0; i < numSensors; i++)
    {
        error += values[i] * sensorValues[i];
    }
    return error;
}

int caclulatePID(int error)
{
    float kp = 3;
    float ki = 0.01;
    float kd = 0.05;

    p = error;
    i = i + error;
    d = (error - lastError);
    lastError = error;
    return kp * p + ki * i + kd * d;
}

void solve(int pid)
{
    int speed = abs(pid) * 12;
    speed = (speed == 0) ? DEFAULT_SPEED : speed;
    speed = (speed < MIN_SPEED) ? MIN_SPEED : speed;

    if (speed == 255) {
        delay(300);
        turnLeft90(speed);
        return;
    }

    if (pid > 0)
    {
        turnLeft(abs(speed));
    }
    else if (pid < 0)
    {
        turnRight(abs(speed));
    }
    else
    {
        moveForward(abs(speed));
    }
}

// Hàm điều khiển động cơ với tốc độ có thể điều chỉnh
// Hàm đi lùi
void moveBackward()
{
    analogWrite(LEFT_MOTOR_EN, DEFAULT_SPEED);
    analogWrite(RIGHT_MOTOR_EN, DEFAULT_SPEED);
    digitalWrite(LEFT_MOTOR_PIN1, LOW);
    digitalWrite(LEFT_MOTOR_PIN2, HIGH);
    digitalWrite(RIGHT_MOTOR_PIN1, LOW);
    digitalWrite(RIGHT_MOTOR_PIN2, HIGH);
}

// Hàm rẽ phải
void turnRight(int speed)
{
    Serial.println("Turn right: ");
    Serial.println(speed);
    analogWrite(LEFT_MOTOR_EN, speed+7.65);
    analogWrite(RIGHT_MOTOR_EN, speed / 2);
    digitalWrite(LEFT_MOTOR_PIN1, HIGH);
    digitalWrite(LEFT_MOTOR_PIN2, LOW);
    digitalWrite(RIGHT_MOTOR_PIN1, LOW);
    digitalWrite(RIGHT_MOTOR_PIN2, HIGH);
}

// Hàm rẽ trái
void turnLeft(int speed)
{
    Serial.println("Turn left: ");
    Serial.println(speed);
    analogWrite(LEFT_MOTOR_EN, speed / 2+7.65);
    analogWrite(RIGHT_MOTOR_EN, speed);
    digitalWrite(LEFT_MOTOR_PIN1, LOW);
    digitalWrite(LEFT_MOTOR_PIN2, HIGH);
    digitalWrite(RIGHT_MOTOR_PIN1, HIGH);
    digitalWrite(RIGHT_MOTOR_PIN2, LOW);
}

void turnLeft90(int speed)
{
    Serial.println("Turn left: ");
    Serial.println(speed);
    analogWrite(LEFT_MOTOR_EN, 160);
    analogWrite(RIGHT_MOTOR_EN, 100);
    digitalWrite(LEFT_MOTOR_PIN1, HIGH);
    digitalWrite(LEFT_MOTOR_PIN2, LOW);
    digitalWrite(RIGHT_MOTOR_PIN1, LOW);
    digitalWrite(RIGHT_MOTOR_PIN2, HIGH);
}

// Hàm đi thẳng
void moveForward(int speed)
{
    Serial.println("Move forward: ");
    Serial.println(speed);
    analogWrite(LEFT_MOTOR_EN, speed);
    analogWrite(RIGHT_MOTOR_EN, speed + 7.65);
    digitalWrite(LEFT_MOTOR_PIN1, HIGH);
    digitalWrite(LEFT_MOTOR_PIN2, LOW);
    digitalWrite(RIGHT_MOTOR_PIN1, HIGH);
    digitalWrite(RIGHT_MOTOR_PIN2, LOW);
}
