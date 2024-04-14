#include <Arduino.h>

using namespace std;

const byte LEFT_MOTOR_PIN1 = 7;
const byte LEFT_MOTOR_PIN2 = 8;
const byte RIGHT_MOTOR_PIN1 = 9;
const byte RIGHT_MOTOR_PIN2 = 10;
const byte LEFT_MOTOR_EN = 5;
const byte RIGHT_MOTOR_EN = 6;

// Định nghĩa các chân kết nối với cảm biến
const int sensorPins[] = {A0, A1, A2, A3, A4};
const int MAX_SPEED = 160;
const int MIN_SPEED = 80;
const int DEFAULT_SPEED = 180;

const int MAX_LENGTH = 10;
String queue[MAX_LENGTH];

int caclulatePID(int error);
int calculeError();
bool checkline();
bool checkStop();
void stopRobot();
void moveForward(int speed);
void turnRight(int speed);
void turnLeft(int speed);
void solve(int pid);
void moveBackward(int speed);
String sensorToString();
bool checkDan();
void addQueue();

float p = 0;
float i = 0;
float d = 0;
bool isRun = true;
int lastError = 0;
int index = 0;

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
    if (!isRun)
        return;
    // in giá trị cảm biến
    readSensors();

    addQueue();

    int error = calculeError();
    int pid = caclulatePID(error);
    solve(pid);

    delay(50);
    // Serial.println("====================================");
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
    float values[5] = {-4, -2.5, 0, 2.5, 4};
    int error = 0;
    for (int i = 0; i < numSensors; i++)
    {
        error += values[i] * sensorValues[i];
    }
    return error;
}

int caclulatePID(int error)
{
    float kp = 5;
    float ki = 0.0001;
    float kd = 10;

    p = error;
    i = i + error;
    d = (error - lastError);
    lastError = error;
    return kp * p + ki * i + kd * d;
}

void solve(int pid)
{
    int speed = abs(pid) * 5;
    speed = (speed == 0) ? DEFAULT_SPEED : speed;
    speed = (speed < MIN_SPEED) ? MIN_SPEED : speed;
    // speed = constrain(speed, MIN_SPEED, MAX_SPEED);

    if (checkline() == false || checkDan())
    {
        moveBackward(speed);
        return;
    }
    if (checkStop() == true)
    {
        stopRobot();
        isRun = false;
        return;
    }
    //  if(speed == 80){
    //   speed +=20;
    //  }
    if (pid > 0)
        turnLeft(speed);
    else if (pid < 0)
        turnRight(speed);
    else
        moveForward(speed);
}
bool checkline()
{
    bool isLine = false;
    for (int i = 0; i < numSensors; i++)
    {
        if (sensorValues[i] == 1)
        {
            isLine = true;
            break;
        }
    }
    return isLine;
}

bool checkStop()
{
    for (int i = 0; i < numSensors; i++)
    {
        if (sensorValues[i] != 1)
            return false;
    }
    return true;
}

// Hàm điều khiển động cơ với tốc độ có thể điều chỉnh
// Hàm đi lùi
void moveBackward(int speed)
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
    analogWrite(LEFT_MOTOR_EN, speed);
    analogWrite(RIGHT_MOTOR_EN, speed / 3);
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
    analogWrite(LEFT_MOTOR_EN, speed / 3);
    analogWrite(RIGHT_MOTOR_EN, speed);
    digitalWrite(LEFT_MOTOR_PIN1, LOW);
    digitalWrite(LEFT_MOTOR_PIN2, HIGH);
    digitalWrite(RIGHT_MOTOR_PIN1, HIGH);
    digitalWrite(RIGHT_MOTOR_PIN2, LOW);
}

// Hàm đi thẳng
void moveForward(int speed)
{
    Serial.println("Move forward: ");
    Serial.println(speed);
    analogWrite(LEFT_MOTOR_EN, speed);
    analogWrite(RIGHT_MOTOR_EN, speed);
    digitalWrite(LEFT_MOTOR_PIN1, HIGH);
    digitalWrite(LEFT_MOTOR_PIN2, LOW);
    digitalWrite(RIGHT_MOTOR_PIN1, HIGH);
    digitalWrite(RIGHT_MOTOR_PIN2, LOW);
}

void stopRobot()
{
    Serial.println("Stop: ");
    analogWrite(LEFT_MOTOR_EN, 0);       // Dừng motor bên trái
    analogWrite(RIGHT_MOTOR_EN, 0);      // Dừng motor bên phải
    digitalWrite(LEFT_MOTOR_PIN1, LOW);  // Chỉ định một chiều quay của motor bên trái
    digitalWrite(LEFT_MOTOR_PIN2, LOW);  // Chỉ định một chiều quay của motor bên trái
    digitalWrite(RIGHT_MOTOR_PIN1, LOW); // Chỉ định một chiều quay của motor bên phải
    digitalWrite(RIGHT_MOTOR_PIN2, LOW); // Chỉ định một chiều quay của motor bên phải
}

String sensorToString()
{
    String result = "";
    for (int i = 0; i < numSensors; i++)
    {
        result = result + char(sensorValues[i] + 48);
    }
    return result;
}

bool checkDan()
{
    if (index < MAX_LENGTH)
        return false;
    for (int i = 0; i < MAX_LENGTH - 1; i++)
    {
        if (queue[i] != queue[i + 1])
        {
            return false;
        }
    }
    return queue[0] != "00100";
}

void addQueue()
{
    if (index >= MAX_LENGTH)
    {
        for (int i = 0; i < index - 1; i++)
        {
            queue[i] = queue[i + 1];
        }
        index--;
    }
    queue[index++] = sensorToString();
    // for (int i = 0; i < index; i++) {
    //   Serial.println(queue[i]);
    // }
    Serial.println(checkDan() ? "Stop" : "Not stop");
}