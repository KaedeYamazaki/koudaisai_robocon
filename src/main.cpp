/* Include */
#include <Arduino.h>
#include <PS4Controller.h>

/*define*/
#define LEDC_CHANNEL_0 0
#define LEDC_CHANNEL_1 1
#define LEDC_CHANNEL_2 2
#define LEDC_CHANNEL_3 3
#define LEDC_CHANNEL_4 4
#define LEDC_CHANNEL_5 5
#define LEDC_CHANNEL_6 6
#define LEDC_CHANNEL_7 7

#define LEDC_Operating_timeR_BIT 8 // 今回はデューティ―比を8bit（0~255）で表す
#define LEDC_BASE_FREQ 490

static const int M1_pwm = 23;
static const int M2_pwm = 22;
static const int M1_1 = 19;
static const int M1_2 = 18;

static const int M2_1 = 4;
static const int M2_2 = 0;

static const int indicator = 27;

int pwm_DS4_R = 0;
int pwm_DS4_L = 0;

void setup()
{
    Serial.begin(115200);
    Serial.println("start setup");

    // motor setup
    pinMode(M1_pwm, OUTPUT);
    pinMode(M2_pwm, OUTPUT);
    pinMode(M1_1, OUTPUT);
    pinMode(M1_2, OUTPUT);
    pinMode(M2_1, OUTPUT);
    pinMode(M2_2, OUTPUT);
    pinMode(indicator, OUTPUT);

    ledcSetup(LEDC_CHANNEL_0, LEDC_BASE_FREQ, LEDC_Operating_timeR_BIT);
    ledcSetup(LEDC_CHANNEL_1, LEDC_BASE_FREQ, LEDC_Operating_timeR_BIT);
    ledcSetup(LEDC_CHANNEL_2, LEDC_BASE_FREQ, LEDC_Operating_timeR_BIT);
    ledcSetup(LEDC_CHANNEL_3, LEDC_BASE_FREQ, LEDC_Operating_timeR_BIT);
    ledcSetup(LEDC_CHANNEL_4, LEDC_BASE_FREQ, LEDC_Operating_timeR_BIT);
    ledcSetup(LEDC_CHANNEL_5, LEDC_BASE_FREQ, LEDC_Operating_timeR_BIT);
    ledcSetup(LEDC_CHANNEL_6, LEDC_BASE_FREQ, LEDC_Operating_timeR_BIT);
    ledcSetup(LEDC_CHANNEL_7, LEDC_BASE_FREQ, LEDC_Operating_timeR_BIT);

    ledcAttachPin(M1_pwm, LEDC_CHANNEL_0);
    ledcAttachPin(M2_pwm, LEDC_CHANNEL_1);

    // PS4 controller setup
    PS4.begin("90:38:0C:EB:09:F2");
    delay(100);
}

void loop()
{
    if (PS4.isConnected())
    {
        digitalWrite(indicator, HIGH);

        float R = PS4.RStickY();
        float L = PS4.LStickY();

        if (abs(R) <= 15)
            R = 0;
        if (abs(L) <= 15)
            L = 0;

        if (abs(R) > 0 || abs(L) > 0)
        {
            pwm_DS4_R = abs(R) * 255 / 127;
            pwm_DS4_L = abs(L) * 255 / 127;
            if (R <= 0)
            {
                digitalWrite(M1_1, LOW);
                digitalWrite(M1_2, HIGH);
                ledcWrite(LEDC_CHANNEL_0, pwm_DS4_R);
            }
            else
            {
                digitalWrite(M1_1, HIGH);
                digitalWrite(M1_2, LOW);
                ledcWrite(LEDC_CHANNEL_0, pwm_DS4_R);
            }

            if (L <= 0)
            {
                digitalWrite(M2_1, HIGH);
                digitalWrite(M2_2, LOW);
                ledcWrite(LEDC_CHANNEL_1, pwm_DS4_L);
            }
            else
            {
                digitalWrite(M2_1, LOW);
                digitalWrite(M2_2, HIGH);
                ledcWrite(LEDC_CHANNEL_1, pwm_DS4_L);
            }
        }
        else
        {
            pwm_DS4_R = 0;
            pwm_DS4_L = 0;
            ledcWrite(LEDC_CHANNEL_0, 0);
            ledcWrite(LEDC_CHANNEL_1, 0);
            digitalWrite(M1_1, LOW);
            digitalWrite(M1_2, LOW);
        }

        Serial.print("R: ");
        Serial.print(R);
        Serial.print(" L: ");
        Serial.print(L);
        Serial.println(" ");

        Serial.print(" pwm_DS4_R: ");
        Serial.print(pwm_DS4_R);
        Serial.print(" pwm_DS4_L: ");
        Serial.println(pwm_DS4_L);
    }
    else
    {
        Serial.println("PS4 controller is not connected");
        digitalWrite(indicator, LOW);
    }
    delay(10);
}
