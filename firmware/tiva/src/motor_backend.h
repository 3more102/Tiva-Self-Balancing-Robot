/* Internal implementation included only by main.c. */
static uint32_t g_pwm_period;

static void outputs_init(void)
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM0);
    wait_for_peripheral(SYSCTL_PERIPH_GPIOA);
    wait_for_peripheral(SYSCTL_PERIPH_GPIOB);
    wait_for_peripheral(SYSCTL_PERIPH_GPIOC);
    wait_for_peripheral(SYSCTL_PERIPH_GPIOF);
    wait_for_peripheral(SYSCTL_PERIPH_PWM0);
    GPIOPinTypeGPIOOutput(GPIO_PORTA_BASE, GPIO_PIN_5);
    GPIOPinTypeGPIOOutput(GPIO_PORTC_BASE, GPIO_MOTOR_PINS);
    GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3);
    GPIOPinTypeGPIOInput(GPIO_PORTF_BASE, GPIO_PIN_4);
    GPIOPadConfigSet(GPIO_PORTF_BASE, GPIO_PIN_4, GPIO_STRENGTH_2MA,
                     GPIO_PIN_TYPE_STD_WPU);
    GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_5, 0U);
    GPIOPinWrite(GPIO_PORTC_BASE, GPIO_MOTOR_PINS, 0U);
    SysCtlPWMClockSet(SYSCTL_PWMDIV_8);
    GPIOPinConfigure(GPIO_PB6_M0PWM0);
    GPIOPinConfigure(GPIO_PB7_M0PWM1);
    GPIOPinTypePWM(GPIO_PORTB_BASE, GPIO_PIN_6 | GPIO_PIN_7);
    g_pwm_period = SysCtlClockGet() / 8U / PWM_HZ;
    PWMGenConfigure(PWM0_BASE, PWM_GEN_0,
                    PWM_GEN_MODE_DOWN | PWM_GEN_MODE_NO_SYNC);
    PWMGenPeriodSet(PWM0_BASE, PWM_GEN_0, g_pwm_period);
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_0, 1U);
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_1, 1U);
    PWMOutputState(PWM0_BASE, PWM_OUT_0_BIT | PWM_OUT_1_BIT, true);
    PWMGenEnable(PWM0_BASE, PWM_GEN_0);
}

static void indicator(uint8_t rgb)
{
    GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3,
                 rgb & (GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3));
}

static void motors_off(void)
{
    GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_5, 0U);
    GPIOPinWrite(GPIO_PORTC_BASE, GPIO_MOTOR_PINS, 0U);
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_0, 1U);
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_1, 1U);
}

static void motor_command(float command)
{
#if MOTOR_OUTPUT_ALLOWED
    float left = command * (float)LEFT_MOTOR_POLARITY;
    float right = command * (float)RIGHT_MOTOR_POLARITY;
    uint8_t dir = 0U;
    uint32_t left_width, right_width;
    if (!isfinite(left) || !isfinite(right)) { motors_off(); return; }
    if (left > 0.0f) dir |= GPIO_PIN_4;
    if (left < 0.0f) dir |= GPIO_PIN_5;
    if (right > 0.0f) dir |= GPIO_PIN_6;
    if (right < 0.0f) dir |= GPIO_PIN_7;
    if (left > 1.0f) left = 1.0f;
    if (left < -1.0f) left = -1.0f;
    if (right > 1.0f) right = 1.0f;
    if (right < -1.0f) right = -1.0f;
    left_width = (uint32_t)(fabsf(left) * (float)(g_pwm_period - 1U));
    right_width = (uint32_t)(fabsf(right) * (float)(g_pwm_period - 1U));
    if (left_width == 0U) left_width = 1U;
    if (right_width == 0U) right_width = 1U;
    GPIOPinWrite(GPIO_PORTC_BASE, GPIO_MOTOR_PINS, dir);
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_0, left_width);
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_1, right_width);
    GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_5, GPIO_PIN_5);
#else
    (void)command;
    motors_off();
#endif
}
