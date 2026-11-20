/* Internal non-blocking HC-SR04 backend; included only by main.c.
 * PD2 = TRIG, PD3 = ECHO. ECHO must be level-shifted to 3.3 V. */
#define ULTRASONIC_TRIGGER_PIN GPIO_PIN_2
#define ULTRASONIC_ECHO_PIN GPIO_PIN_3
static volatile uint32_t g_echo_rise_ticks;
static volatile uint32_t g_echo_width_ticks;
static volatile bool g_echo_waiting_for_fall;
static volatile bool g_echo_waiting_for_echo;
static volatile bool g_echo_result_ready;
static volatile bool g_echo_result_valid;
static uint32_t g_ultrasonic_clock_hz;
static uint32_t g_ultrasonic_last_trigger_ticks;
static uint32_t g_ultrasonic_min_echo_ticks;
static uint32_t g_ultrasonic_max_echo_ticks;
static uint32_t g_ultrasonic_timeout_ticks;
static uint32_t g_ultrasonic_interval_ticks;
static bool g_ultrasonic_initialized;
static void ultrasonic_echo_isr(void)
{
    const uint32_t status = GPIOIntStatus(GPIO_PORTD_BASE, true);
    GPIOIntClear(GPIO_PORTD_BASE, status);
    if ((status & ULTRASONIC_ECHO_PIN) != 0U) {
        const bool pin_high = (GPIOPinRead(GPIO_PORTD_BASE, ULTRASONIC_ECHO_PIN) != 0U);
        const uint32_t now = TimerValueGet(TIMER1_BASE, TIMER_A);
        if (pin_high) {
            if (g_echo_waiting_for_echo && !g_echo_waiting_for_fall) {
                g_echo_rise_ticks = now;
                g_echo_waiting_for_fall = true;
            }
        } else if (g_echo_waiting_for_echo && g_echo_waiting_for_fall) {
            g_echo_width_ticks = g_echo_rise_ticks - now;
            g_echo_result_valid =
                g_echo_width_ticks >= g_ultrasonic_min_echo_ticks &&
                g_echo_width_ticks <= g_ultrasonic_max_echo_ticks;
            g_echo_waiting_for_fall = false;
            g_echo_waiting_for_echo = false;
            g_echo_result_ready = true;
        }
    }
}
static bool ultrasonic_init(void)
{
    g_ultrasonic_clock_hz = SysCtlClockGet();
    if (g_ultrasonic_clock_hz < 1000000U) return false;
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
    wait_for_peripheral(SYSCTL_PERIPH_GPIOD);
    GPIOPinTypeGPIOOutput(GPIO_PORTD_BASE, ULTRASONIC_TRIGGER_PIN);
    GPIOPinTypeGPIOInput(GPIO_PORTD_BASE, ULTRASONIC_ECHO_PIN);
    GPIOPadConfigSet(GPIO_PORTD_BASE, ULTRASONIC_ECHO_PIN,
                     GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD);
    GPIOPinWrite(GPIO_PORTD_BASE, ULTRASONIC_TRIGGER_PIN, 0U);
    g_ultrasonic_min_echo_ticks = (g_ultrasonic_clock_hz / 1000000U) * 117U;
    g_ultrasonic_max_echo_ticks = (g_ultrasonic_clock_hz / 1000000U) * 17500U;
    g_ultrasonic_timeout_ticks = (g_ultrasonic_clock_hz / 1000000U) * 25000U;
    g_ultrasonic_interval_ticks = (g_ultrasonic_clock_hz / 1000000U) * 60000U;
    g_echo_rise_ticks = 0U;
    g_echo_width_ticks = 0U;
    g_echo_waiting_for_fall = false;
    g_echo_waiting_for_echo = false;
    g_echo_result_ready = false;
    g_echo_result_valid = false;
    g_ultrasonic_last_trigger_ticks = TimerValueGet(TIMER1_BASE, TIMER_A);
    GPIOIntDisable(GPIO_PORTD_BASE, ULTRASONIC_ECHO_PIN);
    GPIOIntClear(GPIO_PORTD_BASE, ULTRASONIC_ECHO_PIN);
    GPIOIntTypeSet(GPIO_PORTD_BASE, ULTRASONIC_ECHO_PIN, GPIO_BOTH_EDGES);
    GPIOIntRegister(GPIO_PORTD_BASE, ultrasonic_echo_isr);
    GPIOIntEnable(GPIO_PORTD_BASE, ULTRASONIC_ECHO_PIN);
    g_ultrasonic_initialized = true;
    return true;
}
static void ultrasonic_trigger(uint32_t now_ticks)
{
    (void)now_ticks;
    GPIOIntDisable(GPIO_PORTD_BASE, ULTRASONIC_ECHO_PIN);
    GPIOIntClear(GPIO_PORTD_BASE, ULTRASONIC_ECHO_PIN);
    g_echo_waiting_for_fall = false;
    g_echo_waiting_for_echo = true;
    GPIOIntEnable(GPIO_PORTD_BASE, ULTRASONIC_ECHO_PIN);
    GPIOPinWrite(GPIO_PORTD_BASE, ULTRASONIC_TRIGGER_PIN, 0U);
    GPIOPinWrite(GPIO_PORTD_BASE, ULTRASONIC_TRIGGER_PIN, ULTRASONIC_TRIGGER_PIN);
    SysCtlDelay(g_ultrasonic_clock_hz / 300000U); /* approximately 10 us */
    GPIOPinWrite(GPIO_PORTD_BASE, ULTRASONIC_TRIGGER_PIN, 0U);
    g_ultrasonic_last_trigger_ticks = TimerValueGet(TIMER1_BASE, TIMER_A);
}
static bool ultrasonic_service(uint32_t now_ticks, float *distance_cm,
                               bool *sample_valid)
{
    uint32_t width_ticks = 0U;
    bool result_valid = false;
    bool result_ready = false;
    if (!g_ultrasonic_initialized || distance_cm == 0 || sample_valid == 0) return false;
    if (g_echo_result_ready) {
        GPIOIntDisable(GPIO_PORTD_BASE, ULTRASONIC_ECHO_PIN);
        if (g_echo_result_ready) {
            width_ticks = g_echo_width_ticks;
            result_valid = g_echo_result_valid;
            g_echo_result_ready = false;
            result_ready = true;
        }
        GPIOIntEnable(GPIO_PORTD_BASE, ULTRASONIC_ECHO_PIN);
    }
    if (!result_ready && g_echo_waiting_for_echo &&
        (g_ultrasonic_last_trigger_ticks - now_ticks) > g_ultrasonic_timeout_ticks) {
        GPIOIntDisable(GPIO_PORTD_BASE, ULTRASONIC_ECHO_PIN);
        if (g_echo_waiting_for_echo &&
            (g_ultrasonic_last_trigger_ticks - now_ticks) > g_ultrasonic_timeout_ticks) {
            g_echo_waiting_for_echo = false;
            g_echo_waiting_for_fall = false;
            g_echo_result_ready = false;
            result_ready = true;
            result_valid = false;
        }
        GPIOIntEnable(GPIO_PORTD_BASE, ULTRASONIC_ECHO_PIN);
    }
    if (result_ready) {
        *sample_valid = result_valid;
        if (result_valid) {
            *distance_cm = ((float)width_ticks * 34300.0f) /
                           (2.0f * (float)g_ultrasonic_clock_hz);
        } else {
            *distance_cm = NAN;
        }
    }
    if (!g_echo_waiting_for_echo && !g_echo_waiting_for_fall &&
        !g_echo_result_ready &&
        (g_ultrasonic_last_trigger_ticks - now_ticks) >= g_ultrasonic_interval_ticks) {
        ultrasonic_trigger(now_ticks);
    }
    return result_ready;
}
