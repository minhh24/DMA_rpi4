/*
 * pwm_hybrid_interrupt_polling.c
 *
 * Functionality:
 * - Task 1 (Interrupt): Uses the POSIX Timer API to create an interrupt every 20ms,
 * updating the PWM for a breathing LED effect. CPU efficient.
 * - Task 2 (Polling): Uses mmap to directly read the System Timer register
 * and print it to the screen every 500ms.
 */

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <unistd.h>
#include <time.h>
#include <signal.h>

// --- Keep all #define from the original code ---
#define BCM_PERI_BASE       0xFE000000
#define GPIO_BASE           (BCM_PERI_BASE + 0x200000)
#define PWM_BASE            (BCM_PERI_BASE + 0x20C000)
#define CM_BASE             (BCM_PERI_BASE + 0x101000)
#define SYS_TIMER_BASE      (BCM_PERI_BASE + 0x3000) // This is needed again
#define BLOCK_SIZE          4096
#define GPFSEL1             0x04
#define GPIO18_ALT5         2
#define PWM_CTL             0x00
#define PWM_RNG1            0x10
#define PWM_DAT1            0x14
#define PWM_CTL_PWEN1       (1 << 0)
#define PWM_CTL_MSEN1       (1 << 7)
#define CM_PWMCTL           0xA0
#define CM_PWMDIV           0xA4
#define CM_PASSWORD         0x5A000000
#define SYS_TIMER_CLO       0x04 // This is needed again
// --- End of #define section ---

volatile uint32_t *gpio, *pwm, *systimer; // Add systimer back
int mem_fd;
timer_t timerid;

volatile int duty = 0;
volatile int dir = 1;

// Progress bar drawing function (unchanged)
void draw_bar(int duty_val){
    int width = 20, filled = (duty_val * width) / 100;
    printf("\r[");
    for(int i=0; i<width; i++) printf(i < filled ? "█" : "░");
    printf("] duty=%3d%%", duty_val);
    fflush(stdout);
}

// Cleanup function on exit (unchanged)
void cleanup(int signum){
    printf("\n[CLEANUP] Stopping PWM, deleting timer, and resetting GPIO18...\n");
    if(timerid) timer_delete(timerid);
    if(pwm) pwm[PWM_CTL/4] = 0;
    if(gpio){
        uint32_t sel = gpio[GPFSEL1/4];
        sel &= ~(7 << 24);
        gpio[GPFSEL1/4] = sel;
    }
    if(mem_fd >= 0) close(mem_fd);
    exit(0);
}

// <<< TIMER INTERRUPT HANDLER FOR THE LED >>>
// This function is still called by the Kernel every 20ms, unchanged.
void led_update_handler(int signum) {
    duty += dir;
    if (duty >= 95) dir = -1;
    if (duty <= 5)  dir = 1;

    if (pwm) {
        pwm[PWM_DAT1/4] = duty * 1024 / 100;
        draw_bar(duty);
    }
}

int main(){
    signal(SIGINT, cleanup);

    // --- mmap section (add SYS_TIMER_BASE back) ---
    mem_fd = open("/dev/mem", O_RDWR|O_SYNC);
    if(mem_fd < 0) { perror("open /dev/mem"); return -1; }
    void *gpio_map = mmap(NULL, BLOCK_SIZE, PROT_READ|PROT_WRITE, MAP_SHARED, mem_fd, GPIO_BASE);
    void *pwm_map  = mmap(NULL, BLOCK_SIZE, PROT_READ|PROT_WRITE, MAP_SHARED, mem_fd, PWM_BASE);
    void *clk_map  = mmap(NULL, BLOCK_SIZE, PROT_READ|PROT_WRITE, MAP_SHARED, mem_fd, CM_BASE);
    void *st_map   = mmap(NULL, BLOCK_SIZE, PROT_READ, MAP_SHARED, mem_fd, SYS_TIMER_BASE); // mmap the timer
    if(gpio_map==MAP_FAILED || pwm_map==MAP_FAILED || clk_map==MAP_FAILED || st_map==MAP_FAILED) { perror("mmap failed"); return -1; }

    gpio = (volatile uint32_t *)gpio_map;
    pwm  = (volatile uint32_t *)pwm_map;
    systimer = (volatile uint32_t *)st_map; // assign the systimer pointer
    volatile uint32_t *clk = (volatile uint32_t *)clk_map;

    // --- Hardware Setup (unchanged) ---
    uint32_t sel = gpio[GPFSEL1/4]; sel &= ~(7<<24); sel |= (GPIO18_ALT5 << 24); gpio[GPFSEL1/4] = sel;
    clk[CM_PWMCTL/4] = CM_PASSWORD | (1<<5); usleep(100);
    clk[CM_PWMDIV/4] = CM_PASSWORD | (32<<12);
    clk[CM_PWMCTL/4] = CM_PASSWORD | (1<<4) | 1;
    pwm[PWM_CTL/4] = 0; usleep(10);
    pwm[PWM_RNG1/4] = 1024; pwm[PWM_DAT1/4] = 0;
    pwm[PWM_CTL/4] = PWM_CTL_MSEN1 | PWM_CTL_PWEN1;
    printf("=== PWM Demo (Hybrid: Interrupt + Polling) ===\n");

    // --- Configure POSIX Timer for LED updates (Interrupt-driven) ---
    struct sigevent sev;
    struct itimerspec its;
    signal(SIGRTMIN, led_update_handler);
    sev.sigev_notify = SIGEV_SIGNAL;
    sev.sigev_signo = SIGRTMIN;
    sev.sigev_value.sival_ptr = &timerid;
    timer_create(CLOCK_REALTIME, &sev, &timerid);
    its.it_value.tv_sec = 0;
    its.it_value.tv_nsec = 20000000; // 20ms
    its.it_interval.tv_sec = 0;
    its.it_interval.tv_nsec = 20000000;
    timer_settime(timerid, 0, &its, NULL);
    printf("Interrupt timer for LED has been enabled.\n");

    // --- Main POLLING loop to print the timer value ---
    uint32_t last_print_time = 0;
    printf("Starting polling loop to print timer value...\n");

    while(1) {
        // Read the hardware timer value via mmap
        uint32_t current_time = systimer[SYS_TIMER_CLO/4];

        // Polling: Check if it's time to print yet
        if(current_time - last_print_time > 500000) { // 500ms
            last_print_time = current_time;
            printf(" | Polled SYS_TIMER_CLO=%u\n", current_time);
        }

        // Sleep for 1ms so the loop doesn't take 100% CPU
        usleep(1000);
    }

    cleanup(0);
    return 0;
}
