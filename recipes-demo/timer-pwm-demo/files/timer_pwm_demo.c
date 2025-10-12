/*
 * pwm_timer_demo.c
 *
 * Function:
 * - Hardware PWM breathing LED on GPIO18 (PWM0, Channel 0)
 * - System Timer readout every 500 ms
 * - Display mapped register addresses + values
 */

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <unistd.h>
#include <time.h>
#include <signal.h>

// IMPORTANT: This is the base physical address for peripherals on the Raspberry Pi's SoC.
// This address (0xFE000000) is for Pi 4. Older models used 0x3F000000 or 0x20000000.
#define BCM_PERI_BASE       0xFE000000

// Offsets for specific peripherals from the base address.
#define GPIO_BASE           (BCM_PERI_BASE + 0x200000)
#define PWM_BASE            (BCM_PERI_BASE + 0x20C000)
#define CM_BASE             (BCM_PERI_BASE + 0x101000) // Clock Manager
#define SYS_TIMER_BASE      (BCM_PERI_BASE + 0x3000)

#define BLOCK_SIZE          4096 // Memory is mapped in 4KB pages.

// GPIO Register offset (GPFSEL1 controls pins 10-19)
#define GPFSEL1             0x04
// The value '2' sets a GPIO pin to its Alternate Function 5 (ALT5).
// For GPIO18, ALT5 is PWM0.
#define GPIO18_ALT5         2

// PWM Register offsets
#define PWM_CTL             0x00 // Control register
#define PWM_RNG1            0x10 // Range (period) for Channel 1
#define PWM_DAT1            0x14 // Data (duty cycle) for Channel 1
#define PWM_CTL_PWEN1       (1 << 0) // Enable bit for Channel 1
#define PWM_CTL_MSEN1       (1 << 7) // Use Mark/Space mode for Channel 1

// Clock Manager Register offsets
#define CM_PWMCTL           0xA0 // PWM Clock Control
#define CM_PWMDIV           0xA4 // PWM Clock Divisor
// A "password" required to write to clock manager registers.
#define CM_PASSWORD         0x5A000000

// System Timer Register offset
#define SYS_TIMER_CLO       0x04  // Counter Lower 32-bits (a free-running microsecond counter)

// Global volatile pointers to our memory-mapped peripheral regions.
// 'volatile' tells the compiler that the value can change at any time,
// preventing optimizations that might break our direct hardware access.
volatile uint32_t *gpio, *pwm, *clk, *systimer;
int mem_fd;

// Signal handler to ensure a clean exit (e.g., on Ctrl+C).
void cleanup(int signum){
    printf("\n[CLEANUP] Stopping PWM and resetting GPIO18...\n");
    // Disable the PWM channel before exiting.
    if(pwm) pwm[PWM_CTL/4] = 0;
    if(gpio){
        // Reset GPIO18 back to a standard input pin.
        uint32_t sel = gpio[GPFSEL1/4];
        sel &= ~(7 << 24);  // Clear the 3 bits for GPIO18
        gpio[GPFSEL1/4] = sel;
    }
    if(mem_fd >= 0) close(mem_fd);
    exit(0);
}

void delay_ms(int ms){
    struct timespec ts = {ms/1000, (ms%1000)*1000000};
    nanosleep(&ts, NULL);
}

void draw_bar(int duty){
    int width = 20, filled = (duty*width)/100;
    printf("\r[");
    for(int i=0;i<width;i++) printf(i<filled?"█":"░");
    printf("] duty=%3d%%", duty);
    fflush(stdout);
}

int main(){
    // Register the cleanup function to handle the SIGINT signal (Ctrl+C).
    signal(SIGINT, cleanup);

    // Open /dev/mem to get access to physical memory.
    // This requires root privileges (run with sudo).
    mem_fd = open("/dev/mem", O_RDWR|O_SYNC);
    if(mem_fd<0){ perror("open /dev/mem"); return -1;}

    // Map the physical peripheral addresses into our process's virtual address space.
    // This allows us to access hardware registers as if they were variables in memory.
    void *gpio_map = mmap(NULL,BLOCK_SIZE,PROT_READ|PROT_WRITE,MAP_SHARED,mem_fd,GPIO_BASE);
    void *pwm_map  = mmap(NULL,BLOCK_SIZE,PROT_READ|PROT_WRITE,MAP_SHARED,mem_fd,PWM_BASE);
    void *clk_map  = mmap(NULL,BLOCK_SIZE,PROT_READ|PROT_WRITE,MAP_SHARED,mem_fd,CM_BASE);
    void *st_map   = mmap(NULL,BLOCK_SIZE,PROT_READ,MAP_SHARED,mem_fd,SYS_TIMER_BASE);

    if(gpio_map==MAP_FAILED || pwm_map==MAP_FAILED || clk_map==MAP_FAILED || st_map==MAP_FAILED){
        perror("mmap failed"); close(mem_fd); return -1;
    }

    // Assign the mapped memory regions to our global pointers.
    gpio = (volatile uint32_t *)gpio_map;
    pwm  = (volatile uint32_t *)pwm_map;
    clk  = (volatile uint32_t *)clk_map;
    systimer = (volatile uint32_t *)st_map;

    printf("=== PWM + System Timer Demo ===\n");
    printf("GPIO18 = PWM0\n");
    // NOTE: Pointer arithmetic is used here. Offsets are in bytes, but the pointer is
    // to a 32-bit (4-byte) integer, so we must divide the offset by 4.
    printf("PWM registers mapped at %p\nPWM_CTL=%p PWM_RNG1=%p PWM_DAT1=%p\n", pwm,
        pwm+PWM_CTL/4, pwm+PWM_RNG1/4, pwm+PWM_DAT1/4);
    printf("Clock registers at %p (CM_PWMCTL=%p CM_PWMDIV=%p)\n", clk, clk+CM_PWMCTL/4, clk+CM_PWMDIV/4);
    printf("System Timer CLO at %p\n", systimer+SYS_TIMER_CLO/4);

    // --- GPIO Setup: Configure GPIO18 to use its Alternate Function 5 (PWM0) ---
    uint32_t sel = gpio[GPFSEL1/4];
    sel &= ~(7<<24);             // Clear the 3 bits for GPIO18 (bits 24-26). '7' is '111' in binary.
    sel |= (GPIO18_ALT5 << 24);  // Set the 3 bits to '010' (value 2), which selects ALT5.
    gpio[GPFSEL1/4] = sel;
    printf("[INFO] GPIO_FSEL1=0x%08X\n", gpio[GPFSEL1/4]);

    // --- Setup PWM Clock ---
    // The PWM peripheral needs a clock source. We configure it here.
    // 1. Stop the PWM clock. A password is required.
    clk[CM_PWMCTL/4] = CM_PASSWORD | (1<<5); // stop clock
    usleep(100);
    // 2. Set the clock divisor. This controls the PWM frequency.
    // Source clock (e.g., 500MHz) / divisor (32) = ~15.6 MHz.
    clk[CM_PWMDIV/4] = CM_PASSWORD | (32<<12);
    // 3. Re-enable the PWM clock, sourcing it from an oscillator.
    clk[CM_PWMCTL/4] = CM_PASSWORD | (1<<4) | 1; // enable + src=osc
    printf("[INFO] CM_PWMCTL=0x%08X CM_PWMDIV=0x%08X\n", clk[CM_PWMCTL/4], clk[CM_PWMDIV/4]);

    // --- Setup PWM Channel ---
    // 1. Ensure the PWM controller is off before changing settings.
    pwm[PWM_CTL/4] = 0; usleep(10);
    // 2. Set the PWM range (period). 1024 steps provide 10-bit resolution.
    // PWM frequency = Clock Freq / Divisor / Range = 15.6MHz / 1024 = ~15 kHz.
    pwm[PWM_RNG1/4] = 1024;
    // 3. Set the initial duty cycle data to 0 (0% duty cycle).
    pwm[PWM_DAT1/4] = 0;
    // 4. Enable the PWM channel in Mark/Space mode.
    pwm[PWM_CTL/4] = PWM_CTL_MSEN1 | PWM_CTL_PWEN1;
    printf("[INFO] PWM_CTL=0x%08X PWM_RNG1=0x%08X\n", pwm[PWM_CTL/4], pwm[PWM_RNG1/4]);

    // --- Main loop ---
    int duty=0, dir=1;
    while(1){
        // This is the core of the breathing effect.
        // We write a new value to the Data register to change the LED brightness.
        pwm[PWM_DAT1/4] = duty*1024/100;
        draw_bar(duty);

        // Print the System Timer value roughly every 500ms.
        static int tick=0;
        if(++tick%25==0){ // 25 loops * 20ms delay = 500ms
            // Read the lower 32 bits of the free-running microsecond counter.
            printf(" | SYS_TIMER_CLO=%u\n", systimer[SYS_TIMER_CLO/4]);
        }

        // Logic to make the duty cycle ramp up and down.
        duty += dir;
        if(duty>=95) dir=-1; // Change direction to down
        if(duty<=5)  dir=1;  // Change direction to up

        delay_ms(20); // Controls the speed of the breathing effect.
    }

    cleanup(0);
    return 0;
}
