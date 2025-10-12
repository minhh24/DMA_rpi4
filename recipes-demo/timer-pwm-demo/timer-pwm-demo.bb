SUMMARY = "Timer/PWM Demonstration for Raspberry Pi"
DESCRIPTION = "A program to demonstrate PWM control by creating a 'breathing' LED effect on GPIO18. The CPU sets the duty cycle while the PWM hardware generates the signal."
LICENSE = "MIT"
LIC_FILES_CHKSUM = "file://${COMMON_LICENSE_DIR}/MIT;md5=0835ade698e0bcf8506ecda2f7b4f302"

SRC_URI = "file://timer_pwm_demo.c"

S = "${WORKDIR}"

# Thêm cờ biên dịch "-DRPI4_BUILD" để code C có thể nhận diện
# Đây là cách làm tường minh và an toàn
CFLAGS:append:raspberrypi4 = " -DRPI4_BUILD"
CFLAGS:append:raspberrypi4-64 = " -DRPI4_BUILD"

# do_compile không cần thay đổi, nó sẽ tự động lấy đúng tên file
do_compile() {
    ${CC} ${CFLAGS} ${S}/timer_pwm_demo.c -o ${BPN} ${LDFLAGS}
}

# do_install đã được viết tốt từ file demo DMA, giữ nguyên
do_install() {
    install -d ${D}${bindir}
    install -m 0755 ${B}/${BPN} ${D}${bindir}/
}
