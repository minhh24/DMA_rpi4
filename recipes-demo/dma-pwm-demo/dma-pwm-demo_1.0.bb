SUMMARY = "Real Hardware DMA Demonstration for Raspberry Pi"
DESCRIPTION = "A program to demonstrate a real memory-to-memory DMA transfer while the CPU performs another task."
LICENSE = "MIT"
LIC_FILES_CHKSUM = "file://${COMMON_LICENSE_DIR}/MIT;md5=0835ade698e0bcf8506ecda2f7b4f302"

SRC_URI = "file://dma_pwm_demo.c"

S = "${WORKDIR}"

# 1. Thêm cờ biên dịch "-DRPI4_BUILD" cho Pi 4.
CFLAGS:append:raspberrypi4 = " -DRPI4_BUILD"
CFLAGS:append:raspberrypi4-64 = " -DRPI4_BUILD"

# 2. Giữ lại do_compile như file của Pi 3.
# File output sẽ được tạo ra trong thư mục build (${B}).
do_compile() {
    ${CC} ${CFLAGS} ${S}/dma_pwm_demo.c -o ${BPN} ${LDFLAGS}
}

# 3. Sửa lại do_install để chỉ định đường dẫn ĐẦY ĐỦ và RÕ RÀNG.
do_install() {
    install -d ${D}${bindir}
    # Thay vì chỉ dùng "dma-pwm-demo", ta dùng "${B}/${BPN}"
    # ${B} là đường dẫn tới thư mục build
    # ${BPN} là tên file thực thi (dma-pwm-demo)
    # => Cách này đảm bảo lệnh install luôn tìm thấy file.
    install -m 0755 ${B}/${BPN} ${D}${bindir}/
}
