// David Denny, 1001915603
// Simple Read Example for I2C Peripheral

#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <unistd.h>
#include <stdint.h>
#include <errno.h>
#include <time.h>

#ifndef I2C_BASE_ADDR
#define I2C_BASE_ADDR  0x43C20000UL
#endif
#define I2C_RANGE      0x1000UL

// Register offsets
#define ADDR_OFFSET    0x00UL
#define REG_OFFSET     0x04UL
#define DATA_OFFSET    0x08UL
#define STATUS_OFFSET  0x0CUL
#define CONTROL_OFFSET 0x10UL

// CONTROL Bit Positions
#define CTRL_RW_BIT              0       // Bit  0:     R/~W (1 = read, 0 = write)
#define CTRL_BYTE_COUNT_SHIFT    1       // Bits [4:1]: Byte Count
#define CTRL_USE_REGISTER_BIT    5       // Bit  5:     Use Register
#define CTRL_USE_REPEATED_BIT    6       // Bit  6:     Use Repeated Start
#define CTRL_START_BIT           7       // Bit  7:     Start
#define CTRL_TEST_OUT_BIT        8       // Bit  8:     Test Out
#define CTRL_IGNORE_ACK_BIT     24       // Bit  24:    Ignore ACK

// Helper wrappers for register access
static inline void mmio_write32(volatile uint32_t *base, uint32_t byte_offset, uint32_t val)
{
    base[byte_offset / 4] = val;
    // read-back ensures posting in some buses
    (void)base[byte_offset / 4];
}

static inline uint32_t mmio_read32(volatile uint32_t *base, uint32_t byte_offset)
{
    return base[byte_offset / 4];
}

int main(int argc, char **argv)
{
    off_t base_addr = (off_t)I2C_BASE_ADDR;
    if (argc > 1)
    {
        base_addr = (off_t)strtoul(argv[1], NULL, 0);
    }

    int fd = open("/dev/mem", O_RDWR | O_SYNC);
    if (fd < 0)
    {
        perror("open(/dev/mem)");
        return 1;
    }

    volatile uint32_t *i2c_ptr = (volatile uint32_t *) mmap(NULL, I2C_RANGE,
        PROT_READ | PROT_WRITE, MAP_SHARED, fd, base_addr);

    if (i2c_ptr == MAP_FAILED)
    {
        perror("mmap");
        close(fd);
        return 1;
    }

    printf("Using I2C base at 0x%08lX\n", (unsigned long)base_addr);

    // Transaction parameters
    uint32_t addr_val     = 0x00000020; // Address
    uint32_t byte_count   = 0;          // Byte Count (0 = 1 byte for this implementation)
    uint32_t rw_is_read   = 1;          // R/W
    uint32_t use_register = 0;          // Use Register
    uint32_t use_repeated = 0;          // Use Repeated Start
    uint32_t test_out_en  = 1;          // Test Out
    uint32_t ignore_ack   = 1;          // Ignore ACK

    // Building Control Register
    uint32_t control_val = 0;
    uint32_t control_start;
    control_val |= ((rw_is_read & 0x1)   << CTRL_RW_BIT);
    control_val |= ((byte_count & 0xF)   << CTRL_BYTE_COUNT_SHIFT);
    control_val |= ((use_register & 0x1) << CTRL_USE_REGISTER_BIT);
    control_val |= ((use_repeated & 0x1) << CTRL_USE_REPEATED_BIT);
    control_val |= ((test_out_en & 0x1)  << CTRL_TEST_OUT_BIT);
    control_val |= ((ignore_ack & 0x1)   << CTRL_IGNORE_ACK_BIT);
    control_start = control_val | (1u << CTRL_START_BIT);

    // Step 1: Write to ADDRESS Register
    printf("Writing ADDRESS = 0x%08X\n", addr_val);
    mmio_write32(i2c_ptr, ADDR_OFFSET, addr_val);
    usleep(1000);

    // Step 2: Write to CONTROL Register
    printf("Writing CONTROL (config only) = 0x%08X\n", control_val);
    mmio_write32(i2c_ptr, CONTROL_OFFSET, control_val);
    usleep(1000);

    // Step 3: Trigger the FSM Start
    printf("Triggering START\n");
    mmio_write32(i2c_ptr, CONTROL_OFFSET, control_start);
    usleep(1000);
    
    // Show the results of the byte read
    printf("Byte Read    = 0x%08X\n", mmio_read32(i2c_ptr, DATA_OFFSET));

    // Show final status bits (RX/TX FIFO indicators etc.)
    printf("Final STATUS = 0x%08X\n", mmio_read32(i2c_ptr, STATUS_OFFSET));

    munmap((void *)i2c_ptr, I2C_RANGE);
    close(fd);
    return 0;
}

