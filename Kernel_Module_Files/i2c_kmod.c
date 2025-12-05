// David Denny, 1001915603
// SoC I2C Kernel Module

//-----------------------------------------------------------------------------
// Hardware Target
//-----------------------------------------------------------------------------

// Target Platform: Xilinx XUP Blackboard

// Hardware configuration:
//
// AXI4-Lite interface
//   Mapped to offset of 0x0x43C20000

// Load kernel module with 'sudo insmod ./i2c_kmod.ko i2c_base_phys=0x43C20000

//-----------------------------------------------------------------------------

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/kobject.h>
#include <linux/sysfs.h>
#include <linux/io.h>
#include <linux/uaccess.h>
#include <linux/slab.h>

//-----------------------------------------------------------------------------
// Kernel module information
//-----------------------------------------------------------------------------

MODULE_LICENSE("GPL");
MODULE_AUTHOR("David Denny");
MODULE_DESCRIPTION("I2C IP Kernel Module");

//-----------------------------------------------------------------------------
// Global Variables and Address Mappings
//-----------------------------------------------------------------------------

#define DRV_NAME "axi_i2c_kmod"

static unsigned long i2c_base_phys = 0x43C20000UL;
module_param(i2c_base_phys, ulong, 0444);
MODULE_PARM_DESC(i2c_base_phys, "Physical base address of I2C AXI peripheral");

// Register Offsets
#define ADDR_OFFSET    0x00UL
#define REG_OFFSET     0x04UL
#define DATA_OFFSET    0x08UL
#define STATUS_OFFSET  0x0CUL
#define CONTROL_OFFSET 0x10UL

// CONTROL Bits
#define CTRL_RW_BIT              0 // Bit 0
#define CTRL_BYTE_COUNT_SHIFT    1 // Bits [4:1]
#define CTRL_USE_REGISTER_BIT    5 // Bit 5
#define CTRL_USE_REPEATED_BIT    6 // Bit 6
#define CTRL_START_BIT           7 // Bit 7
#define CTRL_TEST_OUT_BIT        8 // Bit 8
#define CTRL_IGNORE_ACK_BIT     24 // Bit 24

// STATUS Bits
#define STATUS_RX_OVERFLOW 0
#define STATUS_RX_FULL     1
#define STATUS_RX_EMPTY    2
#define STATUS_TX_OVERFLOW 3
#define STATUS_TX_FULL     4
#define STATUS_TX_EMPTY    5
#define STATUS_ACK_ERROR   6
#define STATUS_BUSY        7

static void __iomem *i2c_base = NULL;
static struct kobject *i2c_kobj;

// Default CONTROL values
static unsigned int cached_rw = 0;
static unsigned int cached_byte_count = 0;
static unsigned int cached_use_register = 0;
static unsigned int cached_use_repeated = 0;
static unsigned int cached_test_out = 1; // Show test_out by default
static unsigned int cached_ignore_ack = 0;

//-----------------------------------------------------------------------------
// Subroutines and Kernel Objects
//-----------------------------------------------------------------------------

// CONTROL Register READ/SHOW
static ssize_t mode_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
    ssize_t len;
    len = scnprintf(buf, PAGE_SIZE, "%s\n", cached_rw ? "read" : "write");
    return len;
}

// CONTROL Register WRITE/STORE
static ssize_t mode_store(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t count)
{
    unsigned long val;
    if (sysfs_streq(buf, "read") || sysfs_streq(buf, "1")) 
    {
        cached_rw = 1;
    }
    else if (sysfs_streq(buf, "write") || sysfs_streq(buf, "0"))
    {
        cached_rw = 0;
    }
    else if (kstrtoul(buf, 0, &val) == 0)
    {
        cached_rw = (val != 0);
    }
    else
    {
        return -EINVAL;
    }
    return count;
}
static struct kobj_attribute mode_attr = __ATTR(mode, 0664, mode_show, mode_store);

// CONTROL Register READ/SHOW
static ssize_t byte_count_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
    ssize_t len;
    len = scnprintf(buf, PAGE_SIZE, "%u\n", cached_byte_count);
    return len;
}

// CONTROL Register WRITE/STORE
static ssize_t byte_count_store(struct kobject *kobject, struct kobj_attribute *attr, const char *buf, size_t count)
{
    unsigned long val;
    if (kstrtoul(buf, 0, &val) != 0) return -EINVAL;
    if (val > 15) return -EINVAL;
    cached_byte_count = val;
    return count;
}
static struct kobj_attribute byte_count_attr = __ATTR(byte_count, 0664, byte_count_show, byte_count_store);

// REGISTER Register READ/SHOW
static ssize_t register_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
    uint32_t v = ioread32(i2c_base + REG_OFFSET);
    return scnprintf(buf, PAGE_SIZE, "0x%02X\n", v & 0xFF);
}

// REGISTER Register WRITE/STORE
static ssize_t register_store(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t count)
{
    unsigned long val;
    if (kstrtoul(buf, 0, &val) != 0) return -EINVAL;
    if (val > 0xFF) return -EINVAL;
    iowrite32((uint32_t)val & 0xFF, i2c_base + REG_OFFSET);
    (void)ioread32(i2c_base + REG_OFFSET);
    return count;
}
static struct kobj_attribute register_attr = __ATTR(register, 0664, register_show, register_store);

// ADDRESS Register READ/SHOW
static ssize_t address_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
    uint32_t v = ioread32(i2c_base + ADDR_OFFSET);
    return scnprintf(buf, PAGE_SIZE, "0x%02X\n", v & 0x7F);
}

// ADDRESS Register WRITE/STORE
static ssize_t address_store(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t count)
{
    unsigned long val;
    if (kstrtoul(buf, 0, &val) != 0) return -EINVAL;
    if (val > 0x7F) return -EINVAL;
    iowrite32((uint32_t)val & 0x7F, i2c_base + ADDR_OFFSET);
    (void)ioread32(i2c_base + ADDR_OFFSET);
    return count;
}
static struct kobj_attribute address_attr = __ATTR(address, 0664, address_show, address_store);

// CONTROL Register READ/SHOW
static ssize_t use_repeated_start_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
    ssize_t len;
    len = scnprintf(buf, PAGE_SIZE, "%s\n", cached_use_repeated ? "true" : "false");
    return len;
}

// CONTROL Register WRITE/STORE
static ssize_t use_repeated_start_store(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t count)
{
    unsigned long val;
    if (sysfs_streq(buf, "true") || sysfs_streq(buf, "1"))
    {
        cached_use_repeated = 1;
    }
    else if (sysfs_streq(buf, "false") || sysfs_streq(buf, "0"))
    {
        cached_use_repeated = 0;
    }
    else if (kstrtoul(buf, 0, &val) == 0)
    {
        cached_use_repeated = (val != 0);
    }
    else
    {
        return -EINVAL;
    }
    return count;
}
static struct kobj_attribute use_repeated_attr = __ATTR(use_repeated_start, 0664, use_repeated_start_show, use_repeated_start_store);

// CONTROL Register READ/SHOW
static ssize_t use_register_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
    ssize_t len;
    len = scnprintf(buf, PAGE_SIZE, "%s\n", cached_use_register ? "true" : "false");
    return len;
}

// CONTROL Register WRITE/STORE
static ssize_t use_register_store(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t count)
{
    unsigned long val;
    if (sysfs_streq(buf, "true") || sysfs_streq(buf, "1"))
    {
        cached_use_register = 1;
    }
    else if (sysfs_streq(buf, "false") || sysfs_streq(buf, "0"))
    {
        cached_use_register = 0;
    }
    else if (kstrtoul(buf, 0, &val) == 0)
    {
        cached_use_register = (val != 0);
    }
    else
    {
        return -EINVAL;
    }
    return count;
}
static struct kobj_attribute use_register_attr = __ATTR(use_register, 0664, use_register_show, use_register_store);


// CONTROL Register READ/SHOW
static ssize_t ignore_ack_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
    ssize_t len;
    len = scnprintf(buf, PAGE_SIZE, "%s\n", cached_ignore_ack ? "true" : "false");
    return len;
}

// CONTROL Register WRITE/STORE
static ssize_t ignore_ack_store(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t count)
{
    unsigned long val;
    if (sysfs_streq(buf, "true") || sysfs_streq(buf, "1"))
    {
        cached_ignore_ack = 1;
    }
    else if (sysfs_streq(buf, "false") || sysfs_streq(buf, "0"))
    {
        cached_ignore_ack = 0;
    }
    else if (kstrtoul(buf, 0, &val) == 0)
    {
        cached_ignore_ack = (val != 0);
    }
    else
    {
        return -EINVAL;
    }
    return count;
}
static struct kobj_attribute ignore_ack_attr = __ATTR(ignore_ack, 0664, ignore_ack_show, ignore_ack_store);

// DATA Register WRITE/STORE
//   Increments the TX FIFO's wr_index
static ssize_t tx_data_store(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t count)
{
    unsigned long val;
    if (kstrtoul(buf, 0, &val) != 0) return -EINVAL;
    if (val > 0xFF) return -EINVAL;
    iowrite32((uint32_t)val & 0xFF, i2c_base + DATA_OFFSET);
    (void)ioread32(i2c_base + DATA_OFFSET);
    return count;
}
static struct kobj_attribute tx_data_attr = __ATTR(tx_data, 0664, NULL, tx_data_store);

// DATA Register READ/SHOW
//   Increments the RX FIFO's rd_index
static ssize_t rx_data_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
    uint32_t status, v;
    status = ioread32(i2c_base + STATUS_OFFSET);
    if (status & (1u << STATUS_RX_EMPTY)) {
        return scnprintf(buf, PAGE_SIZE, "-1\n");
    }
    v = ioread32(i2c_base + DATA_OFFSET) & 0xFF;
    return scnprintf(buf, PAGE_SIZE, "0x%02X\n", v);
}
static struct kobj_attribute rx_data_attr = __ATTR(rx_data, 0444, rx_data_show, NULL);

// CONTROL Register Start READ/SHOW
static ssize_t start_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
    uint32_t status = ioread32(i2c_base + STATUS_OFFSET);
    return scnprintf(buf, PAGE_SIZE, "%u\n", (status >> STATUS_BUSY) & 0x1);
}

// CONTROL Register Start WRITE/STORE
static ssize_t start_store(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t count)
{
    uint32_t ctrl_with_start;
    uint32_t ctrl_no_start = 0;
    ctrl_no_start |= (cached_rw & 0x1) << CTRL_RW_BIT;
    ctrl_no_start |= (cached_byte_count & 0xF) << CTRL_BYTE_COUNT_SHIFT;
    ctrl_no_start |= (cached_use_register & 0x1) << CTRL_USE_REGISTER_BIT;
    ctrl_no_start |= (cached_use_repeated & 0x1) << CTRL_USE_REPEATED_BIT;
    ctrl_no_start |= (cached_test_out & 0x1) << CTRL_TEST_OUT_BIT;
    ctrl_no_start |= (cached_ignore_ack & 0x1) << CTRL_IGNORE_ACK_BIT;
    ctrl_with_start = ctrl_no_start | (1u << CTRL_START_BIT);
    iowrite32(ctrl_no_start, i2c_base + CONTROL_OFFSET);
    (void)ioread32(i2c_base + CONTROL_OFFSET);
    iowrite32(ctrl_with_start, i2c_base + CONTROL_OFFSET);
    (void)ioread32(i2c_base + CONTROL_OFFSET);
    return count;
}
static struct kobj_attribute start_attr = __ATTR(start, 0664, start_show, start_store);

// STATUS Register READ/SHOW
static ssize_t status_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
    uint32_t status = ioread32(i2c_base + STATUS_OFFSET);
    return scnprintf(buf, PAGE_SIZE, "0x%08X\n", status);
}
static struct kobj_attribute status_attr = __ATTR(status, 0444, status_show, NULL);

// Attributes

static struct attribute *i2c_attrs[] = {&mode_attr.attr, &byte_count_attr.attr, &register_attr.attr, &address_attr.attr, &use_repeated_attr.attr, &use_register_attr.attr, &ignore_ack_attr.attr, &tx_data_attr.attr, &rx_data_attr.attr, &start_attr.attr, &status_attr.attr, NULL};

static struct attribute_group attr_group =
{
    .attrs = i2c_attrs
};

//-----------------------------------------------------------------------------
// Initialization and Exit
//-----------------------------------------------------------------------------

static int __init i2c_kmod_init(void)
{
    int result;
    printk(KERN_INFO "%s: starting\n", DRV_NAME);

    /* Create i2c directory under /sys/kernel */
    i2c_kobj = kobject_create_and_add("i2c", kernel_kobj);
    if (!i2c_kobj) {
        printk(KERN_ALERT "I2C Driver: failed to create and add kobj\n");
        return -ENOENT;
    }

    /* Create attribute group */
    result = sysfs_create_group(i2c_kobj, &attr_group);
    if (result != 0)
        return result;

    /* Physical to virtual memory map to access I2C registers */
    i2c_base = ioremap(i2c_base_phys, 0x1000UL);
    if (i2c_base == NULL)
        return -ENODEV;

    /* init cached fields (defaults) */
    cached_rw = 0;
    cached_byte_count = 0;
    cached_use_register = 0;
    cached_use_repeated = 0;
    cached_test_out = 1; // Show test_out by default
    cached_ignore_ack = 0;

    printk(KERN_INFO "%s: I2C Driver: initialized\n", DRV_NAME);
    return 0;
}

static void __exit i2c_kmod_exit(void)
{
    pr_info("%s: exit\n", DRV_NAME);
    sysfs_remove_group(i2c_kobj, &attr_group);
    kobject_put(i2c_kobj);
    if (i2c_base) iounmap(i2c_base);
}

module_init(i2c_kmod_init);
module_exit(i2c_kmod_exit);

