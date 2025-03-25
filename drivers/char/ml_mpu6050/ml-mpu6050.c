#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/cdev.h>
#include <linux/smp.h>

#define MPU6050_DEVICE_NAME "ml-mpu6050"

#define MPU6050_ADDR 0x68
#define SMPLRT_DIV 0x19
#define CONFIG 0x1A
#define ACCEL_CONFIG 0x1C
#define FIFO_EN 0x23
// #define GYRO_XOUT_H 0x43
// #define GYRO_XOUT_L 0x44
// #define GYRO_YOUT_H 0x45
// #define GYRO_YOUT_L 0x46
// #define GYRO_ZOUT_H 0x47
// #define GYRO_ZOUT_L 0x48
#define ACCEL_XOUT_H 0x3B
#define ACCEL_XOUT_L 0x3C
#define ACCEL_YOUT_H 0x3D
#define ACCEL_YOUT_L 0x3E
#define ACCEL_ZOUT_H 0x3F
#define ACCEL_ZOUT_L 0x40
#define USER_CTRL 0x6A
#define PWR_MGMT_1 0x6B
#define PWR_MGMT_2 0x6C
#define FIFO_COUNTH 0x72
#define FIFO_COUNTL 0x73
#define FIFO_R_W 0x73

// PMCR: Performance Monitors Control Register
#define ARMV8_PMCR_MASK 0x3f
#define ARMV8_PMCR_E (1 << 0) // E, bit [0] -> 0b1 [Enable all event counters, including PMCCNTR_EL0]
#define ARMV8_PMCR_P (1 << 1) // P, bit [1] -> 0b1 [Reset all event counters except PMCCNTR_EL0]
#define ARMV8_PMCR_C (1 << 2) // C, bit [2] -> 0b1 [Reset PMCCNTR_EL0 counter]

// PMUSERENR: Performance Monitors USER ENable Register
#define ARMV8_PMUSERENR_EN (1 << 0) // EN, bit [0] -> 0b1 [Traps access enable]
#define ARMV8_PMUSERENR_CR (1 << 2) // CR, bit [2] -> 0b1 [Cycle counter read access enable]
#define ARMV8_PMUSERENR_ER (1 << 3) // ER, bit [3] -> 0b1 [Event counter read access enable]

// PMINTENSET: Performance Monitors INTerrupt ENable SET register
#define ARMV8_PMINTENSET_DISABLE (0 << 31) // C, bit[31] -> 0b0 [cycle counter overflow interrupt disabled]

// PMCNTENSET: Performance Monitors CouNT ENable SET register
#define ARMV8_PMCNTENSET_ENABLE (1 << 31)  // C, bit[31] -> 0b1 [cycle counter enabled]
#define ARMV8_PMCNTENSET_DISABLE (0 << 31) // C, bit[31] -> 0b0 [cycle counter disabled]

static struct i2c_client *mpu6050_client;
static dev_t mpu6050_dev_num;
static struct cdev *mpu6050_cdev;
static struct class *mpu6050_class;

static void pmu_pmcr_write(u32 value)
{
    value &= ARMV8_PMCR_MASK;
    asm volatile("isb" : : : "memory");
    asm volatile("MSR PMCR_EL0, %0" : : "r"((u64)value));
}

static u32 pmu_pmccntr_read(void)
{
    u32 value;
    // PMCCNTR: Performance Monitors Cycle CouNT Register
    // Read the cycle counter
    asm volatile("MRS %0, PMCCNTR_EL0" : "=r"(value));
    return value;
}

static void enable_cpu_counter(void)
{
    // Enable PMU user read access
    asm volatile("MSR PMUSERENR_EL0, %0" : : "r"((u64)ARMV8_PMUSERENR_EN | ARMV8_PMUSERENR_ER | ARMV8_PMUSERENR_CR));
    // Init & reset PMU control
    pmu_pmcr_write(ARMV8_PMCR_P | ARMV8_PMCR_C);
    // Disable PMU cycle counter overflow interrupt
    asm volatile("MSR PMINTENSET_EL1, %0" : : "r"((u64)ARMV8_PMINTENSET_DISABLE));
    // Enable PMU cycle counter
    asm volatile("MSR PMCNTENSET_el0, %0" : : "r"((u64)ARMV8_PMCNTENSET_ENABLE));
    // Enable PMU control
    pmu_pmcr_write(ARMV8_PMCR_E);

    printk(KERN_INFO "[MPU6050] PMU access enabled.");
}

static void disable_cpu_counter(void)
{
    // Disable PMU cycle counter
    asm volatile("MSR PMCNTENSET_EL0, %0" : : "r"((u64)ARMV8_PMCNTENSET_DISABLE));
    // Disable PMU control
    pmu_pmcr_write(~ARMV8_PMCR_E); // '~'=not
    // Disable PMU user read access
    asm volatile("MSR PMUSERENR_EL0, %0" : : "r"((u64)0)); // all set 0

    printk(KERN_INFO "PMU access disabled.");
}

static int i2c_write_reg(struct i2c_client *client, u8 address, u8 data)
{
	int ret = 0;
	u8 buf[2];

	buf[0] = address;
	buf[1] = data;

	struct i2c_msg msg = {
		.addr = client->addr,
		.flags = 0,
		.buf = buf,
		.len = 2
	};

	ret = i2c_transfer(client->adapter, &msg, 1);
	if (ret != 1) {
		printk(KERN_ERR "[MPU6050] Failed to write\n");
		return -1;
	}
	return 0;
}

static int i2c_read_reg(struct i2c_client *client, u8 address, void *data, u32 length){
	int ret = 0;
	struct i2c_msg msgs[] = {
		{ .addr = client->addr,
		  .flags = 0, //write
		  .len = 1,
		  .buf = &address},
		{ .addr = client->addr,
		  .flags = 1, //read
		  .buf = data,
		  .len = length },
	};
	
	ret = i2c_transfer(mpu6050_client->adapter, msgs, 2);
	if (ret != 2) {
		printk(KERN_ERR "[MPU6050] Failed to read\n");
		return -1;
	}
	return 0;
}

static ssize_t mpu6050_read(struct file *file, char __user *buf, size_t count, loff_t *offset)
{
    char data_H;
    char data_L;
	short fifo_count;
	unsigned char buffer[12];
    short mpu6050_result[3];
    u32 cycles_before, cycles_after;

    // Save current cpu cycle
    cycles_before = pmu_pmccntr_read(); 

	i2c_read_reg(mpu6050_client, FIFO_COUNTH, &data_H, 1);
    i2c_read_reg(mpu6050_client, FIFO_COUNTL, &data_L, 1);
    fifo_count = (data_H << 8) | data_L;

	if (fifo_count >= 6){
    	i2c_read_reg(mpu6050_client, FIFO_R_W, &buffer, 6);
		mpu6050_result[0] = (buffer[0] << 8) | buffer[1];
		mpu6050_result[1]  = (buffer[2] << 8) | buffer[3];
		mpu6050_result[2] = (buffer[4] << 8) | buffer[5];
	 	i2c_write_reg(mpu6050_client, USER_CTRL, 0x04); // reset FIFO
	}else{
		return 0;
	}

    // i2c_read_reg(mpu6050_client, ACCEL_XOUT_H, &data_H, 1);
    // i2c_read_reg(mpu6050_client, ACCEL_XOUT_L, &data_L, 1);
    // mpu6050_result[0] = data_H << 8 | data_L;

    // i2c_read_reg(mpu6050_client, ACCEL_YOUT_H, &data_H, 1);
    // i2c_read_reg(mpu6050_client, ACCEL_YOUT_L, &data_L, 1);
    // mpu6050_result[1] = data_H << 8 | data_L;

    // i2c_read_reg(mpu6050_client, ACCEL_ZOUT_H, &data_H, 1);
    // i2c_read_reg(mpu6050_client, ACCEL_ZOUT_L, &data_L, 1);
    // mpu6050_result[2] = data_H << 8 | data_L;

    // i2c_read_reg(mpu6050_client, GYRO_XOUT_H, &data_H, 1);
    // i2c_read_reg(mpu6050_client, GYRO_XOUT_L, &data_L, 1);
    // mpu6050_result[3] = data_H << 8 | data_L;

    // i2c_read_reg(mpu6050_client, GYRO_YOUT_H, &data_H, 1);
    // i2c_read_reg(mpu6050_client, GYRO_YOUT_L, &data_L, 1);
    // mpu6050_result[4] = data_H << 8 | data_L;

    // i2c_read_reg(mpu6050_client, GYRO_ZOUT_H, &data_H, 1);
    // i2c_read_reg(mpu6050_client, GYRO_ZOUT_L, &data_L, 1);
    // mpu6050_result[5] = data_H << 8 | data_L;

    int error;

    error = copy_to_user(buf, mpu6050_result, count);

    if(error != 0)
    {
		printk("[MPU6050] Failed to copy_to_user");
		return -1;
    }

    // Save current cpu cycle(after run function)
    cycles_after = pmu_pmccntr_read();

    printk(KERN_INFO "[MPU6050] PMU Test: read operation CPU cycle count: %u\n", cycles_after - cycles_before);

    return 0;
}

static const struct file_operations mpu6050_fops = {
	.owner = THIS_MODULE,
	.read = mpu6050_read,
};

static int mpu6050_probe(struct i2c_client *client)
{
	mpu6050_client = client;
	
	int err = 0; // if err, return -1

	err += i2c_write_reg(mpu6050_client, PWR_MGMT_1, 0x00);
	err += i2c_write_reg(mpu6050_client, SMPLRT_DIV, 0x00);
	err += i2c_write_reg(mpu6050_client, CONFIG, 0x00);
	err += i2c_write_reg(mpu6050_client, ACCEL_CONFIG, 0x03); // 0: +-2g, 1: +-4g, 2: +-8g, 3: +-16g
	err += i2c_write_reg(mpu6050_client, USER_CTRL, 0x04); // reset FIFO
	err += i2c_write_reg(mpu6050_client, FIFO_EN, 0x03); // enable accel FIFO
	err += i2c_write_reg(mpu6050_client, USER_CTRL, 0x40); // enable FIFO

	if (err < 0) {
		printk(KERN_ERR "[MPU6050] Initialization failed\n");
		return -1;
	}

	// Allocate major and minor numbers
	if (alloc_chrdev_region(&mpu6050_dev_num, 0, 1, MPU6050_DEVICE_NAME)) {
		printk(KERN_ERR "[MPU6050] Failed to allocate chrdev region\n");
		return -1;
	}

	// Initialize the cdev structure and add it to the kernel
	mpu6050_cdev = cdev_alloc();
	cdev_init(mpu6050_cdev, &mpu6050_fops);
	mpu6050_cdev->owner = THIS_MODULE;
	if (cdev_add(mpu6050_cdev, mpu6050_dev_num, 1)) {
		printk(KERN_ERR "[MPU6050] Failed to add cdev\n");
		unregister_chrdev_region(mpu6050_dev_num, 1);
  		return -1;
	}

	mpu6050_class = class_create(MPU6050_DEVICE_NAME);

	if (device_create(mpu6050_class, NULL,
			  MKDEV(MAJOR(mpu6050_dev_num), MINOR(mpu6050_dev_num)),
			  NULL, MPU6050_DEVICE_NAME) == NULL) {
		printk(KERN_ERR "[MPU6050] Failed to create device\n");
		return -1;
	}

	printk(KERN_INFO "[MPU6050] device/driver initialized\n");

	enable_cpu_counter();

	return 0;
}

static void mpu6050_remove(struct i2c_client *client)
{
	class_destroy(mpu6050_class);
	cdev_del(mpu6050_cdev);
	unregister_chrdev_region(mpu6050_dev_num, 1);
	disable_cpu_counter();
	printk(KERN_INFO "[MPU6050] device/driver removed\n");
}

static const struct i2c_device_id i2c_id_table[] = { 
	{ "ml-mpu6050", 0 },
	{ } 
};

static const struct of_device_id of_match_table[] = {
	{ .compatible = "ml,mpu6050" },
	{ }
};

MODULE_DEVICE_TABLE(i2c, i2c_id_table);

static struct i2c_driver mpu6050_driver = {
    .driver = {
        .name = "mlpmpu6050",
        .owner = THIS_MODULE,
        .of_match_table = of_match_table,
    },
    .probe = mpu6050_probe,
    .remove = mpu6050_remove,
    .id_table = i2c_id_table,
};

MODULE_LICENSE("GPL");
MODULE_AUTHOR("MAPLELEAF3659");
MODULE_DESCRIPTION("MPU6050 Driver");
MODULE_VERSION("1:0.0");

module_i2c_driver(mpu6050_driver);

