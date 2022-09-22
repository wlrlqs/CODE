#include "Driver_icm42605.h"
#include "util.h"
#include "processing.h"
#include "imu.h"
#include "iir_filter.h"
#include "Driver_USBVCP.h"
static BSP_SPI_TypeDef ICM42605_SPI_Base = ICM42605_SPI_DEFAULT;
static BSP_SPI_TypeDef *ICM42605_SPI = &ICM42605_SPI_Base;

static icm42605Struct_t icm42605Data;

float *getIcm42605GyroScale(void) {
    return &icm42605Data.gyroScale;
}

float *getIcm42605AccelScale(void) {
    return &icm42605Data.accelScale;
}

int16_t getIcm42605Gyroscope(uint8_t index) {
    return icm42605Data.gyroscope[index].s16_temp;
}

int16_t getIcm42605Acceleration(uint8_t index) {
    return icm42605Data.acceleration[index].s16_temp;
}

int16_t getIcm42605Temperature(void) {
    return icm42605Data.temperature.s16_temp;
}

/*
***************************************************
函数名：driver_ICM42605_SPI_Write
功能：SPI方式向ICM42605的指定寄存器写入一个字节数据
入口参数：	regAddr：ICM42605的寄存器地址
            dat：要写入的数据
返回值：无
应用范围：外部调用
备注：
***************************************************
*/
void driver_Icm42605_SPI_Write(uint8_t regAddr, uint8_t dat) {
	ICM42605_CS = 0;
	BSP_SPI_ReadWriteByte(ICM42605_SPI, regAddr & 0x7f);
	BSP_SPI_ReadWriteByte(ICM42605_SPI, dat);
	ICM42605_CS = 1;
}

/*
***************************************************
函数名：driver_ICM42605_SPI_Read
功能：SPI方式从ICM42605的指定寄存器读出一个字节数据
入口参数：	regAddr：ICM42605的寄存器地址
返回值：dat：读出的数据
应用范围：外部调用
备注：
***************************************************
*/
uint8_t driver_Icm42605_SPI_Read(uint8_t regAddr) {
    //读取的数据
	uint8_t dat = 0;
	ICM42605_CS = 0;
	BSP_SPI_ReadWriteByte(ICM42605_SPI, 0x80 | regAddr);
    dat = BSP_SPI_ReadWriteByte(ICM42605_SPI, 0);
	ICM42605_CS = 1;
	return dat;
}

/*
***************************************************
函数名：Driver_ICM42605_SPI_Reads
功能：SPI方式从ICM42605指定寄存器读出指定长度字节数据
入口参数：	regAddr：ICM42605的寄存器地址
					len：读出长度
					dat：要读出的数据的指针
返回值：无
应用范围：外部调用
备注：
***************************************************
*/
uint8_t driver_Icm42605_SPI_Reads(uint8_t regAddr, uint8_t len, uint8_t *dat) {
	uint32_t i = 0;
	ICM42605_CS = 0;
	BSP_SPI_ReadWriteByte(ICM42605_SPI, 0x80 | regAddr);
	while(i < len) {
		dat[i++] = BSP_SPI_ReadWriteByte(ICM42605_SPI, 0);
	}
	ICM42605_CS = 1;
	return 0;
}

static uint16_t dataCNTR_Check(void) {
	static uint16_t last_CNTR;
	uint16_t value = abs((int16_t)icm42605Data.dataCNTR.s16_temp - (int16_t)last_CNTR);
	last_CNTR = icm42605Data.dataCNTR.s16_temp ;
	return value;
}

static void calculateIcm42605GyroScale(void) {
    float actualFSR = 0.0f;
    switch (*icm42605Data.gyroFSR) {
        case ICM42605_FSR_2000DPS:
            actualFSR = 2000.0f;
            break;
        case ICM42605_FSR_1000DPS:
            actualFSR = 1000.0f;
            break;
        case ICM42605_FSR_500DPS:
            actualFSR = 500.0f;
            break;
        case ICM42605_FSR_250DPS:
            actualFSR = 250.0f;
            break;
        case ICM42605_FSR_125DPS:
            actualFSR = 125.0f;
            break;
        case ICM42605_FSR_62DPS:
            actualFSR = 62.5f;
            break;
        case ICM42605_FSR_31DPS:
            actualFSR = 31.25f;
            break;
        case ICM42605_FSR_15DPS:
            actualFSR = 15.625f;
            break;
    }
    icm42605Data.gyroScale =  1.0f / ((1 << 16) / (actualFSR * 2.0f)) * DEG_TO_RAD;   
}

static void calculateIcm42605AccelScale(void) {
    float actualFSR = 0.0f;
    switch (*icm42605Data.accelFSR) {
        case ICM42605_FSR_16G:
            actualFSR = 16.0f;
            break; 
        case ICM42605_FSR_8G:
            actualFSR = 8.0f;
            break;
        case ICM42605_FSR_4G:
            actualFSR = 4.0f;
            break;
        case ICM42605_FSR_2G:
            actualFSR = 2.0f;
            break;
    }
    icm42605Data.accelScale = 1.0f / ((1 << 16) / (actualFSR * 2.0f)) * GRAVITY; 
}

/*
***************************************************
函数名：		Icm42605Check
功能：			检测是否与ICM42605连接
入口参数：	无
返回值：		0：检测到
					1：未检测到
应用范围：	内部调用
备注：
***************************************************
*/
static uint8_t Icm42605Check(void) {
	icm42605Data.whoIAm = driver_Icm42605_SPI_Read(ICM42605_WHO_AM_I);
	if(icm42605Data.whoIAm == 0x42) {
		return 1;
    }
	else {
		return 0;
    }
}

/*
***************************************************
函数名：		changeIcm42605GyroFSR
功能：			更改ICM42605的加速度和陀螺仪FSR
入口参数：	无
返回值：		无
应用范围：	内部调用
备注：
***************************************************
*/
static void changeIcm42605GyroFSR(void) {
    volatile uint8_t regVal = 0;
    uint32_t gyroFSR = *icm42605Data.gyroFSR;
    uint32_t accelFSR = *icm42605Data.accelFSR;
    //检查加速度是否是自动FSR
    if(accelFSR == ICM42605_FSR_AUTO_G) {
        
    }
    else {
        if(accelFSR != icm42605Data.lastAccelFSR) {
            //重新计算分辨率
            calculateIcm42605AccelScale();
            //set accel fsr
            vTaskDelay(1);
            regVal = (driver_Icm42605_SPI_Read(ICM42605_ACCEL_CONFIG0) & 0x1F) | (accelFSR << 5);
            driver_Icm42605_SPI_Write(ICM42605_ACCEL_CONFIG0, regVal);
            icm42605Data.lastAccelFSR = accelFSR;
        }
    }
    //检查加速度是否是自动FSR
    if(gyroFSR == ICM42605_FSR_AUTO_DPS) {
        
    }
    else {    
        if(gyroFSR != icm42605Data.lastGyroFSR) {
            //重新计算分辨率
            calculateIcm42605GyroScale();
            //set gyro fsr
            vTaskDelay(1);
            regVal = (driver_Icm42605_SPI_Read(ICM42605_GYRO_CONFIG0) & 0x1F) | (gyroFSR << 5);
            driver_Icm42605_SPI_Write(ICM42605_GYRO_CONFIG0, regVal);
            icm42605Data.lastGyroFSR = gyroFSR;
        }
    }
}

/*
***************************************************
函数名：		brustReadIcm42605
功能：             ICM42605数据突发读取
入口参数：	无
返回值：		传感器间隔CNTR
应用范围：	外部调用
备注：
***************************************************
*/
uint16_t brustReadIcm42605(void) {
    uint8_t data[ICM42605_BRUST_LENGTH];
    uint16_t dataSum = 0;
    memset(data, 0, sizeof(uint8_t) * ICM42605_BRUST_LENGTH);
    //set to bank 0
    driver_Icm42605_SPI_Write(ICM42605_REG_BANK_SEL, 0x00);
    
    changeIcm42605GyroFSR();
    
    driver_Icm42605_SPI_Reads(ICM42605_TEMP_DATA1,  ICM42605_BRUST_LENGTH, data);
    
    icm42605Data.temperature.s16_temp = (data[0] << 8) | data[1];
    icm42605Data.acceleration[0].s16_temp = (data[2] << 8) | data[3];
	icm42605Data.acceleration[1].s16_temp = (data[4] << 8) | data[5];
	icm42605Data.acceleration[2].s16_temp = (data[6] << 8) | data[7];
    icm42605Data.gyroscope[0].s16_temp = (data[8]  << 8) | data[9];
	icm42605Data.gyroscope[1].s16_temp = (data[10] << 8) | data[11];
	icm42605Data.gyroscope[2].s16_temp = (data[12] << 8) | data[13];
    dataSum = icm42605Data.acceleration[0].s16_temp + icm42605Data.acceleration[1].s16_temp \
                    + icm42605Data.acceleration[2].s16_temp + icm42605Data.temperature.s16_temp \
                    + icm42605Data.gyroscope[0].s16_temp + icm42605Data.gyroscope[1].s16_temp \
                    + icm42605Data.gyroscope[2].s16_temp;
    if(dataSum != 0) {
        digitalIncreasing(&icm42605Data.dataCNTR.u16_temp);
    }
    return dataCNTR_Check();
}

/*
***************************************************
函数名：		driver_Icm42605_Init
功能：             ICM42605初始化
入口参数：	gyroFSR：角速度分辨率
            accelFSR：加速度分辨率
返回值：		true：初始化成功
            false：初始化失败
应用范围：	内部调用
备注：
***************************************************
*/
bool driver_Icm42605_Init(uint8_t *gyroFSR, uint8_t *accelFSR) {
    memset(&icm42605Data, 0, sizeof(icm42605Struct_t));
//    uint16_t waitCheckTime = 0;
//    uint8_t ask = 0;
    volatile uint8_t regVal = 0;
    
    //给指针赋值
    icm42605Data.gyroFSR = gyroFSR;
    icm42605Data.accelFSR = accelFSR;

    /****************** 单片机内部资源设置 ********************/
	BSP_SPI_Init(ICM42605_SPI);
	BSP_SPIx_SetSpeed(ICM42605_SPI,ICM42605_SPI_MIN_PRESCALER);
    /****************** ICM42605初始化设置 ********************/   
    //set to bank 0
    driver_Icm42605_SPI_Write(ICM42605_REG_BANK_SEL, 0x00);
    //chip soft reset
    driver_Icm42605_SPI_Write(ICM42605_DEVICE_CONFIG, 0x01);
    vTaskDelay(1);    
    //check link
//    while(!ask && waitCheckTime < 5000) {
//		ask = Icm42605Check();
//		waitCheckTime++;
//	}
//	if(!ask) {
//		return false;
//    }
    //set to bank 1
    driver_Icm42605_SPI_Write(ICM42605_REG_BANK_SEL, 0x01);
    //4 wire spi mode
    driver_Icm42605_SPI_Write(ICM42605_INTF_CONFIG4, 0x02);   
    //set to bank 0
    driver_Icm42605_SPI_Write(ICM42605_REG_BANK_SEL, 0x00);
//    //stream-to-FIFO Mode
//    driver_Icm42605_SPI_Write(ICM42605_FIFO_CONFIG, 0x40);
//    
//    regVal = driver_Icm42605_SPI_Read(ICM42605_INT_SOURCE0);
//    driver_Icm42605_SPI_Write(ICM42605_INT_SOURCE0, 0x00);
//    // watermark
//    driver_Icm42605_SPI_Write(ICM42605_FIFO_CONFIG2, 0x00);
//    // watermark
//    driver_Icm42605_SPI_Write(ICM42605_FIFO_CONFIG3, 0x02);
//    driver_Icm42605_SPI_Write(ICM42605_INT_SOURCE0, regVal);
//    // Enable the accel and gyro to the FIFO
//    driver_Icm42605_SPI_Write(ICM42605_FIFO_CONFIG1, 0x63);
//    
//    driver_Icm42605_SPI_Write(ICM42605_INT_CONFIG, 0x36);
//    //FIFO threshold interrupt
//    regVal = driver_Icm42605_SPI_Read(ICM42605_INT_SOURCE0) | 0x04;
//    driver_Icm42605_SPI_Write(ICM42605_INT_SOURCE0, regVal);
    
    /************** ICM42605加速度和陀螺仪初始化 ****************/
    icm42605Data.lastAccelFSR = ICM42605_FSR_INIT_G;
    icm42605Data.lastGyroFSR = ICM42605_FSR_INIT_DPS;
    changeIcm42605GyroFSR();
    
    //set accel odr
    vTaskDelay(1);
    regVal = (driver_Icm42605_SPI_Read(ICM42605_ACCEL_CONFIG0) & 0xF0) | ICM42605_ODR_1000;
    driver_Icm42605_SPI_Write(ICM42605_ACCEL_CONFIG0, regVal);
    //set gyro odr
    vTaskDelay(1);
    regVal = (driver_Icm42605_SPI_Read(ICM42605_GYRO_CONFIG0) & 0xF0) | ICM42605_ODR_1000;
    driver_Icm42605_SPI_Write(ICM42605_GYRO_CONFIG0, regVal);
    //set temperature LPF
    vTaskDelay(1);
    regVal = (driver_Icm42605_SPI_Read(ICM42605_GYRO_CONFIG1) & 0x1F) | (0x03 << 5);
    driver_Icm42605_SPI_Write(ICM42605_GYRO_CONFIG1, regVal);
    //set accel LPF
    regVal = (driver_Icm42605_SPI_Read(ICM42605_GYRO_ACCEL_CONFIG0) & 0x0F) | (ICM42605_DLPF_110HZ << 4);
    driver_Icm42605_SPI_Write(ICM42605_GYRO_ACCEL_CONFIG0, regVal);  
    //set gyro LPF
    regVal = (driver_Icm42605_SPI_Read(ICM42605_GYRO_ACCEL_CONFIG0) & 0xF0) | ICM42605_DLPF_110HZ;
    driver_Icm42605_SPI_Write(ICM42605_GYRO_ACCEL_CONFIG0, regVal);
    //accel on in LN mode,LN : low noise,start work
    vTaskDelay(1);
    regVal = driver_Icm42605_SPI_Read(ICM42605_PWR_MGMT0) | 0x03;
    driver_Icm42605_SPI_Write(ICM42605_PWR_MGMT0, regVal);
    //gyro on in LN mode
    vTaskDelay(1);
    regVal = driver_Icm42605_SPI_Read(ICM42605_PWR_MGMT0) | (0x03 << 2);
    driver_Icm42605_SPI_Write(ICM42605_PWR_MGMT0, regVal);
    return true;
}

void imu_data_read(void){
	brustReadIcm42605();
	imuSensorOfChassisData.rawGyo[0] = (f32_t)iirButterworth_4thFilter(getfilter_gyro(),icm42605Data.gyroscope[0].s16_temp);
	imuSensorOfChassisData.rawGyo[1] = (f32_t)iirButterworth_4thFilter(getfilter_gyro(),icm42605Data.gyroscope[1].s16_temp);
	imuSensorOfChassisData.rawGyo[2] = (f32_t)iirButterworth_4thFilter(getfilter_gyro(),icm42605Data.gyroscope[2].s16_temp);
	imuSensorOfChassisData.gyo[0] = imuSensorOfChassisData.rawGyo[0] * icm42605Data.gyroScale;
	imuSensorOfChassisData.gyo[1] = imuSensorOfChassisData.rawGyo[1] * icm42605Data.gyroScale;
	imuSensorOfChassisData.gyo[2] = imuSensorOfChassisData.rawGyo[2] * icm42605Data.gyroScale;
	
	usbVCP_Printf("gyoData if %f",imuSensorOfChassisData.gyo[2]);
}

