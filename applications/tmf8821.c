/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2025-02-20     starry       the first version
 */
#include "tmf8821.h"
#include <board.h>
#include<rtthread.h>
#include<rtdevice.h>
#include "drv_common.h"

#define TMF882X_IMAGE_TERMINATION 0x00200089
#define TMF882X_IMAGE_START       0x00200000
#define TMF882X_IMAGE_FINISH      0x00200A4C
#define TMF882X_IMAGE_LENGTH      0x00000A4C

extern const  unsigned char tmf882x_image[];
static uint8_t tmf8821_calib_value[192]={0};
static struct rt_semaphore rx_sem;
/* 回调函数 */
static rt_err_t sensor_input(void *args)
{
    rt_sem_release(&rx_sem);

    return RT_EOK;
}



#define DBG_TAG "tmf8821"
#define DBG_LVL DBG_INFO
#include <rtdbg.h>
/*
 * SCL
 * SDA
 * INT  PA9
 * EN   PA10
 *
 * */

#define INT_PIN GET_PIN(A, 9)
#define EN_PIN  GET_PIN(A, 10)
#define TMF8821_EN_H  rt_pin_write(EN_PIN, PIN_HIGH)
#define TMF8821_EN_L  rt_pin_write(EN_PIN, PIN_LOW)
#define TMF8821_INT_STATUS  rt_pin_read(EN_PIN)


// tmf8821 has as default i2c slave address
#define TMF8821_SLAVE_ADDR          0x41
#define TMF8821_ENABLE              0xe0
#define I2C_DEVICE_NAME             "i2c3"

static struct rt_i2c_bus_device *i2c_bus = RT_NULL;     /* I2C总线设备句柄 */
static rt_bool_t initialized = RT_FALSE;                /* 传感器初始化状态 */



static void basic_io_init()
{
    rt_pin_mode(EN_PIN, PIN_MODE_OUTPUT);
    TMF8821_EN_L;
//    rt_pin_mode(INT_PIN, PIN_MODE_INPUT_PULLUP);
    rt_pin_mode(INT_PIN, PIN_MODE_INPUT_PULLUP);
    rt_pin_attach_irq(INT_PIN, PIN_IRQ_MODE_FALLING, sensor_input, RT_NULL);

    rt_pin_irq_enable(INT_PIN, PIN_IRQ_ENABLE);

}

static rt_err_t read_reg(struct rt_i2c_bus_device *bus, rt_uint8_t reg, rt_uint8_t len, rt_uint8_t *buf)
{
    struct rt_i2c_msg msg[2];

    RT_ASSERT(bus != RT_NULL);

    msg[0].addr  = TMF8821_SLAVE_ADDR;
    msg[0].flags = RT_I2C_WR;
    msg[0].buf   = &reg;
    msg[0].len   = 1;

    msg[1].addr  = TMF8821_SLAVE_ADDR;
    msg[1].flags = RT_I2C_RD;
    msg[1].len   = len;
    msg[1].buf   = buf;

    if (rt_i2c_transfer(bus, msg, 2) == 2)
    {
        return RT_EOK;
    }

    return RT_ERROR;
}

/* i2c write reg */
static rt_err_t write_reg(struct rt_i2c_bus_device *bus, rt_uint8_t reg, rt_uint8_t data)
{
    rt_uint8_t buf[2];
    struct rt_i2c_msg msgs;

    RT_ASSERT(bus != RT_NULL);

    buf[0] = reg ;
    buf[1] = data;

    msgs.addr = TMF8821_SLAVE_ADDR;
    msgs.flags = RT_I2C_WR;
    msgs.buf = buf;
    msgs.len = 2;

    if (rt_i2c_transfer(bus, &msgs, 1) == 1)
    {
        return RT_EOK;
    }

    return RT_ERROR;
}
static rt_err_t write_multi_regs(struct rt_i2c_bus_device *bus, rt_uint8_t reg, rt_uint8_t *data,rt_uint8_t length)
{
    rt_uint8_t buf[30];
    struct rt_i2c_msg msgs;

    RT_ASSERT(bus != RT_NULL);

    buf[0] = reg ;

    for(uint8_t i=0;i<length;i++)
    {
        buf[1+i] = data[i];
    }

    msgs.addr = TMF8821_SLAVE_ADDR;
    msgs.flags = RT_I2C_WR;
    msgs.buf = buf;
    msgs.len = length+1;

    if (rt_i2c_transfer(bus, &msgs, 1) == 1)
    {
        return RT_EOK;
    }

    return RT_ERROR;
}
static uint8_t calculate_checksum( uint8_t *data, uint8_t data_length)
{
    uint16_t sum = 0;
    for (int i = 0; i < data_length; i++)
    {
        sum += data[i];
    }
    return ~(sum & 0xFF); // 取最低字节的补码
}
static uint8_t load_image(struct rt_i2c_bus_device *i2c_bus)
{
    rt_uint8_t read_value[3]={0};
    //发送DOWNLOAD_INIT指令
    uint8_t data1[4]={0x14,0x01,0x29,0xc1};
    write_multi_regs(i2c_bus, 0x08, data1,4);
    read_value[2]=0;
    while(0xff !=read_value[2]){
        read_reg(i2c_bus,0x08,3,&read_value);
        LOG_I("read 2:0x%x",read_value[2]);
        rt_thread_mdelay(100);
    }
    LOG_I("CMD_STAT ready");
    //Issus ADDR_RAM command
    uint8_t data2[5]={0x43,0x02,0x00,0x00,0xBA};
    write_multi_regs(i2c_bus, 0x08, data2,5);
    read_value[2]=0;
   while(0xff !=read_value[2]){
       read_reg(i2c_bus,0x08,3,&read_value);
       LOG_I("read 3:0x%x",read_value[2]);
       rt_thread_mdelay(100);
   }
   LOG_I("Issus ADDR_RAM command ok");



   //单次下载的n不能大于20
   uint16_t img_length=TMF882X_IMAGE_LENGTH;
   uint16_t image_pos=0;
   uint8_t current_nums=0;
   uint8_t checksum=0;
   uint8_t load_data[23]={0};
   uint8_t i=0;

   load_data[0]=0x41;
   current_nums=20;
   load_data[1]=current_nums;
   while((img_length-image_pos)>20)
   {
       for(i=0;i<current_nums;i++)
       {
           load_data[2+i]=tmf882x_image[image_pos+i];
       }

       checksum=calculate_checksum(load_data,current_nums+2);
       load_data[current_nums+2]=checksum;
//       for(i=0;i<current_nums+3;i++)
//        rt_kprintf("%x ",load_data[i]);
//       rt_kprintf("\r\n");
       write_multi_regs(i2c_bus, 0x08,load_data,current_nums+3);

       read_value[2]=0;
       rt_thread_mdelay(10);
       read_reg(i2c_bus,0x08,3,&read_value);
       while(0xff !=read_value[2]){
          read_reg(i2c_bus,0x08,3,&read_value);
          LOG_I("read ->:0x%x",read_value[2]);
          rt_thread_mdelay(100);
      }
//       while(1);


       image_pos +=current_nums;
//       img_length -=current_nums;
   }



   LOG_I("the last download");
   current_nums=img_length-image_pos;
   load_data[1]=current_nums;
   for(i=0;i<current_nums;i++)
  {
      load_data[2+i]=tmf882x_image[image_pos+i];
  }
  checksum=calculate_checksum(load_data,current_nums+2);
  load_data[current_nums+2]=checksum;
  write_multi_regs(i2c_bus, 0x08,load_data,current_nums+3);
  read_value[2]=0;
  rt_thread_mdelay(10);
  read_reg(i2c_bus,0x08,3,&read_value);
     while(0xff !=read_value[2]){
        read_reg(i2c_bus,0x08,3,&read_value);
        LOG_I("read ->:0x%x",read_value[2]);
        rt_thread_mdelay(100);
    }
     LOG_I("finished firmware download");
     uint8_t data3[3]={0x11,0x00,0xee};
    write_multi_regs(i2c_bus, 0x08, data3,3);
    rt_thread_mdelay(4);
    read_value[0]=0;
    //读APPID  3
    read_reg(i2c_bus,0x00,1,&read_value);
    if(0x3 == read_value[0])
    {
        LOG_I("go to app mode");
    }




}

#define PI 3.14149f
// 自定义求最大值的宏
#define MAX(a, b) ((a) > (b) ? (a) : (b))
// 自定义求最小值的宏
#define MIN(a, b) ((a) < (b) ? (a) : (b))

static double calculate_angel_distance(double a,double b,double angel_c)
{
    double c_rad=PI*angel_c/180.0f;
    double c= a*a+b*b-2*a*b*cos(c_rad);
    c=sqrt(c);
    double cos_A=(b*b+c*c-a*a)/(2*b*c);
    cos_A =MAX(-1,MIN(cos_A,1));
    double a_rad=acos(cos_A)* (180.0 / PI);
    return a_rad;
}

static uint8_t temp_sensor_measure_data[128];
static void read_sensor_measure_data()
{
    uint8_t temp=0;
    double angel,A;
    double a,b;
    double distance;
    read_reg(i2c_bus,0xe1,1,&temp);
    write_reg(i2c_bus,0xe1,temp);
    read_reg(i2c_bus,0xe1,1,&temp);

    read_reg(i2c_bus,0x38,3*9,temp_sensor_measure_data);
//    for(uint8_t i=0;i<9;i++)
//    {
//        rt_kprintf("pos:[%d] confidence=%d ,distance=%d\r\n",i,temp_sensor_measure_data[i*3],temp_sensor_measure_data[i*3+1]+temp_sensor_measure_data[i*3+2]*128);
//    }
///
//    if(temp_sensor_measure_data[0] !=255 ||temp_sensor_measure_data[3] !=255||temp_sensor_measure_data[6] !=255)
//    {
//        return ;
//    }

    rt_kprintf("read one frame finished\r\n");
    a=MIN(temp_sensor_measure_data[1]+temp_sensor_measure_data[2]*128,temp_sensor_measure_data[2*3+1]+temp_sensor_measure_data[2*3+2]*128);
    b=temp_sensor_measure_data[1*3+1]+temp_sensor_measure_data[1*3+2]*128;
    A=calculate_angel_distance(a,b,16.0);
    angel=90.0-A;
    distance=b*sin(PI*A/180.0f);
    LOG_I("get angel=[%d.%2d]'C distance=[%d.%2d]mm",(uint32_t)angel,(uint16_t)(angel*100)%100,(uint32_t)distance,(uint16_t)(distance*100)%100);


}
static void sensor_thread_entry(void *parameter)
{
    while (1)
    {
        rt_sem_take(&rx_sem, RT_WAITING_FOREVER);
//        rt_kprintf("\r\nget\r\n");
        read_sensor_measure_data();
        rt_thread_mdelay(100);
    }
}
static void tmf8821_init(const char *name)
{
    uint8_t temp_data=0;
    rt_uint8_t read_value[4]={0};
    //STEP 1

    basic_io_init();
    rt_thread_mdelay(20);
    //STEP 2
    TMF8821_EN_H;
    /* 查找I2C总线设备，获取I2C总线设备句柄 */
    i2c_bus = (struct rt_i2c_bus_device *)rt_device_find(name);
    if (i2c_bus == RT_NULL)
    {
        LOG_I("can't find %s device!\n", name);
        return ;
    }
    //STEP 3
    LOG_I("STEP 3");
    write_reg(i2c_bus, TMF8821_ENABLE, 0x1);
    //STEP 4
    LOG_I("STEP 4");
    while(0x41 !=read_value[0]){
        read_reg(i2c_bus,TMF8821_ENABLE,1,&read_value);
        LOG_I("read:0x%x",read_value[0]);
        rt_thread_mdelay(500);
    }
//    LOG_I("read ->0x41");
    read_reg(i2c_bus,0x00,1,&read_value);
    if(0x80 == read_value[0])
    {
        LOG_I("go to bootloader mode");
    }

    //STEP 4
    load_image(i2c_bus);



    //工厂校准
    LOG_I("factory calibration start");
    //开始校准
    write_reg(i2c_bus, 0x08, 0x20);
    rt_thread_mdelay(10);
    read_reg(i2c_bus,0x08,1,&temp_data);
    while(0x00 !=temp_data){
        read_reg(i2c_bus,0x08,1,&temp_data);
        LOG_I("read  0x08:0x%x",temp_data);
        rt_thread_mdelay(100);
    }
    //准备读取
    write_reg(i2c_bus, 0x08, 0x19);
    rt_thread_mdelay(10);
    read_reg(i2c_bus,0x08,1,&temp_data);
    while(0x00 !=temp_data){
        read_reg(i2c_bus,0x08,1,&temp_data);
        LOG_I("read  0x08:0x%x",temp_data);
        rt_thread_mdelay(100);
    }
    //读取校准数据  0x20~0xDF  192个
    for(uint8_t i=0;i<9;i++)
        read_reg(i2c_bus,0x20+i*20,20,tmf8821_calib_value+i*20);
    read_reg(i2c_bus,0x20+9*20,12,tmf8821_calib_value+9*20);


    //载入校准数据
    LOG_I("load tmf8821");
    write_reg(i2c_bus, 0x08, 0x19);
    rt_thread_mdelay(10);
    read_reg(i2c_bus,0x08,1,&temp_data);
    while(0x00 !=temp_data){
        read_reg(i2c_bus,0x08,1,&temp_data);
        LOG_I("read  0x08:0x%x",temp_data);
        rt_thread_mdelay(100);
    }
    read_reg(i2c_bus,0x20,3,read_value);
    while(0x19 !=read_value[0] || 0xbc !=read_value[2] ||0x00 !=read_value[3]){
        read_reg(i2c_bus,0x20,4,read_value);
        LOG_I("read  0x20->0x%x 0x%x 0x%x",read_value[0],read_value[2],read_value[3]);
        rt_thread_mdelay(500);
    }
    for(uint8_t i=0x24;i<0xdf;i++)
    {
        write_reg(i2c_bus, i, tmf8821_calib_value[i-0x20]);
    }
    write_reg(i2c_bus, 0x08, 0x15);
    rt_thread_mdelay(10);
    read_reg(i2c_bus,0x08,1,&temp_data);
    while(0x00 !=temp_data){
        read_reg(i2c_bus,0x08,1,&temp_data);
        LOG_I("read  0x08:0x%x",temp_data);
        rt_thread_mdelay(100);
    }
    LOG_I("write back ok");
    //final
    //配置传感器
    LOG_I("config tmf8821");
    write_reg(i2c_bus, 0x08, 0x16);
    rt_thread_mdelay(10);
    read_reg(i2c_bus,0x08,1,&temp_data);
    while(0x00 !=temp_data){
        read_reg(i2c_bus,0x08,1,&temp_data);
        LOG_I("read  0x08:0x%x",temp_data);
        rt_thread_mdelay(100);
    }
    //查配置页面是否被正确载入
    read_reg(i2c_bus,0x20,3,read_value);
    while(0x16 !=read_value[0] || 0xbc !=read_value[2] ||0x00 !=read_value[3]){
        read_reg(i2c_bus,0x20,4,read_value);
        LOG_I("read  0x20->0x%x 0x%x 0x%x",read_value[0],read_value[2],read_value[3]);
        rt_thread_mdelay(500);
    }
    //设定测量周期
    uint8_t data_1[2]={0x64,0x00};
    write_multi_regs(i2c_bus, 0x24, data_1,2);

    //设定SPAD类型
    write_reg(i2c_bus, 0x34,0x06);
    //配置传感器 GPIO0
    write_reg(i2c_bus, 0x31,0x03);
    //将配置写入传感器
    write_reg(i2c_bus, 0x08,0x15);
    //使能中断
    write_reg(i2c_bus, 0xe2,0x02);

    //清除中断
    write_reg(i2c_bus, 0xe1,0xff);


    //开始测量
    write_reg(i2c_bus, 0x08,0x10);

    //检查CMD_STAT寄存器状态
    rt_thread_mdelay(10);
    read_reg(i2c_bus,0x08,1,&temp_data);
    while(0x01 !=temp_data){
        read_reg(i2c_bus,0x08,1,&temp_data);
        LOG_I("read  0x08:0x%x",temp_data);
        rt_thread_mdelay(100);
    }
    LOG_I("get c1  !!!");
    initialized = RT_TRUE;


}

static int i2c_tmf8821_sample(void)
{
    uint8_t ret=0;
    rt_sem_init(&rx_sem, "tmf_sem", 0, RT_IPC_FLAG_FIFO);

    /* 创建 serial 线程 */
    rt_thread_t thread = rt_thread_create("tmf", sensor_thread_entry, RT_NULL, 1024, 5, 10);
    /* 创建成功则启动线程 */
    if (thread != RT_NULL)
    {
        rt_thread_startup(thread);
    }
    else
    {
        ret = RT_ERROR;
    }
    if (!initialized)
    {
        /* 传感器初始化 */
        tmf8821_init(I2C_DEVICE_NAME);
    }
    if (initialized)
    {


//        LOG_I("initialize sensor successfully!\n");

    }
    else
    {
        LOG_I("initialize sensor failed!\n");
    }
}
/* 导出到 msh 命令列表中 */
//MSH_CMD_EXPORT(i2c_tmf8821_sample, i2c tmf8821_init sample);




INIT_APP_EXPORT(i2c_tmf8821_sample);
