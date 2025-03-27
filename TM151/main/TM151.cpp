#include <stdio.h>
/**
 * Sacado del ejemplo obtenido en la documentación de EasyProfile
 */

#include "esp_log.h"
#include "driver/i2c.h"
#include "driver/uart.h"
#include "driver/gpio.h"


#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "EasyObjectDictionary.h"
#include "EasyProfile.h"

#define UART_NUM UART_NUM_1 // Puedes usar UART0, UART1, UART2 según tu necesidad
#define TX_PIN 17           // Define los pines según tu hardware
#define RX_PIN 18
#define BUF_SIZE 1024 // Tamaño del buffer de recepción

void uart_init()
{
    uart_config_t uart_config = {
        .baud_rate = 115200, // Configura la velocidad deseada
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE};

    // Configurar el UART con los parámetros especificados
    uart_param_config(UART_NUM, &uart_config);
    uart_driver_install(UART_NUM, BUF_SIZE * 2, 0, 0, NULL, 0);
    
    uart_set_pin(UART_NUM, TX_PIN, RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);


}

void SerialPort_SendData(const char *data, int size)
{
    uart_write_bytes(UART_NUM, data, size);
}

void SerialPort_ReadData(char *buffer, int *size)
{
    int len = uart_read_bytes(UART_NUM, buffer, BUF_SIZE, pdMS_TO_TICKS(100)); // Timeout 100 ms
    if (len > 0)
    {
        *size = len;
    }
    else
    {
        *size = 0; // No se recibieron datos
    }
}

extern "C"
{
    void app_main(void);
}

//Step 2: Initialization:
EasyObjectDictionary eOD;
EasyProfile eP(&eOD);

void app_main(void)
{
    uart_init(); // Inicializa el UART

    while (1)
    {

        // Step 3 and Step 4 are optional, only if you want to use the request-response communication pattern
        // Step 3: Request Roll Pitch Yaw Data from TransdcuerM
        uint16 toId = EP_ID_BROADCAST_;
        if (EP_SUCC_ == eOD.Write_Ep_Request(toId, EP_CMD_RPY_))
        {
            EP_ID_TYPE_ txToId;
            char *txData;
            int txSize;
            EP_CMD_TYPE_ txCmd = EP_CMD_REQUEST_;
            if (EP_SUCC_ == eP.On_SendPkg(txCmd, &txToId, &txData, &txSize))
            {                                        // You can request a different data type by changing the EP_CMD_RPY_ to some other value defined in EasyObjectDictionary.h
                SerialPort_SendData(txData, txSize); // Step 4:  Send the request via Serial Port. Please modify this line according to your target platform.
            }
        }

        vTaskDelay(pdMS_TO_TICKS(1000)); // Please avoid sending requests too often. Delay for 1000 ms (1 second).
    }
}

// Step 5: Implement a function that is called every time when new serial data is received, like below
void OnSerialRX(void)
{
    char RxBuffer[64];
    char *rxData;
    int rxSize;
    SerialPort_ReadData(RxBuffer, &rxSize); // Read from serial port buffer. Please modify according to your platform.
    rxData = RxBuffer;
    Ep_Header header; // Then let the EasyProfile do the rest such as data assembling and checksum verification.
    if (EP_SUCC_ == eP.On_RecvPkg(rxData, rxSize, &header))
    {
        uint32 fromId = header.fromId;
        (void)fromId;
        switch (header.cmd)
        { // The program will only reach this line if and only if a correct and complete package has received.
        case EP_CMD_ACK_:
        {
            Ep_Ack ep_Ack; //           tasks for different types of data.
            if (EP_SUCC_ == eOD.Read_Ep_Ack(&ep_Ack))
            {
            }
        }
        break;
        case EP_CMD_STATUS_:
        {
            Ep_Status ep_Status;
            if (EP_SUCC_ == eOD.Read_Ep_Status(&ep_Status))
            {
            }
        }
        break;
        case EP_CMD_COMBO_:
        {
            Ep_Combo ep_Combo;
            if (EP_SUCC_ == eOD.Read_Ep_Combo(&ep_Combo))
            {

                uint32_t timeStamp = ep_Combo.timeStamp;          // timeStamp    unit: uS
                uint32_t deviceId = ep_Combo.header.fromId;       // The Node ID of the TransducerM that transmitted this ep_Combo
                uint16_t qos = ep_Combo.sysState.bits.qos;        // Quality-of-Service  possible values: 0,1,2,3,4,5
                int8_t temperature = ep_Combo.temperature;        // temperature  unit: Celcius
                uint16_t updateRate = (ep_Combo.updateRate) * 10; // updateRate   unit: Hz
                // Accelerometer:
                float ax = (ep_Combo.ax) * (1e-5f); // Unit: 1g, 1g = 9.794m/(s^2)
                float ay = (ep_Combo.ay) * (1e-5f);
                float az = (ep_Combo.az) * (1e-5f);
                // Gyroscope:
                float wx = (ep_Combo.wx) * (1e-5f); // Unit: rad/s
                float wy = (ep_Combo.wy) * (1e-5f);
                float wz = (ep_Combo.wz) * (1e-5f);
                // Magnetometer:
                float mx = (ep_Combo.mx) * (1e-3f); // Unit: one earth magnetic field
                float my = (ep_Combo.my) * (1e-3f); // vector (mx, my, mz) is used as direction reference of the local magnetic field.
                float mz = (ep_Combo.mz) * (1e-3f); // The norm(mx, my, mz) may not be accurate.
                // Quaternion in (w,x,y,z) format
                float q1 = (ep_Combo.q1) * (1e-7f);
                float q2 = (ep_Combo.q2) * (1e-7f);
                float q3 = (ep_Combo.q3) * (1e-7f);
                float q4 = (ep_Combo.q4) * (1e-7f);
                // RPY:
                float roll = (ep_Combo.roll) * (1e-2f);   // Unit: degree
                float pitch = (ep_Combo.pitch) * (1e-2f); // Unit: degree
                float yaw = (ep_Combo.yaw) * (1e-2f);     // Unit: degree

                // Simple checksum:
                uint16_t checksum = 0;
                for (unsigned int i = 0; i < (sizeof(Ep_Combo) - 2); i++)
                {
                    checksum += (*(((uint8_t *)(&ep_Combo)) + i));
                }
                if (checksum == (ep_Combo.simpleChecksum))
                { // This is redundant check and is not necessary when SYD Dynamics communication library is used.
                    // Checksum is correct                            // The simple checksum is designed to be used when user implements own communication library and do not wish to use the CRC checking field of the data stream.

                    // Example use of data here...
                    printf("RawAcc %f %f %f \n", ax, ay, az);
                    printf("RawGyro%f %f %f \n", wx, wy, wz);
                    printf("RawMag %f %f %f \n", mx, my, mz);
                    printf("Q %f %f %f %f\n", q1, q2, q3, q4);
                    printf("RPY %f %f %f\n", roll, pitch, yaw);
                    // ...
                }
            }
        }
        break;
        case EP_CMD_Raw_GYRO_ACC_MAG_:
        { // Here we demonstrate a few examples on how to use the received data
            Ep_Raw_GyroAccMag ep_Raw_GyroAccMag;
            if (EP_SUCC_ == eOD.Read_Ep_Raw_GyroAccMag(&ep_Raw_GyroAccMag))
            {
                // Raw Data received
                unsigned int timeStamp = ep_Raw_GyroAccMag.timeStamp;
                float ax = ep_Raw_GyroAccMag.acc[0]; // Note 1: ep_Raw_GyroAccMag is defined in the EasyProfile library as a global variable
                float ay = ep_Raw_GyroAccMag.acc[1]; // Note 2: for the units and meaning of each value, refer to EasyObjectDictionary.h
                float az = ep_Raw_GyroAccMag.acc[2];
                float wx = ep_Raw_GyroAccMag.gyro[0];
                float wy = ep_Raw_GyroAccMag.gyro[1];
                float wz = ep_Raw_GyroAccMag.gyro[2];
                float mx = ep_Raw_GyroAccMag.mag[0];
                float my = ep_Raw_GyroAccMag.mag[1];
                float mz = ep_Raw_GyroAccMag.mag[2];
                // use raw data here...
                // ...
                printf("RawAcc %f %f %f \n", ax, ay, az);
                printf("RawGyro%f %f %f \n", wx, wy, wz);
                printf("RawMag %f %f %f \n", mx, my, mz);
            }
        }
        break;
        case EP_CMD_Q_S1_E_:
        {
            Ep_Q_s1_e ep_Q_s1_e;
            if (EP_SUCC_ == eOD.Read_Ep_Q_s1_e(&ep_Q_s1_e))
            {
                // Quanternion received
                unsigned int timeStamp = ep_Q_s1_e.timeStamp;
                float q0 = ep_Q_s1_e.q[0]; // Note 1, ep_Q_s1_e is defined in the EasyProfile library as a global variable
                float q1 = ep_Q_s1_e.q[1]; // Note 2, for the units and meaning of each value, refer to EasyObjectDictionary.h
                float q2 = ep_Q_s1_e.q[2];
                float q3 = ep_Q_s1_e.q[3];
                // used q1 q2 q3 q4 here...
                // ...
                printf("Q %f %f %f %f\n", q0, q1, q2, q3);
            }
        }
        break;
        case EP_CMD_EULER_S1_E_:

        {
            Ep_Euler_s1_e ep_Euler_s1_e;
            if (EP_SUCC_ == eOD.Read_Ep_Euler_s1_e(&ep_Euler_s1_e))
            {
            }
        }
        break;
        case EP_CMD_RPY_:
        {
            Ep_RPY ep_RPY;
            if (EP_SUCC_ == eOD.Read_Ep_RPY(&ep_RPY))
            {
                // Roll Pitch Yaw data received
                unsigned int timeStamp = ep_RPY.timeStamp;
                float roll = ep_RPY.roll;   // Note 1, ep_RPY is defined in the EasyProfile library as a global variable
                float pitch = ep_RPY.pitch; // Note 2, for the units and meaning of each value, refer to EasyObjectDictionary.h
                float yaw = ep_RPY.yaw;
                // Use roll, pitch, yaw data here...
                // ...
                printf("RPY %f %f %f\n", roll, pitch, yaw);
            }
        }
        break;
        case EP_CMD_GRAVITY_:
        {
            Ep_Gravity ep_Gravity;
            if (EP_SUCC_ == eOD.Read_Ep_Gravity(&ep_Gravity))
            {
            }
        }
        break;
        }
    }
}

