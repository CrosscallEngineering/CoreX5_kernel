#ifndef _FOCAL_MCAPTESTLIB_H
#define _FOCAL_MCAPTESTLIB_H
#define TX_NUM_MAX 80
#define RX_NUM_MAX 80
/*enum boolean {false = 0, true = 1,};*/
#define boolean unsigned char
#define false 0
#define true  1


typedef int (*FTS_I2c_Read_Function)(unsigned char *, int , unsigned char *, int);
typedef int (*FTS_I2c_Write_Function)(unsigned char *, int);

int Init_I2C_Read_Func(FTS_I2c_Read_Function fpI2C_Read);
int Init_I2C_Write_Func(FTS_I2c_Write_Function fpI2C_Write);
int SetParamData(char *TestParamData);
/* void FreeTestParamData(void); */
void focal_save_scap_sample(void);
boolean StartTestTP(void);
int testcsv(char *databuf);
int testtxt(char *databuf);
/*return true pass,else ng*/
#endif
