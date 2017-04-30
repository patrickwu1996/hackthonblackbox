#include "CurieTimerOne.h"
#include "Typedef.h"
#include "JY901.h"
#include "portable.h"
#include "scss_registers.h"
#include "wiring_analog.h"
#include "variant.h"
#include <SoftwareSerial.h>
#include "dht11.h"

#define DHT11PIN 10

//To Do
//測試接收JY901B UART
//測試控制伺股馬達輸入範圍值
//測試AD轉換空壓管
//測試LED

struct STime    stcTime;
struct SAcc     stcAcc;
struct SGyro    stcGyro;
struct SAngle   stcAngle;
struct SMag     stcMag;
struct SDStatus stcDStatus;
struct SPress   stcPress;
struct SLonLat  stcLonLat;
struct SGPSV    stcGPSV;

typedef union {
    
    unsigned short width;

    unsigned char array[ 2 ];

}_pwm;

typedef union
{
    unsigned char array[ 4 ];
    
    unsigned int Data; 
    
}_Float_Data;

_Float_Data Rec_lontitude;
_Float_Data Rec_latitude;
_Float_Data Rec_high;

_pwm pwm_value[ 4 ];

int X=0;
int Y=0;
int Z=0;
int count1=0;
int dir=0;
int pcount=0;
int tcount=0;
int zcount=0;
int counter=0;
int tempad[4];
int adkey=0;
float real_ad=0;
float t=0;
float x_angle[5];
float y_angle[5];
float z_angle[5];
float x_acc[5];
float acX=0;
float acZ=0;
float z_acc[5];
float accx[2];
float accz[2];
float offset=0;
float h=500;
float d=0;
float clat=0;
float clon=0;
float p=0;
float temlon[2];
float temlat[2];
float course=0;
float xsum=0;
float ysum=0;
float zsum=0;
float accxsum=0;
float acczsum=0;
float dx=0;
float dz=0;
float w=0;
float b=0;
float x_rudder=0;
float y_rudder=0;
float nose_down=0;
float lontitude=0;
float latitude=0;
float lontitude_home=121.268323;
float latitude_home=24.86670;
float dlontitude=0;
float dlatitude=0;                  //經緯度假設
u16 Right_rudder_stand_point=0;     //右舵片中立點
u16 Left_rudder_stand_point=0;      //左舵片中立點
u16 Right_rudder_uppermost_sheet=0; //右舵片最上點
u16 Left_rudder_uppermost_sheet=0;  //左舵片最上點
u16 Right_rudder_lowermost_sheet=0; //右舵片最下點
u16 Left_rudder_lowermost_sheet=0;  //左舵片最下點
u16 ad_data;
u16 Right_rudder_actual_value=0;    //右舵實際數值
u16 Left_rudder_actual_value=0;     //左舵實際數值
u16 wind_open;
u16 wind_close;
u16 hook_open;
u16 hook_close;
u16 hook_actual;
u16 wind_actual=569;
float Moment_right_rudder=0;        //當下右舵百分比小數用於運算使用
float Left_rudder_moment=0;         //當下左舵把分比小數用於運算使用
int Moment_right_rudder1=0;         //當下右舵百分比
int Left_rudder_moment1=0;          //當下左舵把分比
double azimuth=0;
u8 str[100];

float RU_difference;                //與基準比較減少 右向上與基準差值
float RD_difference;                //與基準比較增加  右向下與基準差值
float LU_difference;                //與基準比較增加    左向上與基準值差值
float LD_difference;                //與基準比較減少    左向下與基準值差值

unsigned char GPSandHightValue[ 250 ];
unsigned char GPSandHightCnt = 0;

unsigned char ucRxBuffer[250];
unsigned char ucRxCnt = 0;

const int oneSecInUsec = 1000000;   // A second in mirco second unit.
int time;                           // the variable used to set the Timer

void Up_and_down_control( int Up_and_down_Right,int Up_and_down_Left ); //右最大值+100,-100  左最大值+100,-100
void Fly_fly_left_and_right( int Right_and_left_offset );               //+100右  -100左
void The_actual_motor_pwm( void );                                      //計算實際要打pwmp
void fly_option( void );
void wind_up_down( void );
void wireless( int ad );
void position( void );            //方位角計算
void height( float pressure );    //calculate the height
void airspeed( int ad_data );

unsigned char led_status = LOW;
int analogPin;                    // select the input pin for the potentiometer
int ledPin = 13;                  // LED connected to digital pin 13
unsigned int timer_cnt = 0;

long previousMillis = 0;  // last time the battery level was checked, in ms

unsigned char hookservo_pin = 3;
unsigned char windservo_pin = 5;
unsigned char Right_rudderservo_pin = 6;   // 右舵 pwm
unsigned char Left_rudderservo_pin = 9;    // 左舵 pwm
           
SoftwareSerial BTSerial( 7, 8 );  // RX, TX

dht11 DHT11;

// Used to serially push out a String with Serial.write( )
void BT_WriteString( String stringData ) 
{ 
  for( unsigned int i = 0; i < stringData.length(); i++ )
  {
    BTSerial.write( stringData[i] );   // Push each char 1 by 1 on each loop pass
  }
}
    
// callback function when interrupt is asserted
void Time_1mSec_ISR()        
{
    t++;
    
    if( timer_cnt > 0 )
    {
        timer_cnt --;
    }
    
    //Serial1.println( "1mSec Interrupt occurs" );
    //Serial1.print( t );
    // ---------------------------
    
    pcount++;
    x_angle[dir]=X;
    y_angle[dir]=Y;
    z_angle[dir]=Z;
    x_acc[dir]=acX;
    z_acc[dir]=acZ;
    dir++;
    
    if(dir==4)
    {
        for(dir=0;dir<5;dir++)
        {
            xsum=xsum+x_angle[dir];
            ysum=ysum+y_angle[dir];
            zsum=zsum+z_angle[dir];
            accxsum=accxsum+x_acc[dir];
            acczsum=acczsum+z_acc[dir];
            x_angle[dir]=0;
            y_angle[dir]=0;
            z_angle[dir]=0;
            x_acc[dir]=0;
            z_acc[dir]=0;
        }
        xsum=xsum/5;
        ysum=ysum/5;
        zsum=zsum/5;
        y_rudder=-ysum;
        ysum=0;
        
        if(real_ad>=50)
        {
            x_rudder=-xsum;
            xsum=0;
            if(offset>45)offset=45;
            if(offset<-45)offset=-45;
            x_rudder=x_rudder+offset*1.1;
            if(offset>20 || offset<-20)y_rudder=-(y_rudder-10);
        }
        
        accxsum=accxsum/5;
        acczsum=acczsum/5;
        accx[count1]=accxsum;
        accz[count1]=acczsum;
        dir=0;
        
        if(count1==1)count1=0;
        else count1++;
    }
    
    if(counter>=500)
    {
        tcount++;
        wireless(ad_data);
    }
    
    //height(stcPress.lPressure);

    if( adkey == 0 )
    {
        counter++;                   //count 5 seconds
    }
    // ----------------------------
    
    CurieTimerOne.restart( time );    // Restarts Timer
}

void Adc_Init( void )
{   
    // These constants won't change.  They're used to give names
    // to the pins used:
    analogPin = A0;     // potentiometer wiper (middle terminal) connected to analog pin 3
}                 

unsigned int Get_Adc_Average( unsigned char times )
{
    u32 temp_val=0;
    u8 t;
    
    for( t = 0; t < times; t++ )
    {
        temp_val += analogRead( analogPin );
        //delay_ms(5);
    }
    
    return temp_val/times;
}    

// --------------------------
// pin 範圍 3，5，6，9
// val 範圍 0~100 
// ---------------------------
void PWM_PinInit( unsigned char pin, unsigned int val )
{
  const uint32_t ulcnt = 222208 * 2;    //72Hz 13.8mSec
  
  #if 0
  const uint32_t max_width = 2400 * 32; //2400uSec
  const uint32_t min_width = 500 * 32;  //500uSec
  #endif
  
  uint8_t pinmuxMode[NUM_DIGITAL_PINS];
  uint32_t offset;
  uint32_t hcnt = 0; // 1/32MHz * ??? = 6944uSec
  uint32_t lcnt = 0; // 1/32MHz * ??? = 6944uSec
    
  static unsigned int save_compare_data[ ] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };

  if( pin > 10 )
  { 
    Serial1.print( "pin set fail" );
    return;
  }
  //已設定過PWM寬度，提早離開，不重覆設定
  if( save_compare_data[ pin ] == val ) return;
  
  save_compare_data[ pin ] = val;
  
  PinDescription *p = &g_APinDescription[pin];

  #if 0
  hcnt = map( val, 0, 100, min_width, max_width );
  #endif
  hcnt = val * 32;
  
  lcnt = ulcnt - hcnt;
  
  /* Set the high count period (duty cycle) */
  offset = ((p->ulPwmChan * QRK_PWM_N_LCNT2_LEN) + QRK_PWM_N_LOAD_COUNT2);
  MMIO_REG_VAL(QRK_PWM_BASE_ADDR + offset) = hcnt;
  
  /* Set the low count period (duty cycle) */
  offset = ((p->ulPwmChan * QRK_PWM_N_REGS_LEN) + QRK_PWM_N_LOAD_COUNT1);
  MMIO_REG_VAL(QRK_PWM_BASE_ADDR + offset) = lcnt;

  /* start the PWM output */
  offset = ((p->ulPwmChan * QRK_PWM_N_REGS_LEN) + QRK_PWM_N_CONTROL);
  SET_MMIO_MASK(QRK_PWM_BASE_ADDR + offset, QRK_PWM_CONTROL_ENABLE);
  
  if(pinmuxMode[pin] != PWM_MUX_MODE)
  {
      /* Disable pull-up and set pin mux for PWM output */
      SET_PIN_PULLUP(p->ulSocPin, 0);
      SET_PIN_MODE(p->ulSocPin, PWM_MUX_MODE);
  }
}

void PrintDebugMessage()
{
    #if 1
    
    BT_WriteString( "\r\nUART Receiver Acc:" );
    BT_WriteString( String( stcAcc.a[0], HEX ) );
    BT_WriteString( " " );
    BT_WriteString( String( stcAcc.a[1], HEX ) );
    BT_WriteString( " " );
    BT_WriteString( String( stcAcc.a[2], HEX ) );
    
    BT_WriteString( "\r\nUART Receiver pwm_array:" );
    BT_WriteString( String( pwm_value[0].width, HEX ) );
    BT_WriteString( " " );
    
    BT_WriteString( String( pwm_value[1].width, HEX ) );
    BT_WriteString( " " );
    
    BT_WriteString( String( pwm_value[2].width, HEX ) );
   
    #endif
}

#if 1
void serialEvent( )
{
    // if there's any serial available, read it:
    while( Serial.available( ) > 0 ) 
    {
        // get the new byte:
        unsigned char inChar = ( unsigned char )Serial.read( );
                
        // BT_WriteString( String( inChar, HEX ) );
        // Serial.write( inChar );
        
        GPSandHightValue[ GPSandHightCnt++ ]= inChar;
        
        //BT_WriteString( "\r\nString( GPSandHightValue[ 0 ], HEX )" );
        //BT_WriteString( String( GPSandHightValue[ 0 ], HEX ) );
        
        //BT_WriteString( "\r\nGPSandHightCnt" );
        //BT_WriteString( String(GPSandHightCnt, HEX  ) );

        if( GPSandHightValue[ 0 ] != 0x55 )
        {
            GPSandHightCnt=0;
            return;
        }
        
        if( GPSandHightCnt < 13 )
        {
            return;
        }
        else
        {
            Rec_lontitude.array[ 0 ] = GPSandHightValue[ 4 ];
            Rec_lontitude.array[ 1 ] = GPSandHightValue[ 3 ];
            Rec_lontitude.array[ 2 ] = GPSandHightValue[ 2 ];
            Rec_lontitude.array[ 3 ] = GPSandHightValue[ 1 ];
            
            Rec_latitude.array[ 0 ] = GPSandHightValue[ 8 ];
            Rec_latitude.array[ 1 ] = GPSandHightValue[ 7 ];
            Rec_latitude.array[ 2 ] = GPSandHightValue[ 6 ];
            Rec_latitude.array[ 3 ] = GPSandHightValue[ 5 ];
            
            Rec_high.array[ 0 ]  = GPSandHightValue[ 12 ];
            Rec_high.array[ 1 ]  = GPSandHightValue[ 11 ];
            Rec_high.array[ 2 ]  = GPSandHightValue[ 10 ];
            Rec_high.array[ 3 ]  = GPSandHightValue[ 9 ];
            
            /*
            BT_WriteString( "Rec_lontitude.Data"  );
            BT_WriteString( String( Rec_lontitude.Data, HEX )  );
            
            BT_WriteString( "Rec_latitude.Data"  );
            BT_WriteString( String( Rec_latitude.Data, HEX )  );
            
            BT_WriteString( "Rec_high.Data"  );
            BT_WriteString( String( Rec_high.Data, HEX )  );
            */
            
            GPSandHightCnt=0;
        }
    }
}
#endif
/*
    SerialEvent occurs whenever a new data comes in the
    hardware serial RX.  This routine is run between each
    time loop() runs, so using delay inside loop can delay
    response.  Multiple bytes of data may be available.
*/
void serialEvent1( )
{
    while ( Serial1.available( ) )
    {
        // get the new byte:
        unsigned char inChar = ( unsigned char )Serial1.read( );
        
        #if 0
        Serial1.print( "Receiver Byte : " );
        Serial1.print( inChar, HEX );
        Serial1.println( " " );
        #endif
        
        ucRxBuffer[ ucRxCnt++ ]= inChar;

        if( ucRxBuffer[ 0 ] != 0x55 )
        {
            ucRxCnt=0;
            return;
        }

        if( ucRxCnt < 11 )
        {
            //Serial.println( ucRxCnt );
            return;
        }
        else
        {
            switch( ucRxBuffer[ 1 ] )
            {
                case 0x50:  memcpy( &stcTime,   &ucRxBuffer[2], 8);break;
                case 0x51:  memcpy( &stcAcc,    &ucRxBuffer[2], 8);break;
                case 0x52:  memcpy( &stcGyro,   &ucRxBuffer[2], 8);break;
                case 0x53:  memcpy( &stcAngle,  &ucRxBuffer[2], 8);break;
                case 0x54:  memcpy( &stcMag,    &ucRxBuffer[2], 8);break;
                case 0x55:  memcpy( &stcDStatus,&ucRxBuffer[2], 8);break;
                case 0x56:  memcpy( &stcPress,  &ucRxBuffer[2], 8);break;
                case 0x57:  memcpy( &stcLonLat, &ucRxBuffer[2], 8);break;
                case 0x58:  memcpy( &stcGPSV,   &ucRxBuffer[2], 8);break;
                case 0x5F:  memcpy( &pwm_value, &ucRxBuffer[2], 8); break;
            }
            
            //PrintDebugMessage();

            ucRxCnt=0;
        }
    }
}

void PrintJY901B_Message( )
{
    #if 1
    BT_WriteString( "\r\nTime:" );
    BT_WriteString( String( stcTime.ucYear, DEC ) );
    BT_WriteString( "-" );
    BT_WriteString( String( stcTime.ucMonth, DEC ) );
    BT_WriteString("-");
    BT_WriteString( String( stcTime.ucDay, DEC ) );
    BT_WriteString("-");
    BT_WriteString( String( stcTime.ucHour, DEC ) );
    BT_WriteString(":");
    BT_WriteString( String( stcTime.ucMinute, DEC ) );
    
    BT_WriteString( "\r\nAcc:" );
    BT_WriteString( String( stcAcc.a[0], HEX ) );
    BT_WriteString( "," );
    BT_WriteString( String( stcAcc.a[1], HEX ) );
    BT_WriteString( "," );
    BT_WriteString( String( stcAcc.a[2], HEX ) );
    
    BT_WriteString( "\r\nGyro:" );
    BT_WriteString( String( ( float )stcGyro.w[0]/32768*2000, DEC ) );
    BT_WriteString( "," );
    BT_WriteString( String( ( float )stcGyro.w[1]/32768*2000, DEC ) );
    BT_WriteString( "," );
    BT_WriteString( String( ( float )stcGyro.w[2]/32768*2000, DEC ) );
    
    BT_WriteString( "\r\nAngle:" );
    BT_WriteString( String( (float)stcAngle.Angle[0]/32768*180, DEC ) );
    BT_WriteString( "," );
    BT_WriteString( String( (float)stcAngle.Angle[1]/32768*180, DEC ) );
    BT_WriteString( "," );
    BT_WriteString( String( (float)stcAngle.Angle[2]/32768*180, DEC ) );
    
    BT_WriteString( "\r\nMag:" );
    BT_WriteString( String( stcMag.h[0], DEC ) );
    BT_WriteString( "," );
    BT_WriteString( String( stcMag.h[1], DEC ) );
    BT_WriteString( "," );
    BT_WriteString( String( stcMag.h[2], DEC ) );
    
    BT_WriteString( "\r\nPressure:" );
    BT_WriteString( String( stcPress.lPressure, DEC ) ); 
    BT_WriteString( "," );
    BT_WriteString( String( (float)stcPress.lAltitude/100, DEC ) );
    
    BT_WriteString( "\r\nDStatus:" );
    BT_WriteString( String( stcDStatus.sDStatus[0], DEC ) );
    BT_WriteString( "," );
    BT_WriteString( String( stcDStatus.sDStatus[1], DEC ) );
    BT_WriteString( "," );
    BT_WriteString( String( stcDStatus.sDStatus[2], DEC ) );
    BT_WriteString( "," );
    BT_WriteString( String( stcDStatus.sDStatus[3], DEC ) );

    BT_WriteString( "\r\nLongitude:");
    BT_WriteString( String( stcLonLat.lLon/10000000, DEC ) );
    
    //BT_WriteString( String( (double)(stcLonLat.lLon % 10000000)/1e5, DEC ) );
    //BT_WriteString( "\r\nm Lattitude : ");
    //BT_WriteString( String( stcLonLat.lLat/10000000, DEC ) );
    
    //BT_WriteString( (double)(stcLonLat.lLat % 10000000)/1e5);

    BT_WriteString( "\r\nGPSHeight:" );
    BT_WriteString( String( (float)stcGPSV.sGPSHeight/10, DEC ) );
    BT_WriteString( "\r\nGPSYaw:" );
    BT_WriteString( String( (float)stcGPSV.sGPSYaw/10, DEC ) );
    BT_WriteString( "\r\nGPSV:" );
    BT_WriteString( String( (float)stcGPSV.lGPSVelocity/1000, DEC ) );
    #endif
    
    BT_WriteString( "\r\nReceiever_GPS:" );
    BT_WriteString( String( Rec_latitude.Data, DEC ) );
    BT_WriteString( "," );
    BT_WriteString( String( Rec_lontitude.Data, DEC ) );
    
    BT_WriteString( "\r\nReceiever_High:" );
    BT_WriteString( String( Rec_high.Data, DEC ) );
   
    #if 0
    BT_WriteString( "Acc:" );
    BT_WriteString( String( (float)stcAcc.a[0]/32768*16, HEX ) );
    BT_WriteString( "," );
    BT_WriteString( String( (float)stcAcc.a[1]/32768*16, HEX ) );
    BT_WriteString( "," );
    BT_WriteString( String( (float)stcAcc.a[2]/32768*16, HEX ) );
    #endif    
}

#if 1
void PrintDHT11_Message( int chk )
{

    BT_WriteString("\r\nRead_sensor:");
    
    switch (chk)
    {
        case DHTLIB_OK: 
            BT_WriteString("1"); 
            break;
        case DHTLIB_ERROR_CHECKSUM: 
            BT_WriteString("0"); 
            //BT_WriteString("Checksum error"); 
            break;
        case DHTLIB_ERROR_TIMEOUT: 
            BT_WriteString("0"); 
            //BT_WriteString( "Time out error"); 
            break;
        default:
            BT_WriteString("0"); 
            //BT_WriteString("Unknown error"); 
            break;
    }

    BT_WriteString( "\r\nHumidity(%):" );
    BT_WriteString( String( (float)DHT11.humidity, DEC ) );
    
    BT_WriteString( "\r\nTemperature(oC):" );
    BT_WriteString( String( (float)DHT11.temperature, DEC ) );
}
#endif



void Test( )
{
    static unsigned char tcnt = 0;
    
    //static unsigned char index = 0;
        
    //unsigned int tmp[18] = { 2000, 900 };
    
    //平行 960 1500
    
    long currentMillis = millis();
    
    if( ( currentMillis - previousMillis ) >= 1000 )
    {
        previousMillis = currentMillis;
        
        #if 0  
        if( index == 0 )
            index = 1;
        else
            index = 0;
        
        PWM_PinInit( 3, tmp[ index ] );
        
        PWM_PinInit( 5, tmp[ index ] );
        
        PWM_PinInit( 6, tmp[ index ] );
        
        PWM_PinInit( 9, tmp[ index ] ); 

        Serial1.println( "\r\nuart1 test" );
        #endif 
        
        tcnt ++;
    }
    
    if( tcnt == 5 )
    {
        tcnt = 0;
        
        // ad_data = Get_Adc_Average( 10 );  //250ms/次
        
        // BT_WriteString( "\r\nRead_ADC_Value:");
        
        // BT_WriteString( String( ad_data, DEC ) );
        
        // Serial1.println( "\r\nuart1_test" );
        
        int chk = DHT11.read( DHT11PIN );
        
        PrintDHT11_Message( chk );
        
        //BT_WriteString( "\r\n----------------------------" );
        
        PrintJY901B_Message( );
        
        //BT_WriteString( "\r\n----------------------------" );
    }
}

void setup()
{
    pinMode( 8, OUTPUT );
    pinMode( 7, INPUT );
    
    // Open serial communications and wait for port to open:
    Serial.begin( 9600 );              // Receiver GPS & H
    Serial1.begin( 9600 );             // JY901B Module
    BTSerial.begin( 9600 );
    
    Serial1.println( "Setup" );
    Serial1.println( "BT Uart Init Success" );
    
    stcGPSV.sGPSHeight = 0;
    stcGPSV.sGPSYaw = 0;
    stcGPSV.lGPSVelocity = 0;
       
    Adc_Init();
                                 
    Right_rudder_stand_point     = 514;       //右起始中立點
    Right_rudder_uppermost_sheet = 366;       //右最上點
    Right_rudder_lowermost_sheet = 633;       //右最下點
    Left_rudder_stand_point      = 551;       //左起始中立點
    Left_rudder_uppermost_sheet  = 717;       //左最上點
    Left_rudder_lowermost_sheet  = 422;       //左最下點
    wind_open                    = 459;
    wind_close                   = 569;
    hook_open                    = 1986;
    hook_close                   = 961;
    RU_difference = Right_rudder_stand_point-Right_rudder_uppermost_sheet;  //與基準比較減少 右向上與基準差值
    RD_difference = Right_rudder_lowermost_sheet-Right_rudder_stand_point;  //與基準比較增加 右向下與基準差值
    LU_difference = Left_rudder_uppermost_sheet-Left_rudder_stand_point;    //與基準比較增加 左向上與基準值差值
    LD_difference = Left_rudder_stand_point-Left_rudder_lowermost_sheet;    //與基準比較減少 左向下與基準值差值

    //PWM_PinInit( hookservo_pin, hook_actual );
    //PWM_PinInit( windservo_pin, wind_actual );
    //PWM_PinInit( Right_rudderservo_pin, Right_rudder_actual_value );    // 右舵 pwm
    //PWM_PinInit( Left_rudderservo_pin, Left_rudder_actual_value );      // 左舵 pwm
    
    // 等待JY901B初始化
    // wait for a second
    delay( 1000 );
    
    // time is used to toggle the LED is divided by i
    // set timer and callback
    // CurieTimerOne.start( oneSecInUsec / 1000 , &Time_1mSec_ISR );     
}

void loop( )
{
    //測試function
    Test();
    
    //Receiver_GPSandHightValue( );
    
    #if 0
    X = (float)stcAngle.Angle[0] / 32768*180;
    Y = (float)stcAngle.Angle[1] / 32768*180;
    Z = (float)stcAngle.Angle[2] / 32768*180;
    
    //h=(float)stcPress.lAltitude;
    
    acX = (float)stcAcc.a[0] / 32768 * 16;
    acZ = (float)stcAcc.a[2] / 32768 * 16;
    p   = stcPress.lPressure / 100;
    
    ad_data = Get_Adc_Average( 10 );  //250ms/次
    
    Moment_right_rudder1 = 0;//當下右舵百分比
    Left_rudder_moment1  = 0;//當下左舵百分比
    
    if( adkey == 1 )
    {
        airspeed( ad_data );
        
        if( h >= 990 && h <= 1010 )                //脫鉤加速階段
        {        
            x_rudder=0;
            y_rudder=0;
            hook_actual=hook_open;
        }
        
        if( h <= 5 )                               //landing
        {                                       
            wind_actual = 459;
            hook_actual = hook_close;
        }
        
        if( h > 5 && h <= 990 && real_ad <= 60 )   //stall
        {
            hook_actual = hook_close;                
            y_rudder    = -(90-ysum);
        }            
        
        if( h > 5 && real_ad >= 60  )
        {
            hook_actual=hook_close;
            
            if( real_ad < 70 )                     //空速
            {
                y_rudder = -( ( 70 - real_ad ) - y_rudder ); //空速至少大於80
            }
            
            if( y_rudder > 0 && real_ad > 70 )
            {
                wind_up_down();
            }
            
            dx = accx[ 1 ] - accx[ 0 ];
            dz = accz[ 1 ] - accz[ 0 ];
            
            digitalWrite( ledPin, led_status );           // sets the LED on
            
            led_status = ~led_status;
        
            if( pcount >= 85 )                      //navigation
            {
                //position();
                pcount=0;
            }
        }
    }
    
    //depart from ballon and speed is enough
    if( x_rudder > 1000 )   
        x_rudder=100;
    
    if( y_rudder > 100 )    
        y_rudder=100;
    
    if( x_rudder < -100 )   
        x_rudder=-100;
    
    if( y_rudder<-100 )     
        y_rudder=-100;
    
    Up_and_down_control( y_rudder, y_rudder );
    Fly_fly_left_and_right( x_rudder );
    The_actual_motor_pwm( );

    //補正舵角度
    PWM_PinInit( hookservo_pin, hook_actual );
    PWM_PinInit( windservo_pin, wind_actual );
    PWM_PinInit( Right_rudderservo_pin, Right_rudder_actual_value );    // 右舵 pwm
    PWM_PinInit( Left_rudderservo_pin, Left_rudder_actual_value );      // 左舵 pwm
    #endif
    
}

//右+100 -100 左+100 -100
void Up_and_down_control( int Up_and_down_Right,int Up_and_down_Left )
{
    if( Up_and_down_Right > 0 )
    {
        Moment_right_rudder1 = Up_and_down_Right+Moment_right_rudder1;
        
        if( Moment_right_rudder1 > 100 )
        {
            Moment_right_rudder1 = 100;
        }

    }
    else if( Up_and_down_Right < 0 )
    {
        Moment_right_rudder1=Up_and_down_Right+Moment_right_rudder1;
        if(Moment_right_rudder1<-100)
        {
            Moment_right_rudder1=-100;
        }
    }

    if( Up_and_down_Left > 0 )
    {
        Left_rudder_moment1=Up_and_down_Left+Left_rudder_moment1;
        
        if( Left_rudder_moment1 > 100 )
        {
            Left_rudder_moment1 = 100;
        }
    }
    else if( Up_and_down_Left < 0 )
    {
        Left_rudder_moment1 = Up_and_down_Left+Left_rudder_moment1;
        
        if( Left_rudder_moment1 < -100 )
        {
            Left_rudder_moment1 = -100;
        }
    }
}

//100右轉-100左轉
void Fly_fly_left_and_right(int Right_and_left_offset)
{
    if(Right_and_left_offset>0)//右轉
    {
        Moment_right_rudder1=Moment_right_rudder1+Right_and_left_offset;
        if(Moment_right_rudder1>100)
        {
            Moment_right_rudder1=100;
        }
        Left_rudder_moment1=Left_rudder_moment1-Right_and_left_offset;
        if(Left_rudder_moment1<-100)
        {
            Left_rudder_moment1=-100;
        }

    }
    else if(Right_and_left_offset<0)//左轉
    {
        Moment_right_rudder1=Right_and_left_offset+Moment_right_rudder1;
        if(Moment_right_rudder1<-100)
        {
            Moment_right_rudder1=-100;
        }
        Left_rudder_moment1=Left_rudder_moment1-Right_and_left_offset;
        if(Left_rudder_moment1>100)
        {
            Left_rudder_moment1=100;
        }

    }

}

void The_actual_motor_pwm(void)
{
    if(Moment_right_rudder1>0)
    {
        Moment_right_rudder=Moment_right_rudder1;
        Moment_right_rudder=Moment_right_rudder/100;
        Right_rudder_actual_value=Right_rudder_stand_point-(u16)(Moment_right_rudder*RU_difference);

    }
    else if(Moment_right_rudder1<0)
    {
        Moment_right_rudder=-Moment_right_rudder1;
        Moment_right_rudder=Moment_right_rudder/100;
        Right_rudder_actual_value=Right_rudder_stand_point+(u16)(Moment_right_rudder*RD_difference);
    }


    if(Left_rudder_moment1>0)
    {
        Left_rudder_moment=Left_rudder_moment1;
        Left_rudder_moment=Left_rudder_moment/100;
        Left_rudder_actual_value=Left_rudder_stand_point+(u16)(Left_rudder_moment*LU_difference);

    }
    else if(Left_rudder_moment1<0)
    {
        Left_rudder_moment=-Left_rudder_moment1;
        Left_rudder_moment=Left_rudder_moment/100;
        Left_rudder_actual_value=Left_rudder_stand_point-(u16)(Left_rudder_moment*LD_difference);
    }

}

#if 0
void fly_option( void )
{
    if( dx == 0 && dz == 0 )
    {
        x_rudder = x_rudder + 50;
    }
    if( dx == 0 && dz < -0.06 )
    {
        x_rudder=x_rudder+((-dz)*1000);
    }
    if(dx==0 && dz>0.06)
    {
        x_rudder=x_rudder+(dz*1000);
    }
    if(dx<-0.04 && dz==0)
    {
        x_rudder=x_rudder+((-dx)*1000);
    }
    if(dx<-0.04 && dz<-0.08)
    {
        x_rudder=x_rudder+((-dz)*1000);
    }
    if(dx<-0.04 && dz>0.06)
    {
        x_rudder=x_rudder+(dz*1000);
    }
    if(dx>0.04 && dz==0)
    {
        x_rudder=x_rudder-(dx*1000);
    }
    if(dx>0.04 && dz>0.06)
    {
        x_rudder=x_rudder-(dz*1000);
    }
    flag=0;
}
#endif

void wind_up_down(void)
{
    wind_actual = wind_close - ( wind_close - wind_open ) / 100 * x_rudder;
}

void position(void)
{
    lontitude=((double)stcLonLat.lLon/10000000+(double)(stcLonLat.lLon % 10000000)/1e5/60)*3.1415926/180;//現在經度
    latitude=((double)stcLonLat.lLat/10000000+(double)(stcLonLat.lLat % 10000000)/1e5/60)*3.1415926/180;//現在緯度
    
    dlontitude=lontitude_home-lontitude;                              
    dlatitude=latitude_home-latitude;
    
    azimuth=atan((dlontitude/dlatitude));
    azimuth=azimuth*180/3.1415926;
    
    if(dlontitude<0 && dlatitude>0)azimuth=360+azimuth;
    if(dlontitude<0 && dlatitude<0)azimuth=360-azimuth;
    if(dlontitude>0 && dlatitude<0)azimuth=180+azimuth;
    
    if(zcount==1)
    {
        temlon[0]=lontitude;
        temlat[0]=latitude;
    }
    
    if(zcount==3)
    {
        temlon[1]=lontitude;
        temlat[1]=latitude;
    }
    
    if(temlon[1]!=0 && temlat[1]!=0)                             //計算蟹形飛行
    {
        dlontitude=temlon[0]-temlon[1];
        dlatitude=temlat[0]-temlat[1];
        course=atan((dlontitude/dlatitude));
        course=course*180/3.1415926;
        if(dlontitude<0 && dlatitude>0)course=360+course;
        if(dlontitude<0 && dlatitude<0)course=360-course;
        if(dlontitude>0 && dlatitude<0)course=180+course;
    }
    
    //course=0;
    offset=azimuth-course;
    
    if(offset>180)
    {
        offset=offset-360;
    }
    
    if(offset<-180)
    {
        offset=360+offset;
    }
    
    clat=dlatitude*110.574*dlatitude*110.574;
    clon=dlontitude* 111.320*cos(latitude)*dlontitude* 111.320*cos(latitude);
    d=sqrt((clat+clon));
    nose_down=atan(h/d);
    y_rudder=-(y_rudder-nose_down);
    zcount++;
    
    if(zcount==5)
    {
        temlon[0]=temlon[1];
        temlon[1]=0;
        temlat[0]=temlat[1];
        temlat[1]=0;
        zcount=0;
    }
}
void wireless(int ad)
{
    if(tcount==2)
    {
        //Serial1.write( 0x66 );       //開頭
    }
    
    if(tcount==19)
    {
        //Serial1.write( stcPress.lPressure );     //發送氣壓資料給大氣電腦
    }
    
    if(adkey==0)
    {
        if(tcount==24 || tcount==49 || tcount==74 || tcount==99)
        {
            tempad[((tcount+1)/25)-1]=ad;
            if(tempad[3]!=0)
            {
                tempad[0]=tempad[0]+tempad[1];
                tempad[0]=tempad[0]+tempad[2];
                tempad[0]=tempad[0]+tempad[3];
                tempad[0]=tempad[0]/4;
                adkey=1;
            }
        }

    }
    
    if( tcount==39 )
    {
        //Serial1.write(h);      //發送高度資料給大氣電腦
    }
    
    if( tcount==59 )
    {
        //Serial1.write( stcLonLat.lLon );
    }
    
    if(tcount==79)
    {
        //Serial1.write( stcLonLat.lLat );
    }
    
    if(tcount==99)
    {
        //Serial1.write( 0x64 );
        tcount=0;
    }
}
void height(float pressure)
{
    w=1-(pressure/p);
    b=9.80665*1.225*t/(0.0065*pressure);
    w=pow(w,b);
    h=t/6.5*w;
}

void airspeed(int ad)
{
    real_ad=(ad-tempad[0])*10/15;
    if(real_ad<0)real_ad=0;
}

