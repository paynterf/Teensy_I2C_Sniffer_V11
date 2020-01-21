
/*
    Name:       Teensy_I2C_Sniffer_V11.ino
    Created:	1/18/2020 10:55:55 AM
    Author:     FRANKNEWXPS15\Frank
*/
/* 'Notes:

    A typical I2C sentence when communicating with a MPU6050 IMU module goes something like:
        "I2C(68) wrote 1 byte to 75 - C0 Done."
        "I2C(68) wrote 3 bytes to 72 - C0 0C 10 Done."
        "I2C(68) read 5 bytes from 6A - C0 0C 10 14 03 Done."

    To form a sentence, we need:
        Device addr: 68 in the above examples
        Read/Write direction
        To/From register address:  75, 72 and 6A in the above examples
        Data:  C0, C0 0C 10, and C0 0C 10 14 03 in the above examples
        number of bytes written/read:  1,3 & 5 in the above examples

     Each I2C communication proceeds as follows (assuming a START from an IDLE condition):
         A START or RESTART condition, denoted by SDA & SCL HIGH, followed by SDA LOW, SCL HIGH
         A 7-bit device address, MSB first (0x8/0xC = 1, 0x0/0x4 = 0)
         A R/W bit (0x8/0xC = read, 0x0/0x4 = write)
         An ACK bit (0x8/0xC = NAK, 0x0/0x4 = ACK)
         If the bus direction is WRITE, then
             A register address for read/write
             zero or more additional data bytes
         Else (the bus direction is READ)
            One or more additional data bytes
         Endif

    This version uses a fixed-size (2048 bytes) array instead of tonton81's circular buffer library.

    To generalize for any I2C slave device rather than just the MPU6050 IMU, comment out the 
    "#define MPU6050_SPECIFIC line below. This will remove all MPU6050 specific code
*/


//#define MPU6050_SPECIFIC

#include <TimerOne.h> //needed for ISR
#ifdef MPU6050_SPECIFIC
#include "helper_3dmath.h" //Arduino\Libraries\i2cdevlib\Arduino\MPU6050\ needed to compute yaw from MPU6050 DMP packet
#endif


//#define PARSE_LOOP_DEBUG

const uint16_t CAPTURE_ARRAY_SIZE = 2048;
const uint16_t VALID_DATA_ARRAY_SIZE = 2048;
const int WAITING_PRINT_INTERVAL_MSEC = 200;//interval timer for 'Waiting for data...' printout

#define MONITOR_OUT1 2 //so can monitor ISR activity with O'scope
#define MONITOR_OUT2 3 //so can monitor ISR activity with O'scope
#define MONITOR_OUT3 4 //so can monitor ISR activity with O'scope
#define SDA_PIN 18 
#define SCL_PIN 19

#pragma region PROCESSING_VARIABLES
uint8_t devAddr;
uint8_t regAddr;
uint8_t databytes[2048]; //holds multiple databytes for later output sentence construction
uint16_t numbytes = 0; //number of data bytes extracted from data stream
int ACKNAKFlag; //can be negative
uint16_t databyte_idx = 0; //index into databyte_array
uint8_t killbuff[2]; //used to consume start/stop bytes
elapsedMillis mSecSinceLastWaitingPrint;
uint8_t valid_data[2048];
uint16_t numvalidbytes = 0; //number of valid bytes in this burst
uint16_t read_idx = 0; //pointer to next byte pair to be processed

//added for bus direction labels
enum BUSDIR
{
    WRITE,
    READ,
    UNKNOWN = -1
} RWDir;
BUSDIR BusDir = BUSDIR::UNKNOWN;
#pragma endregion ProcVars


#pragma region ISR_SUPPORT
uint8_t raw_data[CAPTURE_ARRAY_SIZE]; //holds data captured from I2C bus
volatile uint16_t  write_idx = 0;
volatile uint8_t   current_portb = 0xFF;
volatile uint8_t   last_portb = 0xFF;
volatile uint16_t mult0xCCount = 0;
const uint16_t MAX_IDLE_COUNT = 2500;

volatile bool bDone = false;
volatile bool bWaitingForStart = true;
volatile bool bIsData = true;
volatile bool bIsStart = false;
volatile bool bIsStop = false;
volatile uint8_t last_current;
#pragma endregion ISR Support

#ifdef MPU6050_SPECIFIC
//01/21/20 these forward declarations are required to force the preprocessor 
//to handle #ifdef MPU6050_SPECIFIC properly
uint8_t dmpGetQuaternion(int16_t* data, const uint8_t* packet);
uint8_t dmpGetQuaternion(Quaternion* q, const uint8_t* packet);
uint8_t dmpGetYawPitchRoll(float* data, Quaternion* q, VectorFloat* gravity);
uint8_t dmpGetGravity(VectorFloat* v, Quaternion* q);

Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
float global_yawval = 0; //updated by GetIMUHeadingDeg()
#endif // MPU6050_SPECIFIC

void setup()
{
    unsigned long now = millis();
    Serial.begin(1); //rate value ignored
    int idx = 0;
    while (!Serial && (millis() - now) < 3000)
    {
        delay(500);
        idx++;
    }
    Serial.printf("Serial available after %lu mSec\n", millis() - now);

    pinMode(MONITOR_OUT1, OUTPUT);
    digitalWrite(MONITOR_OUT1, LOW);
    pinMode(MONITOR_OUT2, OUTPUT);
    digitalWrite(MONITOR_OUT2, LOW);
    pinMode(MONITOR_OUT3, OUTPUT);
    digitalWrite(MONITOR_OUT3, LOW);

    pinMode(SCL_PIN, INPUT);
    pinMode(SDA_PIN, INPUT);

    //reset port byte vars & start timer
    last_portb = current_portb = 0;
    write_idx = 0;
    memset(raw_data, 255, CAPTURE_ARRAY_SIZE);
    //PrintNextArrayBytes(raw_data, 255, 20);
    Timer1.initialize(1); // run every mico second
    Timer1.attachInterrupt(capture_data);


    mSecSinceLastWaitingPrint = 0;
}
//-------------------------------------------------------------------------------
//--------------------------------    ISR    ------------------------------------
//-------------------------------------------------------------------------------
FASTRUN void capture_data()
//void capture_data()
{
    last_portb = current_portb;
    current_portb = GPIOB_PDIR & 12; //reads state of SDA (18) & SCL (19) at same time

    if (!bDone && last_portb != current_portb)
    {
        mult0xCCount = 0; //reset IDLE counter
        digitalWriteFast(MONITOR_OUT1, HIGH);

        //01/17/20: joepasquariello suggestion
        last_current = (last_portb << 4) | (current_portb);
        bIsStart = (last_current == 0xC4);
        bIsStop = (last_current == 0x4C);
        bIsData = (last_current == 0x04) || (last_current == 0x8C);

        if (bIsStart) //START  
        {
            digitalWriteFast(MONITOR_OUT2, HIGH);
            if (bWaitingForStart)
            {
                digitalWriteFast(MONITOR_OUT3, HIGH); //start of entire capture
                bWaitingForStart = false;
            }
        }
        else if (bIsStop) //STOP
        {
            digitalWriteFast(MONITOR_OUT2, LOW);
        }

        if (!bWaitingForStart && (bIsData || bIsStart || bIsStop))
        {
            //digitalWriteFast(MONITOR_OUT3, HIGH);
            raw_data[write_idx] = last_portb;
            write_idx++;
            raw_data[write_idx] = current_portb;
            write_idx++;
            if (write_idx >= CAPTURE_ARRAY_SIZE)
            {
                bDone = true;
                digitalWriteFast(MONITOR_OUT3, LOW);
            }
        }
        digitalWriteFast(MONITOR_OUT1, LOW);
    }
    else if (!bDone && mult0xCCount < MAX_IDLE_COUNT && last_portb == 0xc && current_portb == 0xc)
    {
        mult0xCCount++;
        if (mult0xCCount >= MAX_IDLE_COUNT)
        {
            digitalWriteFast(MONITOR_OUT3, LOW);
            bDone = true;
        }
    }
}
//-------------------------------------------------------------------------------
//-------------------------------- END ISR    ---------------------------------
//-------------------------------------------------------------------------------

void loop()
{
    if (bDone)
    { 
        if (write_idx > 14)
        {
            //OK, we have some data to process. IDLE detection must have been EOM
            Timer1.stop();

            unsigned long startMsec = millis();

            //Serial.printf("%lu\t %d\t", millis(), write_idx);
            //PrintNextArrayBytes(raw_data, 0, 50);
            //Serial.printf(" - %lu\n", millis());
            uint16_t numprocessed = DecodeAndPrintValidData(raw_data); //decode and print everything captured so far
            unsigned long endMsec = millis();
            Serial.printf("%lu: processed = %d elements in %lu mSec\n\n", startMsec, numprocessed, endMsec-startMsec);

            Timer1.start();
        }

        read_idx = 0;
        bDone = false;
        mult0xCCount = 0;
        write_idx = 0;
        bWaitingForStart = true;
    }    
    else
    {
        //no data to process, but don't blow prints out every mSec...
        if (mSecSinceLastWaitingPrint > WAITING_PRINT_INTERVAL_MSEC)
        {
            mSecSinceLastWaitingPrint -= WAITING_PRINT_INTERVAL_MSEC;
            Serial.printf("%lu: Waiting for Data...\n", millis());
        }
    }
}

void PrintNextArrayBytes(uint8_t* data, uint16_t startidx, uint16_t numbytes)
{
    Serial.printf("%d bytes starting at %d: ", numbytes, startidx);
    for (uint16_t i = 0; i < numbytes; i++)
    {
        Serial.printf("%x ", data[i + startidx]);
    }
}


uint16_t DecodeAndPrintValidData(byte* data)
{
    //Purpose:  decode and print I2C conversation held in raw_data array
    //Inputs:  
    //  cb = 2048 element FIFO
    //Outputs:
    //  returns number of bytes processed, or -1 for failure
    //  outputs structured I2C sentence to serial monitor
    //Plan:
    //  Step1: Cull out invalid bytes
    //  Step2: Determine if there is anything to do (have to have more than one transition in FIFO)
    //  Step3: Parse transitions into I2C sentence structure
    //  Step4: Output sentence to serial monitor

    memset(valid_data, 0, VALID_DATA_ARRAY_SIZE);
#ifdef PARSE_LOOP_DEBUG
    PrintNextArrayBytes(valid_data, 0, 20); //print out first 20 bytes for verification
#endif
    numvalidbytes = RemoveInvalidBytes(raw_data, valid_data);
#ifdef PARSE_LOOP_DEBUG
    Serial.printf("Removed %d invalid bytes, leaving %d remaining\n", write_idx + 1 - numvalidbytes, numvalidbytes);
    PrintNextArrayBytes(valid_data, 0, numvalidbytes); //print out first 20 bytes of valid_data array
#endif


    if (numvalidbytes < 2)
    {
        return 0;
    }

    while (read_idx < numvalidbytes)
    {
#ifdef PARSE_LOOP_DEBUG
        Serial.printf("At top of while (read_idx < numvalidbytes): read_idx = %d\n", read_idx);
        Serial.printf("Next two bytes in valid_data are %x, %x\n", valid_data[read_idx], valid_data[read_idx + 1]);
#endif
        //Find a START sequence (0xC followed by 0x4)
        while (!IsStart(valid_data, read_idx) && read_idx < numvalidbytes)
        {
            //Serial.printf("looking for start...\n");
            read_idx++;
        }
        //at this point, read_idx should point to next valid byte pair

#ifdef PARSE_LOOP_DEBUG
        Serial.printf("Start sequence found at %d\n", read_idx - 2);
        //PrintNextFIFOBytes(valid_data, 20);
#endif

        if (numvalidbytes - read_idx > 14)//14 entries required for 7-bit address
        {
            //Get 7-bit device address
            devAddr = Get7BitDeviceAddr(valid_data, read_idx);
#ifdef PARSE_LOOP_DEBUG
            Serial.printf("devAddr = %x\n", devAddr);
#endif
        }
        else
        {

#ifdef PARSE_LOOP_DEBUG
            Serial.printf("ran out of data at readidx = %d - exiting!\n", read_idx);
#endif
            break;
        }

        //get read/write flag  1 = Read, 0 = Write, -1 = error
        BusDir = (BUSDIR)GetReadWriteFlag(valid_data, read_idx);

#ifdef PARSE_LOOP_DEBUG
        Serial.printf("BusDir = %s\n", ((BusDir == BUSDIR::WRITE) ? "WRITE" : "READ"));
        //PrintNextFIFOBytes(valid_data, 20);
#endif

    //get ACK/NAK flag
        ACKNAKFlag = GetACKNAKFlag(valid_data, read_idx);
        numbytes = GetDataBytes(valid_data, read_idx, databytes); //terminates on a START, but the start bytes are not consumed
#ifdef PARSE_LOOP_DEBUG
        Serial.printf("Got %d bytes from GetDataBytes() --> ", numbytes);
        for (size_t i = 0; i < numbytes; i++)
        {
            Serial.printf(" %x ", databytes[i]);
        }
        Serial.printf("\n");

        //PrintNextFIFOBytes(cb_trans, 20);
#endif
    //If the bus direction is WRITE, then extract
    //    A register address for read / write
    //    zero or more additional data bytes
        if (BusDir == BUSDIR::WRITE)
        {
            regAddr = databytes[0];
#ifdef PARSE_LOOP_DEBUG
            Serial.printf("regAddr = %x, read_idx = %d\n", regAddr, read_idx);
#endif

            //check for additional data
            if (numbytes > 1)
            {
#ifdef PARSE_LOOP_DEBUG
                Serial.printf("Additional data found!\n");
                for (size_t i = 0; i < numbytes; i++)
                {
                    Serial.printf("data[%d] = %x\n", i, databytes[i]);
                }
#endif
                //1st byte is register addr, subsequent bytes are data
                OutputFormattedSentence(BusDir, devAddr, regAddr, numbytes, databytes, 1);
            }
        }
        else  //all bytes are data
        {
#ifdef PARSE_LOOP_DEBUG
            Serial.printf("In data block:  got %d bytes of data\n", numbytes);
            for (size_t i = 0; i < numbytes; i++)
            {
                Serial.printf("data[%d] = %x\n", i, databytes[i]);
            }
#endif
            OutputFormattedSentence(BusDir, devAddr, regAddr, numbytes, databytes, 0);
        }
#ifdef PARSE_LOOP_DEBUG
        Serial.printf("At end of while (read_idx < numvalidbytes): read_idx = %d\n", read_idx);
#endif

    }//while (read_idx < numvalidbytes)
    return numvalidbytes;
}


#pragma region Support Functions
bool IsStart(byte* data, uint16_t& readidx)
{
    bool result = false;

    //Serial.printf("IsStart[%d] = %x, IsStart[%d] = %x\n",
    //    readidx, data[readidx], readidx + 1, data[readidx + 1]);

    if (data[readidx] == 0xC && data[readidx + 1] == 0x4)
    {
        result = true;
        readidx += 2; //bump to next byte pair
    }
    return result;
}

bool IsStop(byte* data, uint16_t& readidx)
{
    bool result = false;

    //Serial.printf("IsStop[%d] = %x, IsStop[%d] = %x\n",
        //readidx, data[readidx], readidx + 1, data[readidx + 1]);

    if (data[readidx] == 0x4 && data[readidx + 1] == 0xC)
    {
        result = true;
        readidx += 2; //bump to next byte pair
    }
    return result;
}

uint8_t Get7BitDeviceAddr(byte* data, uint16_t& readidx)
{
    //Purpose: Construct a 7-bit address starting from dataidx
    //Inputs:
    //  data = pointer to valid data array
    //  readidx = starting index of 7-bit address sequence (MSB first)
    //Outputs:
    //  returns the address as an 8-bit value with the MSB = 0, or 0x0 if unsuccessful
    //  dataidx = pointer to next data entry
    //Plan:
    //  Step1: Convert a pair of data entries into a 0 or 1
    //  Step2: Add the appropriate value to an ongoing sum
    //  Step3: return the total.
    //Notes:
    //  A '0' is coded as a 0x0 followed by a 0x4
    //  A '1' is coded as a 0x8 followed by a 0xC

    uint8_t devAddr = 0x0; //failure return value

    //Serial.printf("Get7BitDeviceAddr: readidx = %d\n",readidx);

    //devAddr is exactly 7 bits long, so 8 bits with MSB = 0
    for (size_t i = 0; i < 7; i++)
    {
        if (data[readidx] == 0x0 && data[readidx + 1] == 0x4)
        {
            readidx += 2; //advance the pointer, but don't add to sum
        }

        else if (data[readidx] == 0x8 && data[readidx + 1] == 0xC)
        {
            //Serial.printf("Get7BitDeviceAddr: '1' found at i = %d, adding %x to devAddr to get %x\n",
            //    i, 1 << (7 - i), devAddr + (1 << (7-i)));

            readidx += 2; //advance the pointer
            devAddr += (1 << (7 - i)); //add 2^(7-i) to sum
        }
    }

    devAddr = devAddr >> 1; //divide result by 2 to get 7-bit addr from 8 bits
    return devAddr;
}

int Get8BitDataByte(byte* data, uint16_t& readidx)
{
    //Purpose: Construct a 8-bit data byte starting from dataidx
    //Inputs:
    //  data = pointer to valid data array
    //  readidx = starting index of 8-bit data byte (MSB first)
    //Outputs:
    //  returns the address as an 8-bit value, or 0x0 if unsuccessful
    //  dataidx = pointer to next data entry
    //Plan:
    //  Step1: Convert a pair of data entries into a 0 or 1
    //  Step2: Add the appropriate value to an ongoing sum
    //  Step3: return the total.
    //Notes:
    //  A '0' is coded as a 0x0 followed by a 0x4
    //  A '1' is coded as a 0x8 followed by a 0xC
    //  12/29/19 - changed return val to int, so can return -1 when a 'short byte' is detected

    int dataval = 0x0; //failure return value

#ifdef GET_8BIT_DATABYTE_DEBUG
    Serial.printf("Get8BitDataByte: data[%d] = %x, data[%d] = %x\n",
        readidx, data[readidx], readidx + 1, data[readidx + 1]);
#endif

    //8 bits with MSB = 0
    int numbytes = 0;
    for (size_t i = 0; i < 8; i++)
    {
        if (data[readidx] == 0x0 && data[readidx + 1] == 0x4)
        {
            readidx += 2; //advance the pointer, but don't add to sum
            numbytes++;
        }

        else if (data[readidx] == 0x8 && data[readidx + 1] == 0xC)
        {
#ifdef GET_8BIT_DATABYTE_DEBUG
            Serial.printf("Get8BitDataByte: '1' found at i = %d, adding %x to devAddr to get %x\n",
                i, 1 << (7 - i), dataval + (1 << (7 - i)));
#endif
            readidx += 2; //advance the pointer
            dataval += (1 << (7 - i)); //add 2^(8-i) to sum
            numbytes++;
        }
    }

#ifdef GET_8BIT_DATABYTE_DEBUG
    Serial.printf("Get8BitDataByte: numbytes = %d\n", numbytes);
#endif
    if (numbytes != 8)
    {
        dataval = -1; //error return value
    }

    return dataval;
}

int GetReadWriteFlag(byte* data, uint16_t& readidx)
{
    //Purpose: decode R/W byte pair
    //Inputs:
    //  data = pointer to valid data array
    //  readidx = index into data to start of R/W byte pair
    //Outputs:
    //  readidx = if successful, points to next byte pair in data
    //  returns 1 for Read (0x8/0xC), 0 for Write (0x0/0x4), -1 for failure
    //Notes:
    //  

    //Serial.printf("GetReadWriteFlag: readidx = %d, data[readidx] = %x, data[readidx+1]= %x\n",
    //    readidx, data[readidx], data[readidx + 1]);
    int result = 0;
    if (data[readidx] == 0x8 && data[readidx + 1] == 0xC)
    {
        result = 1; //read detected
        readidx += 2; //point to next byte pair
    }

    else if (data[readidx] == 0x0 && data[readidx + 1] == 0x4)
    {
        result = 0; //write detected
        readidx += 2; //point to next byte pair
    }
    else
    {
        result = -1; //failed to detect read or write
    }

    return result;
}

int GetACKNAKFlag(byte* data, uint16_t& readidx)
{
    //Purpose: decode ACK/NAK byte pair
    //Inputs:
    //  data = pointer to valid data array
    //  readidx = index into data to start of ACK/NAK byte pair
    //Outputs:
    //  readidx = if successful, points to next byte pair in data
    //  returns 1 for NAK (0x8/0xC), 0 for ACK (0x0/0x4), -1 for failure
    //Notes:
    //  

    //Serial.printf("GetACKNAKFlag: readidx = %d, data[readidx] = %x, data[readidx+1]= %x\n",
    //    readidx, data[readidx], data[readidx + 1]);
    int result = 0;
    if (data[readidx] == 0x8 && data[readidx + 1] == 0xC)
    {
        result = 1; //NAK detected
        readidx += 2; //point to next byte pair
    }

    else if (data[readidx] == 0x0 && data[readidx + 1] == 0x4)
    {
        result = 0; //ACK detected
        readidx += 2; //point to next byte pair
    }
    else
    {
        result = -1; //failed to detect ACK or NAK
    }

    return result;
}

int GetDataBytes(uint8_t* data, uint16_t& readidx, uint8_t* databytes)
{
    //Notes:
    //  01/01/2020: removed databyteidx from sig - always starts at zero

    uint16_t numbytes = 0;
    uint16_t databyte_idx = 0;

    bool StartFlag = false;
    bool StopFlag = false;

    do
    {
        int dataval = Get8BitDataByte(data, readidx);

        //watch out for 'short byte' reads
        if (dataval >= 0)
        {
            uint8_t databyte = (uint8_t)dataval;
            databytes[databyte_idx] = databyte;
            databyte_idx++;
            numbytes++;
        }

        ACKNAKFlag = GetACKNAKFlag(data, readidx);
        StartFlag = IsStart(data, readidx);
        StopFlag = IsStop(data, readidx);

#ifdef PARSE_LOOP_DEBUG
        Serial.printf("IsStart returned %d, IsStop returned %d, dataidx = %d\n",
            StartFlag, StopFlag, readidx);
#endif

    } while (!StartFlag && !StopFlag && readidx < numvalidbytes);


    readidx -= 2;//back readidx up so loop top is positioned correctly.

    return numbytes;
}

void OutputFormattedSentence(int RW, uint8_t dev, uint8_t reg, uint8_t numbytes, uint8_t* bytearray, uint16_t startidx)
{
    Serial.printf("%lu I2C(%x) %s %d bytes %s %x... ",
        millis(), dev, (RW == 0 ? "writing" : "reading"),  numbytes - startidx, (RW == 0 ? "to" : "from"), reg);
    for (size_t i = startidx; i < numbytes; i++)
    {
        Serial.printf("%x ", bytearray[i]);
    }
    Serial.printf(". Done\n");

//#ifdef MPU6050_SPECIFIC
//
//
//    //01/18/20 experiment to decode 28-byte packet into yaw value
//    if (numbytes == 28)
//    {
//        dmpGetQuaternion(&q, bytearray);
//        dmpGetGravity(&gravity, &q);
//        dmpGetYawPitchRoll(ypr, &q, &gravity);
//
//        //compute the yaw value
//        global_yawval = ypr[0] * 180 / M_PI;
//        Serial.printf("yawval = %3.2f\n", global_yawval);
//    }
//    else
//    {
//        Serial.printf(". Done\n");
//    }
//#endif // MPU6050_SPECIFIC



}

uint16_t RemoveInvalidBytes(uint8_t* rawdata, uint8_t* validdata)
{
    uint16_t numvalid = 0;
    uint16_t valididx = 0;

    //Serial.printf("raw data array contains %d bytes\n", write_idx + 1);
    //PrintNextArrayBytes(raw_data, 0, 20);

    //OK, now go back through the array, excising invalid sequences
    for (uint16_t rawidx = 0; rawidx < write_idx;/*rawidx incremented internally*/)
    {
        uint8_t firstByte = raw_data[rawidx]; //get the first byte
        uint8_t secondByte = raw_data[rawidx + 1]; //get the next byte
        bool validpair =
            (
            (firstByte == 0xC && secondByte == 0x4) //START or RESTART
                || (firstByte == 0x4 && secondByte == 0xC) //STOP
                || (firstByte == 0x0 && secondByte == 0x4) //0 OR ACK
                || (firstByte == 0x8 && secondByte == 0xC) //1 or NAK
                );

        //Serial.printf("rawidx %d: Considering %x and %x: validity = %d\n",
            //rawidx, firstByte, secondByte, validpair);
        if (validpair)
        {
            //save valid bytes to valid_bytes array
            validdata[valididx] = firstByte;
            validdata[valididx + 1] = secondByte;
            numvalid += 2;
            //Serial.printf("Added %x & %x at idx = %d & %d\n", firstByte, secondByte, valididx, valididx + 1);
            //PrintNextArrayBytes(validdata,0,numvalid);
            rawidx += 2;
            valididx += 2;
        }
        else
        {
            rawidx++; //on invalid, just go to next byte
        }
    }

    return numvalid;
}
#pragma endregion Support Functions

#ifdef MPU6050_SPECIFIC

//#pragma region YAW_COMPUTATIONS
////01/18/2020: I copied these functions from MPU6050_6Axis_MotionApps_V6_12.h and
////  modified them to be called directly instead of from an 'mpu' object
//    
uint8_t dmpGetQuaternion(int16_t* data, const uint8_t* packet) 
{
    // TODO: accommodate different arrangements of sent data (ONLY default supported now)
    if (packet != 0) 
    {
        data[0] = ((packet[0] << 8) | packet[1]);
        data[1] = ((packet[4] << 8) | packet[5]);
        data[2] = ((packet[8] << 8) | packet[9]);
        data[3] = ((packet[12] << 8) | packet[13]);
    }
    return 0;
}

uint8_t dmpGetQuaternion(Quaternion* q, const uint8_t* packet) 
{
    // TODO: accommodate different arrangements of sent data (ONLY default supported now)
    int16_t qI[4]{ 0,0,0,0 };
    uint8_t status = dmpGetQuaternion(qI, packet);
    if (status == 0) {
        q->w = (float)qI[0] / 16384.0f;
        q->x = (float)qI[1] / 16384.0f;
        q->y = (float)qI[2] / 16384.0f;
        q->z = (float)qI[3] / 16384.0f;
        return 0;
    }
    return status; // int16 return value, indicates error if this line is reached
}

uint8_t dmpGetYawPitchRoll(float* data, Quaternion* q, VectorFloat* gravity) 
{
    // yaw: (about Z axis)
    data[0] = atan2(2 * q->x * q->y - 2 * q->w * q->z, 2 * q->w * q->w + 2 * q->x * q->x - 1);
    // pitch: (nose up/down, about Y axis)
    data[1] = atan2(gravity->x, sqrt(gravity->y * gravity->y + gravity->z * gravity->z));
    // roll: (tilt left/right, about X axis)
    data[2] = atan2(gravity->y, gravity->z);
    if (gravity->z < 0) {
        if (data[1] > 0) {
            data[1] = PI - data[1];
        }
        else {
            data[1] = -PI - data[1];
        }
    }
    return 0;
}


uint8_t dmpGetGravity(VectorFloat* v, Quaternion* q) 
{
    v->x = 2 * (q->x * q->z - q->w * q->y);
    v->y = 2 * (q->w * q->x + q->y * q->z);
    v->z = q->w * q->w - q->x * q->x - q->y * q->y + q->z * q->z;
    return 0;
}
//#pragma endregion YAW_COMPUTATIONS
#endif //MPU6050_SPECIFIC
