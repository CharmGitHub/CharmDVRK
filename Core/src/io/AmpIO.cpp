/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  $Id: AmpIO.cpp 78 2013-04-25 23:47:40Z pkazanz1 $

  Author(s):  Zihan Chen, Peter Kazanzides

  (C) Copyright 2011-2012 Johns Hopkins University (JHU), All Rights Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---
*/

#include <byteswap.h>
#include <unistd.h>
#include <iostream>
#include <sstream>

#include "io/AmpIO.h"
#include "io/FirewirePort.h"

const AmpIO_UInt32 VALID_BIT        = 0x80000000;  /*!< High bit of 32-bit word */
const AmpIO_UInt32 MIDRANGE_ADC     = 0x00008000;  /*!< Midrange value of ADC bits */
const AmpIO_UInt32 MIDRANGE_VEL     = 0x00008000;  /*!< Midrange value of encoder velocity */
const AmpIO_UInt32 MIDRANGE_FRQ     = 0x00008000;  /*!< Midrange value of encoder frequency */
const AmpIO_UInt32 MIDRANGE_ACC     = 0x00008000;  /*!< Midrange value of encoder acc */
const AmpIO_UInt32 ENC_PRELOAD      = 0x007fffff;  /*!< Encoder preload value */

const AmpIO_UInt32 PWR_ENABLE       = 0x000c0000;  /*!< Turn pwr_en on             */
const AmpIO_UInt32 PWR_DISABLE      = 0x00080000;  /*!< Turn pwr_en off            */
const AmpIO_UInt32 RELAY_ON         = 0x00030000;  /*!< Turn safety relay on       */
const AmpIO_UInt32 RELAY_OFF        = 0x00020000;  /*!< Turn safety relay off      */
const AmpIO_UInt32 ENABLE_MASK      = 0x0000ffff;  /*!< Mask for power enable bits */
const AmpIO_UInt32 MOTOR_CURR_MASK  = 0x0000ffff;  /*!< Mask for motor current adc bits */
const AmpIO_UInt32 ANALOG_POS_MASK  = 0xffff0000;  /*!< Mask for analog pot ADC bits */
const AmpIO_UInt32 ADC_MASK         = 0x0000ffff;  /*!< Mask for right aligned ADC bits */
const AmpIO_UInt32 DAC_MASK         = 0x0000ffff;  /*!< Mask for 16-bit DAC values */
const AmpIO_UInt32 ENC_POS_MASK     = 0x01ffffff;  /*!< Mask for quad encoder bits */
const AmpIO_UInt32 ENC_VEL_MASK     = 0x0000ffff;  /*!< Mask for encoder velocity bits */
const AmpIO_UInt32 ENC_FRQ_MASK     = 0x0000ffff;  /*!< Mask for encoder frequency bits */

const AmpIO_UInt32 DAC_WR_A         = 0x00300000;  /*!< Command to write DAC channel A */


// PROGRESS_CALLBACK: inform the caller when the software is busy waiting: in this case,
//                    the parameter is NULL, but the function returns an error if
//                    the callback returns false.
// ERROR_CALLBACK:    inform the caller of an error; in this case, the error message
//                    (char *) is passed as a parameter, and the return value is ignored.

#define PROGRESS_CALLBACK(CB, ERR)             \
    if (CB) { if (!(*CB)(0)) return ERR; }     \
    else std::cout << '.';

#define ERROR_CALLBACK(CB, MSG)         \
    if (CB) (*CB)(MSG.str().c_str());   \
    else { std::cout << MSG.str() << std::endl; }


AmpIO::AmpIO(AmpIO_UInt8 board_id, unsigned int numAxes) : BoardIO(board_id), NumAxes(numAxes)
{
    memset(read_buffer, 0, sizeof(read_buffer));
    memset(write_buffer, 0, sizeof(write_buffer));
    // Set members in base class (in the future, some of these may be set from
    // the FirewirePort class, when the board is added)
    ReadBufferSize = sizeof(read_buffer);
    ReadBuffer = read_buffer;
    WriteBufferSize = sizeof(write_buffer);
    WriteBuffer = write_buffer;
}

AmpIO::~AmpIO()
{
    if (port) {
        std::cerr << "Warning: AmpIO being destroyed while still in use by FirewirePort" << std::endl;
        port->RemoveBoard(this);
    }
}

AmpIO_UInt32 AmpIO::GetFirmwareVersion() const
{
    return (port ? port->GetFirmwareVersion(BoardId) : 0);
}

void AmpIO::DisplayReadBuffer(std::ostream &out) const
{
    // first two quadlets are timestamp and status, resp.
    out << std::hex << bswap_32(read_buffer[0]) << std::endl;
    out << std::hex << bswap_32(read_buffer[1]) << std::endl;
    // next two quadlets are digital I/O and amplifier temperature
    out << std::hex << bswap_32(read_buffer[2]) << std::endl;
    out << std::hex << bswap_32(read_buffer[3]) << std::endl;

    // remaining quadlets are in 4 groups of NUM_CHANNELS as follows:
    //   - motor current and analog pot per channel
    //   - encoder position per channel
    //   - encoder velocity per channel
    //   - encoder frequency per channel
    for (int i=4; i<ReadBufSize; i++) {
        out << std::hex << bswap_32(read_buffer[i]) << " ";
        if (!((i-1)%NUM_CHANNELS)) out << std::endl;
    }
    out << std::dec;
}

AmpIO_UInt32 AmpIO::GetStatus(void) const
{
    return bswap_32(read_buffer[STATUS_OFFSET]);
}

AmpIO_UInt32 AmpIO::GetTimestamp(void) const
{
    return bswap_32(read_buffer[TIMESTAMP_OFFSET]);
}

AmpIO_UInt32 AmpIO::GetDigitalInput(void) const
{
    return bswap_32(read_buffer[DIGIO_OFFSET])&0x00000fff;
}

AmpIO_UInt8 AmpIO::GetDigitalOutput(void) const
{
    return static_cast<AmpIO_UInt8>((bswap_32(read_buffer[DIGIO_OFFSET])>>12)&0x000f);
}

AmpIO_UInt8 AmpIO::GetNegativeLimitSwitches(void) const
{
    return (this->GetDigitalInput()&0x0f00)>>8;
}

AmpIO_UInt8 AmpIO::GetPositiveLimitSwitches(void) const
{
    return (this->GetDigitalInput()&0x00f0)>>4;
}

AmpIO_UInt8 AmpIO::GetHomeSwitches(void) const
{
    return (this->GetDigitalInput()&0x00f);
}

AmpIO_UInt8 AmpIO::GetAmpTemperature(unsigned int index) const
{
    AmpIO_UInt8 temp = 0;
    if (index == 0)
        temp = (bswap_32(read_buffer[TEMP_OFFSET])>>8) & 0x000000ff;
    else if (index == 1)
        temp = bswap_32(read_buffer[TEMP_OFFSET]) & 0x000000ff;
    return temp;
}

AmpIO_UInt32 AmpIO::GetMotorCurrent(unsigned int index) const
{
    if (index >= NUM_CHANNELS)
        return 0L;

    quadlet_t buff;
    buff = bswap_32(read_buffer[index+MOTOR_CURR_OFFSET]);
    buff &= MOTOR_CURR_MASK;       // mask for applicable bits

    return static_cast<AmpIO_UInt32>(buff) & ADC_MASK;
}

AmpIO_UInt32 AmpIO::GetAnalogInput(unsigned int index) const
{
    if (index >= NUM_CHANNELS)
        return 0L;

    quadlet_t buff;
    buff = bswap_32(read_buffer[index+ANALOG_POS_OFFSET]);
    buff &= ANALOG_POS_MASK;       // mask for applicable bits
    buff >>= 16;                   // shift to lsb alignment

    return static_cast<AmpIO_UInt32>(buff) & ADC_MASK;
}

AmpIO_UInt32 AmpIO::GetEncoderPosition(unsigned int index) const
{
    if (index < NUM_CHANNELS)
        return bswap_32(read_buffer[index+ENC_POS_OFFSET]);
    else
        return 0;
}

// temp current the enc period velocity is unsigned 16 bits
// for low level function the + MIDRANGE_VEL
AmpIO_UInt32 AmpIO::GetEncoderVelocity(unsigned int index) const
{
    // buff is a signed 16 bits counter
    // returns how many clock counts per encoder tick
    // Clock = 768 kHz
    // stored in a 32 bit unsiged int
    if (index >= NUM_CHANNELS)
        return 0L;

    quadlet_t buff;
    buff = bswap_32(read_buffer[index+ENC_VEL_DT_OFFSET]);
    buff &= ENC_VEL_MASK;          // mask for applicable bits

    return static_cast<AmpIO_UInt32>(buff) & ENC_VEL_MASK;
}


bool AmpIO::GetPowerStatus(void) const
{
    // Bit 19: MV_GOOD
    return (GetStatus()&0x00080000);
}

bool AmpIO::GetSafetyRelayStatus(void) const
{
    // Bit 17
    return (GetStatus()&0x00020000);
}

bool AmpIO::GetWatchdogTimeoutStatus(void) const
{
    // Bit 23
    return (GetStatus()&0x00800000);
}

bool AmpIO::GetAmpEnable(unsigned int index) const
{
    if (index >= NUM_CHANNELS)
        return false;
    AmpIO_UInt32 mask = (0x00000001 << index);
    return GetStatus()&mask;
}

bool AmpIO::GetAmpStatus(unsigned int index) const
{
    if (index >= NUM_CHANNELS)
        return false;
    AmpIO_UInt32 mask = (0x00000100 << index);
    return GetStatus()&mask;
}


/*******************************************************************************
 * Set commands
 */

void AmpIO::SetPowerEnable(bool state)
{
    AmpIO_UInt32 enable_mask = bswap_32(0x00080000);
    write_buffer[WB_CTRL_OFFSET] |=  enable_mask;
    AmpIO_UInt32 state_mask  = bswap_32(0x00040000);
    if (state)
        write_buffer[WB_CTRL_OFFSET] |=  state_mask;
    else
        write_buffer[WB_CTRL_OFFSET] &= ~state_mask;
}

bool AmpIO::SetAmpEnable(unsigned int index, bool state)
{
    if (index < NUM_CHANNELS) {
        AmpIO_UInt32 enable_mask = bswap_32(0x00000100 << index);
        write_buffer[WB_CTRL_OFFSET] |=  enable_mask;
        AmpIO_UInt32 state_mask  = bswap_32(0x00000001 << index);
        if (state)
            write_buffer[WB_CTRL_OFFSET] |=  state_mask;
        else
            write_buffer[WB_CTRL_OFFSET] &= ~state_mask;
        return true;
    }
    return false;
}

void AmpIO::SetSafetyRelay(bool state)
{
    AmpIO_UInt32 enable_mask = bswap_32(0x00020000);
    write_buffer[WB_CTRL_OFFSET] |=  enable_mask;
    AmpIO_UInt32 state_mask  = bswap_32(0x00010000);
    if (state)
        write_buffer[WB_CTRL_OFFSET] |=  state_mask;
    else
        write_buffer[WB_CTRL_OFFSET] &= ~state_mask;
}

bool AmpIO::SetMotorCurrent(unsigned int index, AmpIO_UInt32 sdata)
{
    quadlet_t data = VALID_BIT | DAC_WR_A | (sdata & DAC_MASK);

    if (index < NUM_CHANNELS) {
        write_buffer[index+WB_CURR_OFFSET] = bswap_32(data);
        return true;
    }
    else
        return false;
}

/*******************************************************************************
 * Read commands
 */

AmpIO_UInt32 AmpIO::ReadStatus(void) const
{
    AmpIO_UInt32 read_data = 0;
    if (port) port->ReadQuadlet(BoardId, 0, read_data);
    return bswap_32(read_data);
}

bool AmpIO::ReadPowerStatus(void) const
{
    return (ReadStatus()&0x00080000);

}

bool AmpIO::ReadSafetyRelayStatus(void) const
{
    return (ReadStatus()&0x00020000);
}

/*******************************************************************************
 * Write commands
 */

bool AmpIO::WritePowerEnable(bool state)
{
    AmpIO_UInt32 write_data = state ? PWR_ENABLE : PWR_DISABLE;
    return (port ? port->WriteQuadlet(BoardId, 0, bswap_32(write_data)) : false);
}

bool AmpIO::WriteAmpEnable(AmpIO_UInt8 mask, AmpIO_UInt8 state)
{
    quadlet_t write_data = (mask << 8) | state;
    return (port ? port->WriteQuadlet(BoardId, 0, bswap_32(write_data)) : false);
}

bool AmpIO::WriteSafetyRelay(bool state)
{
    AmpIO_UInt32 write_data = state ? RELAY_ON : RELAY_OFF;
    return (port ? port->WriteQuadlet(BoardId, 0, bswap_32(write_data)) : false);
}

bool AmpIO::WriteEncoderPreload(unsigned int index, AmpIO_UInt32 sdata)
{
    unsigned int channel = (index+1) << 4;

    if (port && (index < NUM_CHANNELS))
        return port->WriteQuadlet(BoardId, channel | ENC_LOAD_OFFSET, bswap_32(sdata));
    else
        return false;
}

bool AmpIO::WriteDigitalOutput(AmpIO_UInt8 mask, AmpIO_UInt8 bits)
{
    quadlet_t write_data = (mask << 8) | bits;
    return port->WriteQuadlet(BoardId, 6, bswap_32(write_data));
}

bool AmpIO::WriteWatchdogPeriod(AmpIO_UInt32 counts)
{
    // period = counts(16 bits) * 5.208333 us (default = 0 = no timeout)
    return port->WriteQuadlet(BoardId, 3, bswap_32(counts));
}

/*******************************************************************************
 * PROM commands
 */

AmpIO_UInt32 AmpIO::PromGetId(void)
{
    AmpIO_UInt32 id = 0;
    quadlet_t data = 0x9f000000;
    if (port->WriteQuadlet(BoardId, 8, bswap_32(data))) {
        // Should be ready by now...
        id = PromGetResult();
    }
    return id;
}

AmpIO_UInt32 AmpIO::PromGetStatus(void)
{
    AmpIO_UInt32 status = 0x80000000;
    quadlet_t data = 0x05000000;
    if (port->WriteQuadlet(BoardId, 8, bswap_32(data))) {
        // Should be ready by now...
        status = PromGetResult();
    }
    return status;
}

AmpIO_UInt32 AmpIO::PromGetResult(void)
{
    AmpIO_UInt32 result = 0xffffffff;
    quadlet_t data;
    if (port->ReadQuadlet(BoardId, 9, data))
        result = static_cast<AmpIO_UInt32>(bswap_32(data));
    return result;
}

bool AmpIO::PromReadData(AmpIO_UInt32 addr, AmpIO_UInt8 *data,
                         unsigned int nbytes)
{
    AmpIO_UInt32 addr24 = addr&0x00ffffff;
    if (addr24+nbytes > 0x00ffffff)
        return false;
    quadlet_t write_data = 0x03000000|addr24;
    AmpIO_UInt32 page = 0;
    while (page < nbytes) {
        unsigned int bytesToRead = std::min(nbytes - page, 256u);
        if (!port->WriteQuadlet(BoardId, 8, bswap_32(write_data)))
            return false;
        // Read FPGA status register; if 4 LSB are 0, command has finished.
        // The IEEE-1394 clock is 24.576 MHz, so it should take
        // about 256*8*(1/24.576) = 83.3 microseconds to read 256 bytes.
        // Experimentally, 1 iteration of the loop below is sufficient
        // most of the time, with 2 iterations required occasionally.
        quadlet_t read_data = 0x000f;
        int i;
        const int MAX_LOOP_CNT = 8;
        for (i = 0; (i < MAX_LOOP_CNT) && read_data; i++) {
            usleep(10);
            if (!port->ReadQuadlet(BoardId, 8, read_data)) return false;
            read_data = bswap_32(read_data)&0x000f;
        }
        if (i == MAX_LOOP_CNT) {
            std::cout << "PromReadData: command failed to finish, status = "
                      << std::hex << read_data << std::dec << std::endl;
            return false;
        }
        // Now, read result. This should be the number of quadlets written.
        AmpIO_UInt32 nRead = 4*PromGetResult();
        if (nRead != 256) { // should never happen
            std::cout << "PromReadData: incorrect number of bytes = "
                      << nRead << std::endl;
            return false;
        }
        if (!port->ReadBlock(BoardId, 0xc0, (quadlet_t *)(data+page), bytesToRead))
            return false;
        write_data += bytesToRead;
        page += bytesToRead;
    }
    return true;
}

bool AmpIO::PromWriteEnable(void)
{
    quadlet_t write_data = 0x06000000;
    return port->WriteQuadlet(BoardId, 8, bswap_32(write_data));
}

bool AmpIO::PromWriteDisable(void)
{
    quadlet_t write_data = 0x04000000;
    return port->WriteQuadlet(BoardId, 8, bswap_32(write_data));
}

bool AmpIO::PromSectorErase(AmpIO_UInt32 addr, const ProgressCallback cb)
{
    PromWriteEnable();
    quadlet_t write_data = 0xd8000000 | (addr&0x00ffffff);
    if (!port->WriteQuadlet(BoardId, 8, bswap_32(write_data)))
        return false;
    // Wait for erase to finish
    while (PromGetStatus())
        PROGRESS_CALLBACK(cb, false);
    return true;
}

int AmpIO::PromProgramPage(AmpIO_UInt32 addr, const AmpIO_UInt8 *bytes,
                           unsigned int nbytes, const ProgressCallback cb)
{
    const unsigned int MAX_PAGE = 256;
    if (nbytes > MAX_PAGE) {
        std::ostringstream msg;
        msg << "PromProgramPage: error, nbytes = " << nbytes
            << " (max = " << MAX_PAGE << ")";
        ERROR_CALLBACK(cb, msg);
        return -1;
    }
    PromWriteEnable();
    // Block write of the data
    AmpIO_UInt8 page_data[MAX_PAGE+sizeof(quadlet_t)];
    quadlet_t *data_ptr = reinterpret_cast<quadlet_t *>(page_data);
    // First quadlet is the "page program" instruction (0x02)
    data_ptr[0] = bswap_32(0x02000000 | (addr & 0x00ffffff));
    // Remaining quadlets are the data to be programmed. These do not
    // need to be byte-swapped.
    memcpy(page_data+sizeof(quadlet_t), bytes, nbytes);
    if (!port->WriteBlock(BoardId, 0xc0, data_ptr, nbytes+sizeof(quadlet_t)))
        return -1;
    // Read FPGA status register; if 4 LSB are 0, command has finished
    quadlet_t read_data;
    if (!port->ReadQuadlet(BoardId, 8, read_data)) return -1;
    read_data = bswap_32(read_data);
    while (read_data&0x000f) {
        PROGRESS_CALLBACK(cb, -1);
        if (!port->ReadQuadlet(BoardId, 8, read_data)) return false;
        read_data = bswap_32(read_data);
    }
    if (read_data & 0xff000000) { // shouldn't happen
        std::ostringstream msg;
        msg << "PromProgramPage: FPGA error = " << read_data;
        ERROR_CALLBACK(cb, msg);
    }
    // Now, read result. This should be the number of quadlets written.
    AmpIO_UInt32 nWritten = 4*(PromGetResult()-1);
    if (nWritten != nbytes) {
        std::ostringstream msg;
        msg << "PromProgramPage: wrote " << nWritten << " of "
            << nbytes << "bytes";
        ERROR_CALLBACK(cb, msg);
    }
    // Wait for "Write in Progress" bit to be cleared
    while (PromGetStatus()&MASK_WIP)
        PROGRESS_CALLBACK(cb, 0);
    return nWritten;
}
