/*
    BSD 3-Clause License

    Copyright (c) 2017, Roboy
            All rights reserved.

    Redistribution and use in source and binary forms, with or without
            modification, are permitted provided that the following conditions are met:

    * Redistributions of source code must retain the above copyright notice, this
    list of conditions and the following disclaimer.

    * Redistributions in binary form must reproduce the above copyright notice,
    this list of conditions and the following disclaimer in the documentation
    and/or other materials provided with the distribution.

    * Neither the name of the copyright holder nor the names of its
    contributors may be used to endorse or promote products derived from
    this software without specific prior written permission.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
    AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
            IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
    DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
            FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
            DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
            SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
            CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
    OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
    OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

    author: Simon Trendel ( simon.trendel@tum.de ), 2018
    description:
*/
#pragma once

#define MOTORS_PER_MYOCONTROL 7

#define IORD(base,reg) (*(((volatile int32_t*)base)+reg))
#define IOWR(base,reg,data) (*(((volatile int32_t*)base)+reg)=data)

// the upper 8 bit define which value, the lower 8 bit define which motor
#define MYO_READ_Kp(base,motor) IORD(base, (uint32_t)(0x00<<8|motor&0xff) )
#define MYO_READ_Ki(base,motor) IORD(base, (uint32_t)(0x01<<8|motor&0xff) )
#define MYO_READ_Kd(base,motor) IORD(base, (uint32_t)(0x02<<8|motor&0xff) )
#define MYO_READ_sp(base,motor) IORD(base, (uint32_t)(0x03<<8|motor&0xff) )
#define MYO_READ_forwardGain(base,motor) IORD(base, (uint32_t)(0x04<<8|motor&0xff) )
#define MYO_READ_outputPosMax(base,motor) IORD(base, (uint32_t)(0x05<<8|motor&0xff) )
#define MYO_READ_outputNegMax(base,motor) IORD(base, (uint32_t)(0x06<<8|motor&0xff) )
#define MYO_READ_IntegralPosMax(base,motor) IORD(base, (uint32_t)(0x07<<8|motor&0xff) )
#define MYO_READ_IntegralNegMax(base,motor) IORD(base, (uint32_t)(0x08<<8|motor&0xff) )
#define MYO_READ_deadBand(base,motor) IORD(base, (uint32_t)(0x09<<8|motor&0xff) )
#define MYO_READ_control(base,motor) IORD(base, (uint32_t)(0x0A<<8|motor&0xff) )
#define MYO_READ_position(base,motor) IORD(base, (uint32_t)(0x0B<<8|motor&0xff) )
#define MYO_READ_velocity(base,motor) IORD(base, (uint32_t)(0x0C<<8|motor&0xff) )
#define MYO_READ_current(base,motor) IORD(base, (uint32_t)(0x0D<<8|motor&0xff) )
#define MYO_READ_displacement(base,motor) IORD(base, (uint32_t)(0x0E<<8|motor&0xff) )
#define MYO_READ_pwmRef(base,motor) IORD(base, (uint32_t)(0x0F<<8|motor&0xff) )
#define MYO_READ_update_frequency(base,motor) IORD(base, (uint32_t)(0x10<<8|motor&0xff) )

#define MYO_WRITE_Kp(base,motor,data) IOWR(base, (uint32_t)(0x00<<8|motor&0xff), data )
#define MYO_WRITE_Ki(base,motor,data) IOWR(base, (uint32_t)(0x01<<8|motor&0xff), data )
#define MYO_WRITE_Kd(base,motor,data) IOWR(base, (uint32_t)(0x02<<8|motor&0xff), data )
#define MYO_WRITE_sp(base,motor,data) IOWR(base, (uint32_t)(0x03<<8|motor&0xff), data )
#define MYO_WRITE_forwardGain(base,motor,data) IOWR(base, (uint32_t)(0x04<<8|motor&0xff), data )
#define MYO_WRITE_outputPosMax(base,motor,data) IOWR(base, (uint32_t)(0x05<<8|motor&0xff), data )
#define MYO_WRITE_outputNegMax(base,motor,data) IOWR(base, (uint32_t)(0x06<<8|motor&0xff), data )
#define MYO_WRITE_IntegralPosMax(base,motor,data) IOWR(base, (uint32_t)(0x07<<8|motor&0xff), data )
#define MYO_WRITE_IntegralNegMax(base,motor,data) IOWR(base, (uint32_t)(0x08<<8|motor&0xff), data )
#define MYO_WRITE_deadBand(base,motor,data) IOWR(base, (uint32_t)(0x09<<8|motor&0xff), data )
#define MYO_WRITE_control(base,motor,data) IOWR(base, (uint32_t)(0x0A<<8|motor&0xff), data )
#define MYO_WRITE_reset_myo_control(base,data) IOWR(base, (uint32_t)(0x0B<<8|0), data )
#define MYO_WRITE_spi_activated(base,data) IOWR(base, (uint32_t)(0x0C<<8|0), data )
#define MYO_WRITE_reset_controller(base,motor) IOWR(base, (uint32_t)(0x0D<<8|motor&0xff), 1 )
#define MYO_WRITE_update_frequency(base,motor) IOWR(base, (uint32_t)(0x0E<<8|motor&0xff), 1 )