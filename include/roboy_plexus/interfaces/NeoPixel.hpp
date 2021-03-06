/*
    BSD 3-Clause License

    Copyright (c) 2020, Roboy
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

    author: Simon Trendel ( st@gi.ai ), 2020
    description: Class for interfacing neopixel module, implements some funky animations
*/

#pragma once

#include <ros/ros.h>

using namespace std;

#define IORD(base, reg) (*(((volatile int32_t*)base)+reg))
#define IOWR(base, reg, data) (*(((volatile int32_t*)base)+reg)=data)

struct NeoPixelColorRGBW{
    static const int black = 0;
    static const int white = 0x01000000;
    static const int blue  = 0x00010000;
    static const int red   = 0x00000100;
    static const int green = 0x00000001;
    static const int yellow = 0x00010001;
    static const int all = 0x01010101;
};

struct NeoPixelColorRGB{
    static const int black = 0;
    static const int blue  = 0xE00000;
    static const int red   = 0x00E000;
    static const int green = 0x0000E0;
    static const int yellow = 0x00E000E0;
    static const int white = 0x00E0E0E0;
};

class NeoPixel{
public:
    NeoPixel(int32_t *base, int number_of_neopixel):base(base), number_of_neopixel(number_of_neopixel){
        for(int j=1;j<=number_of_neopixel;j++){
            IOWR(base,j,NeoPixelColorRGB::black);
        }
        //latch
        IOWR(base,0,true);
    }
    ~NeoPixel(){

    }
    void runPattern(map<int,vector<int>> pattern, ros::Rate rate, bool loop = false, int timeOut = -1){
        for(int j=1;j<=number_of_neopixel;j++){
            IOWR(base,j,NeoPixelColorRGB::black);
        }
        //latch
        IOWR(base,0,true);
        int i=0;
        int pattern_length = 0;
        for(auto &p:pattern){
            pattern_length = p.second.size();
        }
        do{
            for(auto &p:pattern){
                IOWR(base,p.first,p.second[i]);
            }
            //latch
            IOWR(base,0,true);
            rate.sleep();
            i++;
        }while(i<pattern_length && !abort);
    }
    map<int,vector<int>> getPattern(string name, int color){
        map<int,vector<int>> pattern;
        if(name=="run"){
            pattern[1] = {color,0,0,0,0,0,0,0,0,0};
            pattern[2] = {0,color,0,0,0,0,0,0,0,0};
            pattern[3] = {0,0,color,0,0,0,0,0,0,0};
            pattern[4] = {0,0,0,color,0,0,0,0,0,0};
            pattern[5] = {0,0,0,0,color,0,0,0,0,0};
            pattern[6] = {0,0,0,0,0,color,0,0,0,0};
            pattern[7] = {0,0,0,0,0,0,color,0,0,0};
            pattern[8] = {0,0,0,0,0,0,0,color,0,0};
            pattern[9] = {0,0,0,0,0,0,0,0,color,0};
            pattern[10] = {0,0,0,0,0,0,0,0,0,color};
        }else if(name=="nightrider"){
            pattern[1] =  {color,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,color};
            pattern[2] =  {0,color,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,color,0};
            pattern[3] =  {0,0,color,0,0,0,0,0,0,0,0,0,0,0,0,0,0,color,0,0};
            pattern[4] =  {0,0,0,color,0,0,0,0,0,0,0,0,0,0,0,0,color,0,0,0};
            pattern[5] =  {0,0,0,0,color,0,0,0,0,0,0,0,0,0,0,color,0,0,0,0};
            pattern[6] =  {0,0,0,0,0,color,0,0,0,0,0,0,0,0,color,0,0,0,0,0};
            pattern[7] =  {0,0,0,0,0,0,color,0,0,0,0,0,0,color,0,0,0,0,0,0};
            pattern[8] =  {0,0,0,0,0,0,0,color,0,0,0,0,color,0,0,0,0,0,0,0};
            pattern[9] =  {0,0,0,0,0,0,0,0,color,0,0,color,0,0,0,0,0,0,0,0};
            pattern[10] = {0,0,0,0,0,0,0,0,0,color,color,0,0,0,0,0,0,0,0,0};
        }else if(name=="fadeout"){
            switch(color){
                case NeoPixelColorRGB::blue:{
                    int color = 0xFF0000;
                    vector<int> p;
                    for(int i=0;i<255;i++){
                        p.push_back(color);
                        color-=0x10000;
                    }
                    for(int i=0;i<10;i++){
                        pattern[i] =p;
                    }
                    break;
                }
                case NeoPixelColorRGB::red:{
                    int color = 0xFF00;
                    vector<int> p;
                    for(int i=0;i<255;i++){
                        p.push_back(color);
                        color-=0x100;
                    }
                    for(int i=0;i<10;i++){
                        pattern[i] =p;
                    }
                    break;
                }
                case NeoPixelColorRGB::green:{
                    int color = 0xFF;
                    vector<int> p;
                    for(int i=0;i<255;i++){
                        p.push_back(color);
                        color-=0x1;
                    }
                    for(int i=0;i<10;i++){
                        pattern[i] =p;
                    }
                    break;
                }
            }
        }
        return pattern;
    }
    void setColor(int pixel, int color){
        IOWR(base,pixel,color);
        //latch
        IOWR(base,0,true);
    }
    void setColorAll(int color){
        for(int j=1;j<=number_of_neopixel;j++){
            IOWR(base,j,color);
        }
        //latch
        IOWR(base,0,true);
    }
    int number_of_neopixel;
    bool abort = false;
private:
    int32_t *base;
    map<string,map<int,vector<int>>> pattern;
};

typedef boost::shared_ptr<NeoPixel> NeoPixelPtr;
