#pragma once

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
    static const int blue  = 0x010000;
    static const int red   = 0x00000100;
    static const int green = 0x00000001;
    static const int yellow = 0x00010001;
    static const int white = 0x00010101;
};

class NeoPixel{
public:
    NeoPixel(int32_t *base, int number_of_neopixel):base(base), number_of_neopixel(number_of_neopixel){

    }
    ~NeoPixel(){

    }
    void runPattern(map<int,vector<int>> pattern, ros::Rate rate, bool loop = false, int timeOut = -1){
        for(int j=0;j<number_of_neopixel;j++){
            IOWR(base,j,NeoPixelColorRGB::black);
        }
        int i=0;
        int pattern_length = 0;
        for(auto &p:pattern){
            pattern_length = p.second.size();
        }
        do{
            for(auto &p:pattern){
                IOWR(base,p.first,p.second[i]);
            }
            rate.sleep();
            i++;
        }while(i<pattern_length);
    }
    void setColor(int pixel, int color){
        IOWR(base,pixel,color);
    }
    void setColorAll(int color){
        for(int i=0;i<number_of_neopixel;i++){
            IOWR(base,i,color);
        }
    }
    int number_of_neopixel;
private:
    int32_t *base;
};