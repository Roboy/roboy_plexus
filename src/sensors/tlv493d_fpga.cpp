#include "roboy_plexus/tlv493d_fpga.hpp"

TLV493D_FPGA::TLV493D_FPGA(int32_t *tlv_base):tlv_base(tlv_base){

}

TLV493D_FPGA::~TLV493D_FPGA() {

}

float TLV493D_FPGA::convertToMilliTesla(int32_t data) {
    int val = 0;
    for(int i=11;i>=0;i--){
        if(i==11){
            if((data>>i)&0x1)
                val = -2048;
        }else{
            if((data>>i)&0x1)
                val += (1<<i);
        }
    }
    return val*0.098;
}

bool TLV493D_FPGA::read(float &fx, float &fy, float &fz){
    int32_t x,y,z;
    x = IORD(tlv_base, (0<<8));
    y = IORD(tlv_base, (1<<8));
    z = IORD(tlv_base, (2<<8));

    fx = convertToMilliTesla(x);
    fy = convertToMilliTesla(y);
    fz = convertToMilliTesla(z);
    return true;
}