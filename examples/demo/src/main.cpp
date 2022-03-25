#include <Arduino.h>
#include <gfx_cpp14.hpp>
#include <lilygot54in7.hpp>
#include "image3.h"
#include "Maziro.h"

using namespace arduino;
using namespace gfx;

lilygot54in7 epd;
using epd_color = color<decltype(epd)::pixel_type>;
void lines_demo() {
    

    draw::suspend(epd);

    draw::filled_rectangle(epd,(srect16)epd.bounds(),epd_color::white);
    const open_font& f = Maziro_ttf;
    const char* text = "GFX";
    const float scale = f.scale(min(epd.dimensions().width,epd.dimensions().height)/2);
    srect16 text_rect = srect16(spoint16(0,0),
                            f.measure_text((ssize16)epd.dimensions(),
                            {0,0},text,scale));
    draw::text(epd,text_rect.center((srect16)epd.bounds()),{0,0},text,f,scale,epd_color::black,epd_color::white,true,true);

    for(int i = 1;i<100;i+=4) {
        // calculate our extents
        srect16 r(i*(epd.dimensions().width/100.0),
                i*(epd.dimensions().height/100.0),
                epd.dimensions().width-i*(epd.dimensions().width/100.0)-1,
                epd.dimensions().height-i*(epd.dimensions().height/100.0)-1);

        draw::line(epd,srect16(0,r.y1,r.x1,epd.dimensions().height-1),epd_color::red);
        draw::line(epd,srect16(r.x2,0,epd.dimensions().width-1,r.y2),epd_color::yellow);
        draw::line(epd,srect16(0,r.y2,r.x1,0),epd_color::orange);
        draw::line(epd,srect16(epd.dimensions().width-1,r.y1,r.x2,epd.dimensions().height-1),epd_color::green);        
    }
    
    draw::resume(epd);
    
    delay(500);
}
void alpha_demo() {
    draw::suspend(epd);
    draw::filled_rectangle(epd, (srect16)epd.bounds(), epd_color::black);

    for (int y = 0; y < epd.dimensions().height; y += 16) {
        for (int x = 0; x < epd.dimensions().width; x += 16) {
            if (0 != ((x + y) % 32)) {
                draw::filled_rectangle(epd,
                                       srect16(spoint16(x, y), ssize16(16, 16)),
                                       epd_color::white);
            }
        }
    }
    randomSeed(millis());

    rgba_pixel<32> px;
    spoint16 tpa[3];
    const uint16_t sw =
        min(epd.dimensions().width, epd.dimensions().height) / 4;
    for (int i = 0; i < 30; ++i) {
        px.channel<channel_name::R>((rand() % 256));
        px.channel<channel_name::G>((rand() % 256));
        px.channel<channel_name::B>((rand() % 256));
        px.channel<channel_name::A>(50 + rand() % 156);
        srect16 sr(0, 0, rand() % sw + sw, rand() % sw + sw);
        sr.offset_inplace(rand() % (epd.dimensions().width - sr.width()),
                          rand() % (epd.dimensions().height - sr.height()));
        switch (rand() % 4) {
            case 0:
                draw::filled_rectangle(epd, sr, px);
                break;
            case 1:
                draw::filled_rounded_rectangle(epd, sr, .1, px);
                break;
            case 2:
                draw::filled_ellipse(epd, sr, px);
                break;
            case 3:
                tpa[0] = {int16_t(((sr.x2 - sr.x1) / 2) + sr.x1), sr.y1};
                tpa[1] = {sr.x2, sr.y2};
                tpa[2] = {sr.x1, sr.y2};
                spath16 path(3, tpa);
                draw::filled_polygon(epd, path, px);
                break;
        }
    }
    draw::resume(epd);

    delay(2000);
}
void setup() {
    Serial.begin(115200);

    epd.clear(epd.bounds());
    
}

void loop() {
    lines_demo();
    alpha_demo();
    epd.sleep(true);
    delay(2000);
    draw::suspend(epd);
    srect16 sr(0,0,335,255);
    sr.center_inplace((srect16)epd.bounds());
    image3_jpg_stream.seek(0);
    draw::image(epd,sr,&image3_jpg_stream);
    draw::resume(epd);
    delay(2000);
}
