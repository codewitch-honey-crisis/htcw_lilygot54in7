#pragma once
#include <Arduino.h>
#include <gfx_bitmap.hpp>
namespace arduino {
    struct lilygot54in7 {
        using type = lilygot54in7;
        using pixel_type = gfx::gsc_pixel<4>;
        using caps = gfx::gfx_caps<false,false,false,false,true,true,false>;
        gfx::gfx_result initialize();
        bool initialized() const;
        void sleep(bool all=false);
        void wash();
        float voltage() const;
        bool auto_clear() const;
        void auto_clear(bool value);
        uint8_t rotation() const;
        void rotation(uint8_t value);
        void invalidate();
        gfx::size16 dimensions() const;
        gfx::rect16 bounds() const;
        gfx::gfx_result clear(const gfx::rect16& bounds);
        gfx::gfx_result fill(const gfx::rect16& bounds,pixel_type color);
        gfx::gfx_result point(gfx::point16 location,pixel_type color);
        gfx::gfx_result point(gfx::point16 location,pixel_type* out_color) const;
        gfx::gfx_result suspend();
        gfx::gfx_result resume(bool force=false);

    };
}