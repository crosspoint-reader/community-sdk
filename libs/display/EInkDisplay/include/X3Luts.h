#pragma once

#include <cstdint>

// Extern declarations for the X3 LUT bank arrays. Definitions live in
// X3Luts.cpp; the sole in-tree consumer is X3Panel.cpp.
//
// Four banks for BW page-turn paths (full / half / fast / normal) and
// two banks for grayscale (gc for OEM 4-level grayscale, grayscale for
// the differential AA path). Each bank has the five LUT registers
// VCOM / WW / BW / WB / BB.

extern const uint8_t lut_x3_vcom_normal[];
extern const uint8_t lut_x3_ww_normal[];
extern const uint8_t lut_x3_bw_normal[];
extern const uint8_t lut_x3_wb_normal[];
extern const uint8_t lut_x3_bb_normal[];
extern const uint8_t lut_x3_vcom_half[];
extern const uint8_t lut_x3_ww_half[];
extern const uint8_t lut_x3_bw_half[];
extern const uint8_t lut_x3_wb_half[];
extern const uint8_t lut_x3_bb_half[];
extern const uint8_t lut_x3_vcom_fast[];
extern const uint8_t lut_x3_ww_fast[];
extern const uint8_t lut_x3_bw_fast[];
extern const uint8_t lut_x3_wb_fast[];
extern const uint8_t lut_x3_bb_fast[];
extern const uint8_t lut_x3_vcom_full[];
extern const uint8_t lut_x3_ww_full[];
extern const uint8_t lut_x3_bw_full[];
extern const uint8_t lut_x3_wb_full[];
extern const uint8_t lut_x3_bb_full[];

// grayscale + gc banks
extern const uint8_t lut_x3_vcom_grayscale[];
extern const uint8_t lut_x3_ww_grayscale[];
extern const uint8_t lut_x3_bw_grayscale[];
extern const uint8_t lut_x3_wb_grayscale[];
extern const uint8_t lut_x3_bb_grayscale[];
extern const uint8_t lut_x3_vcom_gc[];
extern const uint8_t lut_x3_ww_gc[];
extern const uint8_t lut_x3_bw_gc[];
extern const uint8_t lut_x3_wb_gc[];
extern const uint8_t lut_x3_bb_gc[];
