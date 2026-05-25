#pragma once

// Extern declarations for the X4 LUT bank data. Definitions live in
// X4Luts.cpp; consumers are X4Panel.cpp and any external code that
// reaches in by name (HAL / firmware grayscale chooser).

extern const unsigned char lut_grayscale[];
extern const unsigned char lut_grayscale_revert[];
extern const unsigned char lut_factory_fast[];
extern const unsigned char lut_factory_quality[];
