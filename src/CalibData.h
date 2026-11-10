#pragma once
// ==============================================================
// 校正データ（PCアプリの「校正を書き出し (CalibData.h)」がこのファイルを丸ごと書き換える）
//  - 機体ごとの値。ファームとPCアプリの両方がこの形式を読む（PCアプリは「CalibData.h を開く」で読み込む）。
//  - 手で編集しない。値の範囲: OFFSET は 3500〜11500、FOOTDEF は -100〜100 [mm]。
//  - FORMAT を変えたら、PCアプリ（PC_App/main.py の CALIB_FORMAT）も合わせること。
// ==============================================================
#include <stdint.h>

namespace calib_data {
inline constexpr int FORMAT = 1;
// サーボごとのオフセット（生ICSで論理7500=中立になる値。範囲 3500〜11500）
inline constexpr int16_t OFFSET[19] = {
    7500,                           // [0] 腰
    7678, 7500, 7500, 7500,         // [1..4] 左腕
    7204, 7500, 7500, 7500,         // [5..8] 右腕
    7500, 7676, 7710, 7955, 6300,   // [9..13] 左脚
    7500, 7575, 7299, 7299, 7720    // [14..18] 右脚
};
// 左右の脚の高さ補正 [mm]
inline constexpr float FOOTDEF_L = 0.0f;
inline constexpr float FOOTDEF_R = 10.0f;
} // namespace calib_data
