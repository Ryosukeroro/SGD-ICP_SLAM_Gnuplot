#ifndef CONSTANTS_H
#define CONSTANTS_H

#include <cmath> // M_PIを使うため

// 名前空間で囲むことで、他のライブラリとの名前衝突を防ぐ
namespace SlamParams {

    // 定数はすべて大文字 + アンダースコア区切りが一般的
    constexpr int MAX_ITERATION = 40;        // 最大反復回数
    constexpr double LEARNING_RATE = 0.4;    // 学習率
    constexpr double CONVERGENCE_EPS = 1e-6; // 収束判定閾値 (単にEPSだと衝突しやすい)

    // 微小変位 (数値微分用)
    constexpr double DELTA = 1.0e-7;

    // 対応点とみなす最大距離の閾値 (の2乗)
    // 例: 0.4m以内を有効とする -> 0.16
    constexpr double MAX_CORRESPONDENCE_DIST_SQ = 0.16;
