/**
 * @file stampfly_math.hpp
 * @brief Math Library for ESKF (Vector3, Quaternion, Matrix)
 *
 * 組み込みシステム向け数学ライブラリ
 * 外部依存なし、静的メモリ割り当て
 */

#pragma once

#include <cstdint>
#include <cmath>
#include <algorithm>

namespace stampfly {
namespace math {

/**
 * @brief 3次元ベクトルクラス
 */
class Vector3 {
public:
    float x = 0.0f;
    float y = 0.0f;
    float z = 0.0f;

    Vector3() = default;
    Vector3(float x_, float y_, float z_) : x(x_), y(y_), z(z_) {}

    // ノルム（大きさ）
    float norm() const {
        return std::sqrt(x*x + y*y + z*z);
    }

    float squaredNorm() const {
        return x*x + y*y + z*z;
    }

    // 正規化
    Vector3 normalized() const {
        float n = norm();
        if (n < 1e-10f) return Vector3(0, 0, 0);
        return Vector3(x/n, y/n, z/n);
    }

    void normalize() {
        float n = norm();
        if (n < 1e-10f) return;
        x /= n; y /= n; z /= n;
    }

    // 内積
    float dot(const Vector3& v) const {
        return x*v.x + y*v.y + z*v.z;
    }

    // 外積
    Vector3 cross(const Vector3& v) const {
        return Vector3(
            y*v.z - z*v.y,
            z*v.x - x*v.z,
            x*v.y - y*v.x
        );
    }

    // 演算子
    Vector3 operator+(const Vector3& v) const { return Vector3(x+v.x, y+v.y, z+v.z); }
    Vector3 operator-(const Vector3& v) const { return Vector3(x-v.x, y-v.y, z-v.z); }
    Vector3 operator*(float s) const { return Vector3(x*s, y*s, z*s); }
    Vector3 operator/(float s) const { return Vector3(x/s, y/s, z/s); }
    Vector3 operator-() const { return Vector3(-x, -y, -z); }

    Vector3& operator+=(const Vector3& v) { x+=v.x; y+=v.y; z+=v.z; return *this; }
    Vector3& operator-=(const Vector3& v) { x-=v.x; y-=v.y; z-=v.z; return *this; }
    Vector3& operator*=(float s) { x*=s; y*=s; z*=s; return *this; }
    Vector3& operator/=(float s) { x/=s; y/=s; z/=s; return *this; }

    // インデックスアクセス
    float& operator[](int i) {
        if (i == 0) return x;
        if (i == 1) return y;
        return z;
    }
    float operator[](int i) const {
        if (i == 0) return x;
        if (i == 1) return y;
        return z;
    }

    // 静的ファクトリ
    static Vector3 zero() { return Vector3(0, 0, 0); }
    static Vector3 unitX() { return Vector3(1, 0, 0); }
    static Vector3 unitY() { return Vector3(0, 1, 0); }
    static Vector3 unitZ() { return Vector3(0, 0, 1); }
};

// スカラー×ベクトル
inline Vector3 operator*(float s, const Vector3& v) { return v * s; }

/**
 * @brief クォータニオンクラス（回転表現）
 *
 * 格納順序: [w, x, y, z] (スカラー優先)
 */
class Quaternion {
public:
    float w = 1.0f;
    float x = 0.0f;
    float y = 0.0f;
    float z = 0.0f;

    Quaternion() = default;
    Quaternion(float w_, float x_, float y_, float z_) : w(w_), x(x_), y(y_), z(z_) {}

    // ノルム
    float norm() const {
        return std::sqrt(w*w + x*x + y*y + z*z);
    }

    // 正規化
    void normalize() {
        float n = norm();
        if (n < 1e-10f) {
            w = 1.0f; x = y = z = 0.0f;
            return;
        }
        w /= n; x /= n; y /= n; z /= n;
    }

    Quaternion normalized() const {
        Quaternion q = *this;
        q.normalize();
        return q;
    }

    // 共役
    Quaternion conjugate() const {
        return Quaternion(w, -x, -y, -z);
    }

    // 逆クォータニオン
    Quaternion inverse() const {
        float n2 = w*w + x*x + y*y + z*z;
        if (n2 < 1e-10f) return Quaternion();
        return Quaternion(w/n2, -x/n2, -y/n2, -z/n2);
    }

    // ベクトル回転 v' = q * v * q^{-1}
    Vector3 rotate(const Vector3& v) const {
        // 効率的な計算: v' = v + 2*w*(q_v × v) + 2*(q_v × (q_v × v))
        Vector3 qv(x, y, z);
        Vector3 t = 2.0f * qv.cross(v);
        return v + w * t + qv.cross(t);
    }

    // クォータニオン積 (Hamilton積)
    Quaternion operator*(const Quaternion& q) const {
        return Quaternion(
            w*q.w - x*q.x - y*q.y - z*q.z,
            w*q.x + x*q.w + y*q.z - z*q.y,
            w*q.y - x*q.z + y*q.w + z*q.x,
            w*q.z + x*q.y - y*q.x + z*q.w
        );
    }

    Quaternion& operator*=(const Quaternion& q) {
        *this = *this * q;
        return *this;
    }

    // オイラー角への変換 (ZYX順序: Yaw-Pitch-Roll)
    void toEuler(float& roll, float& pitch, float& yaw) const {
        // Roll (x-axis rotation)
        float sinr_cosp = 2.0f * (w * x + y * z);
        float cosr_cosp = 1.0f - 2.0f * (x * x + y * y);
        roll = std::atan2(sinr_cosp, cosr_cosp);

        // Pitch (y-axis rotation)
        float sinp = 2.0f * (w * y - z * x);
        if (std::abs(sinp) >= 1.0f) {
            pitch = std::copysign(M_PI / 2.0f, sinp);
        } else {
            pitch = std::asin(sinp);
        }

        // Yaw (z-axis rotation)
        float siny_cosp = 2.0f * (w * z + x * y);
        float cosy_cosp = 1.0f - 2.0f * (y * y + z * z);
        yaw = std::atan2(siny_cosp, cosy_cosp);
    }

    // オイラー角からの生成 (ZYX順序)
    static Quaternion fromEuler(float roll, float pitch, float yaw) {
        float cr = std::cos(roll * 0.5f);
        float sr = std::sin(roll * 0.5f);
        float cp = std::cos(pitch * 0.5f);
        float sp = std::sin(pitch * 0.5f);
        float cy = std::cos(yaw * 0.5f);
        float sy = std::sin(yaw * 0.5f);

        return Quaternion(
            cr * cp * cy + sr * sp * sy,
            sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy
        );
    }

    // 軸角度からの生成
    static Quaternion fromAxisAngle(const Vector3& axis, float angle) {
        Vector3 n = axis.normalized();
        float half = angle * 0.5f;
        float s = std::sin(half);
        return Quaternion(std::cos(half), n.x * s, n.y * s, n.z * s);
    }

    // 回転ベクトルからの生成（小角度近似ではない）
    static Quaternion fromRotationVector(const Vector3& rv) {
        float angle = rv.norm();
        if (angle < 1e-10f) {
            return Quaternion(1.0f, 0.0f, 0.0f, 0.0f);
        }
        return fromAxisAngle(rv / angle, angle);
    }

    // 球面線形補間 (SLERP)
    static Quaternion slerp(const Quaternion& q0, const Quaternion& q1, float t) {
        float dot = q0.w*q1.w + q0.x*q1.x + q0.y*q1.y + q0.z*q1.z;

        Quaternion q1_ = q1;
        if (dot < 0.0f) {
            q1_ = Quaternion(-q1.w, -q1.x, -q1.y, -q1.z);
            dot = -dot;
        }

        if (dot > 0.9995f) {
            // 線形補間にフォールバック
            return Quaternion(
                q0.w + t * (q1_.w - q0.w),
                q0.x + t * (q1_.x - q0.x),
                q0.y + t * (q1_.y - q0.y),
                q0.z + t * (q1_.z - q0.z)
            ).normalized();
        }

        float theta = std::acos(dot);
        float sin_theta = std::sin(theta);
        float s0 = std::sin((1.0f - t) * theta) / sin_theta;
        float s1 = std::sin(t * theta) / sin_theta;

        return Quaternion(
            s0 * q0.w + s1 * q1_.w,
            s0 * q0.x + s1 * q1_.x,
            s0 * q0.y + s1 * q1_.y,
            s0 * q0.z + s1 * q1_.z
        );
    }

    // 単位クォータニオン
    static Quaternion identity() { return Quaternion(1, 0, 0, 0); }
};

/**
 * @brief 固定サイズ行列クラス（テンプレート）
 *
 * 組み込みシステム向け、静的メモリ割り当て
 */
template<int Rows, int Cols>
class Matrix {
public:
    float data[Rows][Cols] = {};

    Matrix() = default;

    // 全要素を値で初期化
    explicit Matrix(float val) {
        for (int i = 0; i < Rows; i++) {
            for (int j = 0; j < Cols; j++) {
                data[i][j] = val;
            }
        }
    }

    // 要素アクセス
    float& operator()(int row, int col) { return data[row][col]; }
    float operator()(int row, int col) const { return data[row][col]; }

    // 行数・列数
    static constexpr int rows() { return Rows; }
    static constexpr int cols() { return Cols; }

    // 転置
    Matrix<Cols, Rows> transpose() const {
        Matrix<Cols, Rows> result;
        for (int i = 0; i < Rows; i++) {
            for (int j = 0; j < Cols; j++) {
                result(j, i) = data[i][j];
            }
        }
        return result;
    }

    // 行列加算
    Matrix operator+(const Matrix& m) const {
        Matrix result;
        for (int i = 0; i < Rows; i++) {
            for (int j = 0; j < Cols; j++) {
                result(i, j) = data[i][j] + m(i, j);
            }
        }
        return result;
    }

    // 行列減算
    Matrix operator-(const Matrix& m) const {
        Matrix result;
        for (int i = 0; i < Rows; i++) {
            for (int j = 0; j < Cols; j++) {
                result(i, j) = data[i][j] - m(i, j);
            }
        }
        return result;
    }

    // スカラー乗算
    Matrix operator*(float s) const {
        Matrix result;
        for (int i = 0; i < Rows; i++) {
            for (int j = 0; j < Cols; j++) {
                result(i, j) = data[i][j] * s;
            }
        }
        return result;
    }

    // スカラー除算
    Matrix operator/(float s) const {
        return *this * (1.0f / s);
    }

    // 行列乗算
    template<int K>
    Matrix<Rows, K> operator*(const Matrix<Cols, K>& m) const {
        Matrix<Rows, K> result;
        for (int i = 0; i < Rows; i++) {
            for (int j = 0; j < K; j++) {
                float sum = 0.0f;
                for (int k = 0; k < Cols; k++) {
                    sum += data[i][k] * m(k, j);
                }
                result(i, j) = sum;
            }
        }
        return result;
    }

    // 複合代入演算子
    Matrix& operator+=(const Matrix& m) {
        for (int i = 0; i < Rows; i++) {
            for (int j = 0; j < Cols; j++) {
                data[i][j] += m(i, j);
            }
        }
        return *this;
    }

    Matrix& operator-=(const Matrix& m) {
        for (int i = 0; i < Rows; i++) {
            for (int j = 0; j < Cols; j++) {
                data[i][j] -= m(i, j);
            }
        }
        return *this;
    }

    Matrix& operator*=(float s) {
        for (int i = 0; i < Rows; i++) {
            for (int j = 0; j < Cols; j++) {
                data[i][j] *= s;
            }
        }
        return *this;
    }

    // 単位行列
    static Matrix identity() {
        static_assert(Rows == Cols, "Identity matrix requires square matrix");
        Matrix result;
        for (int i = 0; i < Rows; i++) {
            result(i, i) = 1.0f;
        }
        return result;
    }

    // ゼロ行列
    static Matrix zeros() {
        return Matrix();
    }

    // ブロック抽出
    template<int R, int C>
    Matrix<R, C> block(int startRow, int startCol) const {
        Matrix<R, C> result;
        for (int i = 0; i < R; i++) {
            for (int j = 0; j < C; j++) {
                result(i, j) = data[startRow + i][startCol + j];
            }
        }
        return result;
    }

    // ブロック設定
    template<int R, int C>
    void setBlock(int startRow, int startCol, const Matrix<R, C>& block) {
        for (int i = 0; i < R; i++) {
            for (int j = 0; j < C; j++) {
                data[startRow + i][startCol + j] = block(i, j);
            }
        }
    }

    // 対角成分設定
    void setDiagonal(float val) {
        int n = (Rows < Cols) ? Rows : Cols;
        for (int i = 0; i < n; i++) {
            data[i][i] = val;
        }
    }

    // 対角行列作成
    static Matrix diagonal(float val) {
        Matrix result;
        result.setDiagonal(val);
        return result;
    }
};

// スカラー×行列
template<int R, int C>
Matrix<R, C> operator*(float s, const Matrix<R, C>& m) {
    return m * s;
}

/**
 * @brief 正方行列の逆行列計算（Gauss-Jordan法）
 *
 * 最大6x6までサポート（ESKFで使用する観測更新用）
 */
template<int N>
Matrix<N, N> inverse(const Matrix<N, N>& m) {
    static_assert(N <= 15, "Matrix too large for inversion");

    Matrix<N, N> aug;
    Matrix<N, N> result = Matrix<N, N>::identity();

    // 拡大行列を作成
    for (int i = 0; i < N; i++) {
        for (int j = 0; j < N; j++) {
            aug(i, j) = m(i, j);
        }
    }

    // Gauss-Jordan消去法
    for (int i = 0; i < N; i++) {
        // ピボット選択
        int maxRow = i;
        float maxVal = std::abs(aug(i, i));
        for (int k = i + 1; k < N; k++) {
            if (std::abs(aug(k, i)) > maxVal) {
                maxVal = std::abs(aug(k, i));
                maxRow = k;
            }
        }

        // 行の入れ替え
        if (maxRow != i) {
            for (int j = 0; j < N; j++) {
                std::swap(aug(i, j), aug(maxRow, j));
                std::swap(result(i, j), result(maxRow, j));
            }
        }

        // 特異行列チェック
        float pivot = aug(i, i);
        if (std::abs(pivot) < 1e-10f) {
            // 特異行列の場合は単位行列を返す（エラー処理）
            return Matrix<N, N>::identity();
        }

        // ピボット行を正規化
        for (int j = 0; j < N; j++) {
            aug(i, j) /= pivot;
            result(i, j) /= pivot;
        }

        // 他の行を消去
        for (int k = 0; k < N; k++) {
            if (k != i) {
                float factor = aug(k, i);
                for (int j = 0; j < N; j++) {
                    aug(k, j) -= factor * aug(i, j);
                    result(k, j) -= factor * result(i, j);
                }
            }
        }
    }

    return result;
}

/**
 * @brief Vector3を列ベクトル行列に変換
 */
inline Matrix<3, 1> toMatrix(const Vector3& v) {
    Matrix<3, 1> m;
    m(0, 0) = v.x;
    m(1, 0) = v.y;
    m(2, 0) = v.z;
    return m;
}

/**
 * @brief 列ベクトル行列をVector3に変換
 */
inline Vector3 toVector3(const Matrix<3, 1>& m) {
    return Vector3(m(0, 0), m(1, 0), m(2, 0));
}

/**
 * @brief スキュー対称行列生成（外積の行列表現）
 * [v]× * u = v × u
 */
inline Matrix<3, 3> skewSymmetric(const Vector3& v) {
    Matrix<3, 3> m;
    m(0, 0) = 0;     m(0, 1) = -v.z;  m(0, 2) = v.y;
    m(1, 0) = v.z;   m(1, 1) = 0;     m(1, 2) = -v.x;
    m(2, 0) = -v.y;  m(2, 1) = v.x;   m(2, 2) = 0;
    return m;
}

/**
 * @brief クォータニオンから回転行列 (DCM) への変換
 */
inline Matrix<3, 3> quaternionToRotationMatrix(const Quaternion& q) {
    Matrix<3, 3> R;

    float qw = q.w, qx = q.x, qy = q.y, qz = q.z;
    float qw2 = qw*qw, qx2 = qx*qx, qy2 = qy*qy, qz2 = qz*qz;

    R(0, 0) = qw2 + qx2 - qy2 - qz2;
    R(0, 1) = 2.0f * (qx*qy - qw*qz);
    R(0, 2) = 2.0f * (qx*qz + qw*qy);

    R(1, 0) = 2.0f * (qx*qy + qw*qz);
    R(1, 1) = qw2 - qx2 + qy2 - qz2;
    R(1, 2) = 2.0f * (qy*qz - qw*qx);

    R(2, 0) = 2.0f * (qx*qz - qw*qy);
    R(2, 1) = 2.0f * (qy*qz + qw*qx);
    R(2, 2) = qw2 - qx2 - qy2 + qz2;

    return R;
}

}  // namespace math
}  // namespace stampfly
