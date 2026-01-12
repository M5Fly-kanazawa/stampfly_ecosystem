/**
 * @file control_allocation.hpp
 * @brief Control Allocation for X-Quad
 *
 * X-Quadのコントロールアロケーション
 *
 * Allocation Matrix B (forward):
 * アロケーション行列 B（順方向）:
 *   [uₜ, u_φ, u_θ, u_ψ]ᵀ = B × [T₁, T₂, T₃, T₄]ᵀ
 *
 * Mixing Matrix B⁻¹ (inverse):
 * ミキサー行列 B⁻¹（逆方向）:
 *   [T₁, T₂, T₃, T₄]ᵀ = B⁻¹ × [uₜ, u_φ, u_θ, u_ψ]ᵀ
 *
 * Physical units / 物理単位:
 *   uₜ: Total thrust (N) / 総推力
 *   u_φ: Roll torque (Nm) / ロールトルク
 *   u_θ: Pitch torque (Nm) / ピッチトルク
 *   u_ψ: Yaw torque (Nm) / ヨートルク
 *   T₁~T₄: Motor thrusts (N) / モータ推力
 *
 * Motor layout (NED body frame):
 * モータ配置（NED機体座標系）:
 *
 *         Front (NED +X)
 *     FL(M4)     FR(M1)
 *       CW   ▲    CCW
 *         ╲  │  ╱
 *          ╲ │ ╱
 *           ╲│╱
 *           ╱│╲
 *          ╱ │ ╲
 *         ╱  │  ╲
 *       CCW  │   CW
 *     RL(M3)     RR(M2)
 *         Rear
 *
 * @see docs/architecture/control-allocation-migration.md
 */

#pragma once

namespace stampfly {

/**
 * @brief X-Quad configuration parameters
 * X-Quad設定パラメータ
 */
struct QuadConfig {
    // Moment arm (m) / モーメントアーム
    float d = 0.023f;

    // Torque/Thrust ratio κ = Cq/Ct (m)
    // トルク推力比
    float kappa = 9.71e-3f;

    // Motor positions [x, y] in NED body frame (m)
    // モータ位置（NED機体座標系）
    // M1:FR, M2:RR, M3:RL, M4:FL
    float motor_x[4] = {0.023f, -0.023f, -0.023f, 0.023f};
    float motor_y[4] = {0.023f, 0.023f, -0.023f, -0.023f};

    // Motor rotation directions (1=CW, -1=CCW)
    // モータ回転方向
    int motor_dir[4] = {-1, 1, -1, 1};  // CCW, CW, CCW, CW

    // Maximum thrust per motor (N)
    // モータあたりの最大推力
    float max_thrust_per_motor = 0.15f;
};

/**
 * @brief Motor parameters for thrust-to-duty conversion
 * 推力-Duty変換用モータパラメータ
 */
struct MotorParams {
    float Ct = 1.0e-8f;       // Thrust coefficient [N/(rad/s)²]
    float Cq = 9.71e-11f;     // Torque coefficient [Nm/(rad/s)²]
    float Rm = 0.34f;         // Motor resistance [Ω]
    float Km = 6.125e-4f;     // Motor constant [V·s/rad]
    float Dm = 3.69e-8f;      // Viscous damping [Nm·s/rad]
    float Qf = 2.76e-5f;      // Friction torque [Nm]
    float Vbat = 3.7f;        // Battery voltage [V]
};

/**
 * @brief Control allocation for X-Quad
 * X-Quadのコントロールアロケーション
 *
 * Converts between:
 * 変換:
 *   - Control inputs [uₜ, u_φ, u_θ, u_ψ] (N, Nm, Nm, Nm)
 *   - Motor thrusts [T₁, T₂, T₃, T₄] (N)
 *
 * Usage:
 * @code
 * ControlAllocator alloc;
 * alloc.init(QuadConfig{});
 *
 * float control[4] = {0.343f, 0.0f, 0.0f, 0.0f};  // hover
 * float thrusts[4];
 * alloc.mix(control, thrusts);
 *
 * float duties[4];
 * alloc.thrustsToduties(thrusts, duties);
 * @endcode
 */
class ControlAllocator {
public:
    ControlAllocator() = default;

    /**
     * @brief Initialize allocator with configuration
     * 設定でアロケータを初期化
     * @param config Quad configuration / クワッド設定
     */
    void init(const QuadConfig& config);

    /**
     * @brief Initialize motor parameters for thrust-to-duty conversion
     * 推力-Duty変換用モータパラメータを初期化
     * @param params Motor parameters / モータパラメータ
     */
    void setMotorParams(const MotorParams& params);

    /**
     * @brief Inverse allocation (mixing): control inputs → motor thrusts
     * 逆アロケーション（ミキシング）: 制御入力 → モータ推力
     *
     * @param control [uₜ, u_φ, u_θ, u_ψ] control inputs (N, Nm, Nm, Nm)
     * @param thrusts_out [T₁, T₂, T₃, T₄] motor thrusts (N), clamped to [0, max_thrust]
     * @return true if any motor was saturated / 飽和した場合true
     */
    bool mix(const float control[4], float thrusts_out[4]) const;

    /**
     * @brief Forward allocation: motor thrusts → control inputs
     * 順方向アロケーション: モータ推力 → 制御入力
     *
     * @param thrusts [T₁, T₂, T₃, T₄] motor thrusts (N)
     * @param control_out [uₜ, u_φ, u_θ, u_ψ] control inputs (N, Nm, Nm, Nm)
     */
    void allocate(const float thrusts[4], float control_out[4]) const;

    /**
     * @brief Convert motor thrusts to duty cycles
     * モータ推力をDutyサイクルに変換
     *
     * @param thrusts [T₁, T₂, T₃, T₄] motor thrusts (N)
     * @param duties_out [d₁, d₂, d₃, d₄] duty cycles (0.0 to 1.0)
     */
    void thrustsToDuties(const float thrusts[4], float duties_out[4]) const;

    /**
     * @brief Convert single thrust to duty cycle (steady-state approximation)
     * 単一推力をDutyサイクルに変換（定常状態近似）
     *
     * @param thrust Desired thrust (N)
     * @return Duty cycle (0.0 to 1.0)
     */
    float thrustToDuty(float thrust) const;

    // Getters for debugging
    const float* getBMatrix() const { return &B_[0][0]; }
    const float* getBInvMatrix() const { return &B_inv_[0][0]; }
    float getMaxThrustPerMotor() const { return max_thrust_; }

private:
    /**
     * @brief Build allocation and mixing matrices from configuration
     * 設定からアロケーション行列とミキサー行列を構築
     */
    void buildMatrices();

    // Allocation matrix B: u = B × T
    float B_[4][4] = {};

    // Mixing matrix B⁻¹: T = B⁻¹ × u
    float B_inv_[4][4] = {};

    // Configuration
    QuadConfig config_ = {};
    MotorParams motor_params_ = {};
    float max_thrust_ = 0.15f;
};

// Default instances
// デフォルトインスタンス
extern const QuadConfig DEFAULT_QUAD_CONFIG;
extern const MotorParams DEFAULT_MOTOR_PARAMS;

}  // namespace stampfly
