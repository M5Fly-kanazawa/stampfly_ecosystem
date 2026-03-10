/**
 * @file config.hpp
 * @brief Unified configuration (backward compatibility)
 *        統合設定（後方互換性）
 *
 * Includes both platform and vehicle configurations.
 * Existing code that includes config.hpp will continue to work.
 * New code should include the specific config file it needs.
 *
 * プラットフォーム設定と vehicle 設定の両方をインクルード。
 * 既存コードの後方互換性を維持。
 * 新規コードは必要な設定ファイルを個別にインクルードすること。
 */

#pragma once

#include "platform_config.hpp"
#include "vehicle_config.hpp"
