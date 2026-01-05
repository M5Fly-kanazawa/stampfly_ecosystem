# firmware/common

vehicle / controller で共有される組込み向け共通実装。

## ディレクトリ構成

- `protocol/` - protocol/spec に基づく組込み側実装（エンコード・デコード、CRC 等）
- `math/` - 組込み向け数値演算ユーティリティ（行列・ベクトル・フィルタ補助）
- `utils/` - ログ、リングバッファ、汎用ヘルパ

※ 仕様の単一の真実（SSOT）は `protocol/` ディレクトリに置く。

---

# firmware/common

Shared embedded code between vehicle and controller.

## Directory Structure

- `protocol/` - Embedded implementation based on protocol/spec (encode/decode, CRC, etc.)
- `math/` - Embedded-safe math utilities (matrix, vector, filter helpers)
- `utils/` - Logging, ring buffers, generic helpers

Note: The Single Source of Truth (SSOT) for the specification is in the `protocol/` directory.
