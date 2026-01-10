#!/usr/bin/env python3
"""
01_hello_genesis.py - Genesis初期化テスト
Genesis initialization test

目的/Purpose:
- Genesisが正しくインストールされているか確認
- Verify Genesis is correctly installed
- ビューアウィンドウが開くか確認
- Verify viewer window opens
"""

import genesis as gs

def main():
    print("=" * 50)
    print("Genesis Hello World Test")
    print("=" * 50)

    # Genesis初期化（まずCPUバックエンドで確認）
    # Initialize Genesis (start with CPU backend)
    print("\n[1] Initializing Genesis with CPU backend...")
    gs.init(backend=gs.cpu)
    print("    -> Genesis initialized successfully!")

    # シーン作成
    # Create scene
    print("\n[2] Creating scene with viewer...")
    scene = gs.Scene(show_viewer=True)
    print("    -> Scene created!")

    # シーンビルド
    # Build scene
    print("\n[3] Building scene...")
    scene.build()
    print("    -> Scene built!")

    # シミュレーション実行（5秒間 = 300ステップ @ 60Hz）
    # Run simulation (5 seconds = 300 steps @ 60Hz)
    print("\n[4] Running simulation for 5 seconds...")
    print("    (Close the viewer window or wait to exit)")

    try:
        for i in range(300):
            scene.step()
            if i % 60 == 0:
                print(f"    Step {i}/300 ({i/60:.1f}s)")
    except KeyboardInterrupt:
        print("\n    Interrupted by user")

    print("\n" + "=" * 50)
    print("Test completed successfully!")
    print("=" * 50)

if __name__ == "__main__":
    main()
