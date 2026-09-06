#!/usr/bin/env python3
"""
コミット選択機能付き進捗投稿生成スクリプト

Usage:
    generate_post.py -n 5                    # 直近5件
    generate_post.py --select                # インタラクティブ選択
    generate_post.py --commits abc123,def456 # ハッシュ指定
    generate_post.py --range abc123..def456  # 範囲指定
"""

import argparse
import subprocess
import sys
from pathlib import Path


def get_commits_by_count(count: int) -> list[dict]:
    """直近N件のコミットを取得"""
    result = subprocess.run(
        ["git", "log", f"-{count}", "--format=%H|%s|%b"],
        capture_output=True,
        text=True,
        encoding="utf-8",
        errors="replace",
        check=True,
    )
    return parse_git_log(result.stdout)


def get_commits_by_days(days: int) -> list[dict]:
    """直近N日間のコミットを取得"""
    result = subprocess.run(
        ["git", "log", f"--since={days} days ago", "--format=%H|%s|%b"],
        capture_output=True,
        text=True,
        encoding="utf-8",
        errors="replace",
        check=True,
    )
    return parse_git_log(result.stdout)


def get_commits_by_hashes(hashes: list[str]) -> list[dict]:
    """指定されたハッシュのコミットを取得"""
    commits = []
    for hash in hashes:
        result = subprocess.run(
            ["git", "show", hash, "--format=%H|%s|%b", "--no-patch"],
            capture_output=True,
            text=True,
            encoding="utf-8",
            errors="replace",
            check=True,
        )
        commits.extend(parse_git_log(result.stdout))
    return commits


def get_commits_by_range(range_spec: str) -> list[dict]:
    """範囲指定でコミットを取得"""
    result = subprocess.run(
        ["git", "log", range_spec, "--format=%H|%s|%b"],
        capture_output=True,
        text=True,
        encoding="utf-8",
        errors="replace",
        check=True,
    )
    return parse_git_log(result.stdout)


def parse_git_log(output: str) -> list[dict]:
    """git log の出力をパース"""
    commits = []
    current_commit = None

    for line in output.split('\n'):
        if '|' in line and len(line.split('|')[0]) == 40:  # コミットハッシュ行
            if current_commit:
                commits.append(current_commit)
            parts = line.split('|', 2)
            current_commit = {
                'hash': parts[0],
                'subject': parts[1],
                'body': parts[2] if len(parts) > 2 else ''
            }
        elif current_commit:
            # 本文の続き
            current_commit['body'] += '\n' + line

    if current_commit:
        commits.append(current_commit)

    return commits


def interactive_select() -> list[dict]:
    """インタラクティブにコミットを選択"""
    # 直近20件を表示
    result = subprocess.run(
        ["git", "log", "-20", "--oneline"],
        capture_output=True,
        text=True,
        encoding="utf-8",
        errors="replace",
        check=True,
    )

    lines = result.stdout.strip().split('\n')

    print("=== コミット選択 ===")
    for i, line in enumerate(lines, 1):
        print(f"{i:2d}. {line}")
    print()
    print("選択したいコミット番号をカンマ区切りで入力 (例: 1,2,5):")

    selection = input("> ").strip()
    selected_indices = [int(x.strip()) - 1 for x in selection.split(',')]

    hashes = [lines[i].split()[0] for i in selected_indices]
    return get_commits_by_hashes(hashes)


def extract_section(body: str, section_name: str) -> list[str]:
    """コミット本文から特定セクションの項目を抽出"""
    if section_name not in body:
        return []

    # セクション開始位置を探す
    start_idx = body.find(section_name)
    if start_idx == -1:
        return []

    # セクション内容を取得（次のセクションまたは本文終了まで）
    section_start = start_idx + len(section_name)
    rest_of_body = body[section_start:]

    # 次のセクション（大文字で始まる行）を探す
    lines = rest_of_body.split('\n')
    section_lines = []
    for line in lines:
        # 空行はスキップ
        if not line.strip():
            continue
        # 次のセクション（大文字で始まる、または "Next steps:" など）なら終了
        if line.strip() and not line.startswith((' ', '-', '  -')) and ':' in line:
            if line.split(':')[0].strip() not in section_lines:
                break
        # 箇条書き項目を収集
        if line.strip().startswith('-'):
            section_lines.append(line.strip()[2:])  # "- " を削除

    return section_lines


def generate_post(commits: list[dict], user_message: str = "") -> str:
    """投稿文を生成"""
    if not commits:
        return ""

    # 冒頭（背景・動機 + コミット数明示）
    intro = user_message + "\n\n" if user_message else ""

    # コミットの種類から背景・動機を生成
    commit_types = [c['subject'].split('(')[0].split(':')[0] if ':' in c['subject'] else '' for c in commits]
    scopes = [c['subject'].split('(')[1].split(')')[0] if '(' in c['subject'] else '' for c in commits]

    # 背景文を生成（Why + What）
    has_feat = any('feat' in t for t in commit_types)
    has_fix = any('fix' in t for t in commit_types)
    has_workflow = any('workflow' in s for s in scopes)
    has_cli = any('cli' in s for s in scopes)
    has_control = any('control' in s for s in scopes)

    # 全コミットから日本語の背景説明を探す（「なぜ」を含む文のみ）
    background_text = None
    for commit in commits[:2]:
        body = commit['body'].strip()
        first_para = body.split('\n\n')[0] if body else ""
        # 日本語を含み、かつ「ため」「問題」「課題」などの文脈的キーワードを含む段落のみ
        if first_para and any(ord(c) > 127 for c in first_para):
            if any(keyword in first_para for keyword in ['ため', '問題', '課題', 'により', 'ことで', 'なかった', '必要']):
                background_text = first_para
                break

    if background_text:
        # コミットメッセージに文脈的な背景があればそれを使う
        intro += background_text + "\n\n"
    elif has_cli and has_feat:
        # CLI機能追加の場合
        intro += f"StampFlyの開発効率を上げるため、PCから直接機体を操作できる機能を実装した（{len(commits)}件のコミット）。従来はファームウェアを書き込んで実機テストするしかなく、開発サイクルが遅かった。\n\n"
    elif has_workflow:
        # ワークフロー改善の場合
        intro += f"開発ワークフローを改善し、作業効率を大幅に向上させた（{len(commits)}件のコミット）。\n\n"
    elif has_control and has_feat:
        # 制御機能追加の場合
        intro += f"StampFlyの制御性能を向上させるため、新しい制御モードを実装した（{len(commits)}件のコミット）。\n\n"
    elif has_fix:
        # バグ修正の場合
        intro += f"StampFlyの動作を安定させるため、いくつかの問題を修正した（{len(commits)}件のコミット）。\n\n"
    else:
        # デフォルト
        intro += f"StampFlyドローンの開発を進めた（{len(commits)}件のコミット）。\n\n"

    # 実装内容（Changes セクションから抽出 - 成果として表現）
    all_changes = []
    for commit in commits:
        changes = extract_section(commit['body'], 'Changes:')
        all_changes.extend(changes[:3])  # 各コミットから最大3項目

    impl_text = "実装内容：\n"
    for change in all_changes[:5]:  # 最大5項目
        # 「Add XXX」→「XXXを実装」のような読みやすい表現に変換
        readable_change = change
        if change.startswith('Add '):
            readable_change = change.replace('Add ', '', 1)
            # さらに具体的な成果表現に変換（汎用的に）
            if 'packet_parser' in readable_change:
                readable_change = "パケット解析モジュールの実装（WiFiキャプチャから抽出）"
            elif 'vehicle_connection' in readable_change:
                readable_change = "機体接続ライブラリの実装（TCP CLI + WebSocketテレメトリ）"
            elif 'flight.py' in readable_change:
                readable_change = "4つのフライトコマンドの実装（takeoff/land/hover/jump）"
            elif 'POSITION_HOLD' in readable_change or 'FlightMode' in readable_change:
                readable_change = "POSITION_HOLDモードの追加（位置保持制御）"
            elif 'PID' in readable_change:
                readable_change = readable_change  # PID関連はそのまま
            elif '.hpp' in readable_change or '.cpp' in readable_change:
                # ファイル名が含まれる場合は、括弧内に移動
                parts = readable_change.split('(')
                if len(parts) > 1:
                    readable_change = f"{parts[0].strip()}（{parts[1]}"
        elif change.startswith('Update '):
            readable_change = change.replace('Update ', '', 1) + 'を更新'
        elif change.startswith('Create '):
            readable_change = change.replace('Create ', '', 1) + 'を新規作成'

        impl_text += f"・{readable_change}\n"
    impl_text += "\n"

    # 詳細セクション（複数のセクションから生成 - ストーリー性重視）
    details = []
    for commit in commits[:3]:  # 最大3件の詳細
        body = commit['body'].strip()
        subject = commit['subject'].split(':', 1)[1].strip() if ':' in commit['subject'] else commit['subject']

        # コミットメッセージの最初の詳細段落を取得（Changes: より前）
        first_detail = ""
        if 'Changes:' in body:
            first_detail = body.split('Changes:')[0].strip().split('\n\n')[0]
            # subject と同じ内容は除外
            if first_detail and first_detail != commit['subject'] and len(first_detail) > 50:
                # 日本語がなければ英語の説明を活用
                if not any(ord(c) > 127 for c in first_detail):
                    # 英語の説明を読みやすい日本語に変換（制御系の場合）
                    if 'Position Hold' in first_detail or 'cascade PID' in first_detail:
                        details.append(f"◆制御アーキテクチャ: {first_detail}")
                    else:
                        details.append(f"◆概要: {first_detail}")

        # Control structure セクション（制御フロー説明）
        control_items = extract_section(body, 'Control structure:')
        if control_items and len(control_items) > 0:
            control_text = "。".join(control_items[:2])
            details.append(f"◆制御フロー: {control_text}")

        # Safety セクション（安全機構説明）
        safety_items = extract_section(body, 'Safety:')
        if safety_items and len(safety_items) > 0:
            safety_text = "。".join(safety_items[:3])
            details.append(f"◆安全機構: {safety_text}")

        # Architecture セクション（因果関係を説明）
        arch_items = extract_section(body, 'Architecture:')
        if arch_items and len(arch_items) > 0:
            # 技術詳細を読みやすく変換
            arch_text = "。".join(arch_items[:3])
            # より読みやすい表現に変換
            if 'CLI' in arch_text and 'WebSocket' in arch_text:
                details.append(f"◆アーキテクチャ: WiFi経由で機体にコマンドを送信し、400Hzの高速テレメトリで機体状態をリアルタイム監視する設計。TCP接続でコマンド送信、WebSocketで高速データ受信という役割分担により、PCから機体の詳細な動作を把握しながら操作できる環境が実現した")
            else:
                details.append(f"◆アーキテクチャ: {arch_text}")

        # Commands セクション（何ができるようになったかを説明）
        cmd_items = extract_section(body, 'Commands:')
        if cmd_items and len(cmd_items) > 0:
            if 'takeoff' in str(cmd_items) and 'land' in str(cmd_items):
                details.append(f"◆実現した機能: PCのターミナルから `sf takeoff 0.5` と入力するだけで機体が離陸し、`sf land` で着陸する。ホバリングテストやジャンプ動作も簡単に試せるようになり、従来のファームウェア書き込み→実機起動→テストという煩雑な手順が不要になった")
            else:
                cmd_desc = "、".join(cmd_items[:4])
                details.append(f"◆コマンド: {cmd_desc}")

        # Implementation notes セクション（工夫点を語る）
        tech_items = extract_section(body, 'Implementation notes:')
        if tech_items and len(tech_items) > 0:
            tech_text = "。".join(tech_items[:3])
            if 'No new dependencies' in tech_text:
                details.append(f"◆実装の工夫: 既存のWebSocketライブラリを活用し、新たな依存を増やさずに実装。WiFi CLIのtelnetプロトコルにも対応し、既存のファームウェアとシームレスに連携できるようにした。0.5秒間隔でフライト状態をポーリングすることで、リアルタイム性と負荷のバランスを取った")
            else:
                details.append(f"◆技術詳細: {tech_text}")

        # User Benefits セクション（変化を具体的に）
        benefit_items = extract_section(body, 'User Benefits:')
        if benefit_items and len(benefit_items) > 0:
            benefit_desc = "。".join(benefit_items[:3])
            details.append(f"◆開発体験の変化: {benefit_desc}")

    # 詳細が多い場合は調整（最大5セクション）
    if len(details) > 5:
        details = details[:5]

    detail_text = "\n\n".join(details) + "\n\n" if details else ""

    # 締め（成果のインパクト - 何が改善されたか）
    if has_cli and has_feat:
        closing = "PCから直接機体を操作できるようになり、開発サイクルが大幅に短縮。実機テストの効率が劇的に向上した\n\n"
    elif has_workflow and len(commits) >= 3:
        closing = "これらのツール改善により、開発から進捗共有までのワークフロー全体が効率化。日々の開発体験が大きく向上した\n\n"
    elif has_control and has_feat:
        closing = "新しい制御モードにより、より高度な飛行が可能に。実用性が大きく向上した\n\n"
    elif has_fix:
        closing = "これらの修正により、動作が安定し、実機テストの信頼性が向上した\n\n"
    elif len(commits) >= 3:
        closing = "これらの改善により、StampFly開発環境が大幅に強化され、開発効率が向上した\n\n"
    else:
        closing = "StampFly開発が一歩前進した\n\n"

    # ハッシュタグ（コミット内容に応じて調整）
    tags = ["#StampFly"]
    # scope に応じてタグを追加
    scopes = [c['subject'].split('(')[1].split(')')[0] if '(' in c['subject'] else "" for c in commits]
    if any('cli' in s or 'flight' in s or 'workflow' in s for s in scopes):
        tags.append("#ドローン開発")
    if any('control' in s for s in scopes):
        tags.append("#制御工学")
    tags.append("#ESP32")
    hashtags = " ".join(tags[:3])  # 最大3個

    post = intro + impl_text + detail_text + closing + hashtags

    # 文字数制限チェック（段階的に調整、目標800-900文字）
    # X (Twitter) は現在数千文字まで可能だが、読みやすさのため1000文字程度に抑える
    if len(post) > 1000:
        # Step 1: 詳細セクションを4つに削減
        if len(details) > 4:
            detail_text = "\n\n".join(details[:4]) + "\n\n"
            post = intro + impl_text + detail_text + closing + hashtags

    if len(post) > 1000:
        # Step 2: 詳細セクションを3つに削減
        if len(details) > 3:
            detail_text = "\n\n".join(details[:3]) + "\n\n"
            post = intro + impl_text + detail_text + closing + hashtags

    if len(post) > 1000:
        # Step 3: 実装内容を3項目に削減
        impl_text = "実装内容：\n"
        for change in all_changes[:3]:
            impl_text += f"・{change}\n"
        impl_text += "\n"
        post = intro + impl_text + detail_text + closing + hashtags

    return post


def main():
    parser = argparse.ArgumentParser(description='進捗投稿生成')
    group = parser.add_mutually_exclusive_group(required=True)
    group.add_argument('-n', type=int, help='直近N件のコミット')
    group.add_argument('--day', type=int, help='直近N日間のコミット')
    group.add_argument('--select', action='store_true', help='インタラクティブ選択（ターミナル専用）')
    group.add_argument('--list', type=int, default=None, nargs='?', const=20, help='コミット一覧をJSON出力（デフォルト20件）')
    group.add_argument('--commits', help='コミットハッシュ（カンマ区切り）')
    group.add_argument('--range', help='範囲指定（例: abc123..def456）')

    parser.add_argument('-m', '--message', help='冒頭に追加するユーザーメッセージ')
    parser.add_argument('--dry-run', action='store_true', help='プレビューのみ')

    args = parser.parse_args()

    # --list: コミット一覧をJSON出力（Claude Code用）
    if args.list is not None:
        import json
        result = subprocess.run(
            ["git", "log", f"-{args.list}", "--format=%H|%s"],
            capture_output=True,
            text=True,
            encoding="utf-8",
            errors="replace",
            check=True,
        )
        commits_list = []
        for line in result.stdout.strip().split('\n'):
            if '|' in line:
                hash, subject = line.split('|', 1)
                commits_list.append({"hash": hash[:7], "subject": subject})
        print(json.dumps(commits_list, ensure_ascii=False, indent=2))
        sys.exit(0)

    # コミット取得
    if args.n:
        commits = get_commits_by_count(args.n)
    elif args.day:
        commits = get_commits_by_days(args.day)
    elif args.select:
        commits = interactive_select()
    elif args.commits:
        hashes = [h.strip() for h in args.commits.split(',')]
        commits = get_commits_by_hashes(hashes)
    elif args.range:
        commits = get_commits_by_range(args.range)

    if not commits:
        print("エラー: コミットが見つかりません")
        sys.exit(1)

    # 投稿文生成
    post = generate_post(commits, args.message or "")

    # プレビュー表示
    print("=== 投稿プレビュー ===")
    print(post)
    print()
    print(f"文字数: {len(post)}文字")
    print()

    if args.dry_run:
        print("--dry-run モードのため、投稿しません")
        sys.exit(0)

    # 確認
    print("この内容で投稿しますか？ (y/N): ", end='')
    confirm = input().strip().lower()

    if confirm != 'y':
        print("キャンセルしました")
        sys.exit(0)

    # 投稿実行
    import os
    script_dir = Path(__file__).parent
    post_script = script_dir / "post_to_x.py"

    result = subprocess.run(
        [sys.executable, str(post_script), post],
        env=os.environ.copy(),
    )

    sys.exit(result.returncode)


if __name__ == "__main__":
    main()
