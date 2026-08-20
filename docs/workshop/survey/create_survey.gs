/**
 * StampFly Workshop Survey - Google Forms Auto-Generator
 * StampFly 勉強会 2026春 アンケート - Google Forms 自動生成スクリプト
 *
 * Usage / 使い方:
 * 1. Open https://script.google.com
 * 2. Create a new project
 * 3. Paste this code
 * 4. Run createWorkshopSurvey()
 * 5. Authorize when prompted
 * 6. Check the log for the form URL
 */

function createWorkshopSurvey() {
  var form = FormApp.create('StampFly 勉強会 2026春 受講アンケート');

  form.setDescription(
    'StampFly 勉強会 2026春にご参加いただきありがとうございました。\n' +
    '今後の講義改善のため、アンケートにご協力ください（所要時間: 約5分）。\n' +
    '回答は匿名で処理され、講義改善の目的にのみ使用します。'
  );

  form.setIsQuiz(false);
  form.setCollectEmail(false);
  form.setAllowResponseEdits(false);
  form.setLimitOneResponsePerUser(false);

  // =====================================================================
  // Section 1: Overall Evaluation / 全体評価
  // =====================================================================

  form.addPageBreakItem().setTitle('セクション 1: 全体評価');

  form.addScaleItem()
    .setTitle('Q1. 勉強会全体の満足度')
    .setLabels('不満', '大変満足')
    .setBounds(1, 5)
    .setRequired(true);

  form.addScaleItem()
    .setTitle('Q2. 勉強会の難易度は適切でしたか？')
    .setHelpText('3が「ちょうどよい」です')
    .setLabels('簡単すぎる', '難しすぎる')
    .setBounds(1, 5)
    .setRequired(true);

  form.addScaleItem()
    .setTitle('Q3. 講義のペース（進行速度）は適切でしたか？')
    .setHelpText('3が「ちょうどよい」です')
    .setLabels('遅すぎる', '速すぎる')
    .setBounds(1, 5)
    .setRequired(true);

  form.addScaleItem()
    .setTitle('Q4. 講師の説明はわかりやすかったですか？')
    .setLabels('わかりにくい', '大変わかりやすい')
    .setBounds(1, 5)
    .setRequired(true);

  // =====================================================================
  // Section 2: Per-Lesson Evaluation / レッスン別評価
  // =====================================================================

  form.addPageBreakItem().setTitle('セクション 2: レッスン別評価');

  var lessons = [
    'L0: 環境セットアップ',
    'L1: モータ制御',
    'L2: コントローラ入力',
    'L3: LED 制御',
    'L4: IMU センサ',
    'L5: レート P 制御 + 初フライト',
    'L6: システムモデリング',
    'L7: システム同定',
    'L8: PID 制御',
    'L9: 姿勢推定',
    'L10: API リファレンス',
    'L11: 独自ファームウェア開発',
    'L12: Python SDK プログラム飛行',
    'L13: 精密着陸競技会'
  ];

  var comprehensionLevels = [
    '全く理解できなかった',
    'あまり理解できなかった',
    'だいたい理解できた',
    'よく理解できた',
    '完全に理解できた'
  ];

  var gridItem = form.addGridItem();
  gridItem.setTitle('Q5. 各レッスンの理解度を教えてください');
  gridItem.setRows(lessons);
  gridItem.setColumns(comprehensionLevels);
  gridItem.setRequired(true);

  var interestingItem = form.addCheckboxItem();
  interestingItem.setTitle('Q6. 特に面白かった・役に立ったレッスンを選んでください（複数選択可）');
  interestingItem.setChoiceValues(lessons);
  interestingItem.setRequired(false);

  var difficultItem = form.addCheckboxItem();
  difficultItem.setTitle('Q7. 特に難しかった・つまずいたレッスンを選んでください（複数選択可）');
  difficultItem.setChoiceValues(lessons);
  difficultItem.setRequired(false);

  // =====================================================================
  // Section 3: Hands-on & Materials / 実習・教材評価
  // =====================================================================

  form.addPageBreakItem().setTitle('セクション 3: 実習・教材評価');

  form.addScaleItem()
    .setTitle('Q8. ハンズオン（実機での実習）の量は適切でしたか？')
    .setHelpText('3が「ちょうどよい」です')
    .setLabels('少なすぎる', '多すぎる')
    .setBounds(1, 5)
    .setRequired(true);

  form.addScaleItem()
    .setTitle('Q9. スライド教材はわかりやすかったですか？')
    .setLabels('わかりにくい', '大変わかりやすい')
    .setBounds(1, 5)
    .setRequired(true);

  form.addScaleItem()
    .setTitle('Q10. 実機（StampFly）の使いやすさはどうでしたか？')
    .setLabels('使いにくい', '大変使いやすい')
    .setBounds(1, 5)
    .setRequired(true);

  form.addMultipleChoiceItem()
    .setTitle('Q11. 開発環境（ESP-IDF, ビルド, 書き込み）のセットアップで困ったことはありましたか？')
    .setChoiceValues([
      '特に問題なかった',
      '少し困ったが自分で解決できた',
      'TA・講師のサポートで解決できた',
      '最後まで解決できなかった問題があった'
    ])
    .setRequired(true);

  // =====================================================================
  // Section 4: StampFly Ecosystem / エコシステム評価
  // =====================================================================

  form.addPageBreakItem().setTitle('セクション 4: StampFly Ecosystem について');

  form.addScaleItem()
    .setTitle('Q12. StampFly Ecosystem（ファームウェア, ツール, ドキュメント等）の全体構成は理解できましたか？')
    .setLabels('全く理解できなかった', '十分に理解できた')
    .setBounds(1, 5)
    .setRequired(true);

  form.addScaleItem()
    .setTitle('Q13. 勉強会終了後も StampFly Ecosystem を使って開発を続けたいと思いますか？')
    .setLabels('全く思わない', 'ぜひ続けたい')
    .setBounds(1, 5)
    .setRequired(true);

  var ecosystemItem = form.addCheckboxItem();
  ecosystemItem.setTitle('Q14. StampFly Ecosystem の中で、特に便利だった・良かったものを選んでください（複数選択可）');
  ecosystemItem.setChoiceValues([
    'ws:: API（ワークショップ用簡易API）',
    'sf CLI（ビルド・書き込みツール）',
    'Teleplot（リアルタイム可視化）',
    'CLI コンソール（シリアルコマンド）',
    'スライド教材（Beamer / PPTX）',
    'Python SDK',
    'ドキュメント / README'
  ]);
  ecosystemItem.showOtherOption(true);
  ecosystemItem.setRequired(false);

  form.addParagraphTextItem()
    .setTitle('Q15. StampFly Ecosystem に今後追加・改善してほしい機能やツールがあれば教えてください')
    .setRequired(false);

  // =====================================================================
  // Section 5: Learning Outcomes / 学習成果
  // =====================================================================

  form.addPageBreakItem().setTitle('セクション 5: 学習成果');

  var skillAreas = [
    '制御工学の基礎理論（PID, フィードバック）',
    'センサの仕組みと使い方（IMU, ToF, 光学フロー）',
    '組み込みプログラミング（C/C++, ESP-IDF）',
    'システム同定・モデリングの考え方',
    'ドローンの飛行原理',
    'デバッグ・問題解決能力'
  ];

  var improvementLevels = [
    '向上しなかった',
    '少し向上した',
    'ある程度向上した',
    '大きく向上した',
    '非常に大きく向上した'
  ];

  var skillGrid = form.addGridItem();
  skillGrid.setTitle('Q16. 勉強会受講前と比べて、以下の知識・スキルはどの程度向上しましたか？');
  skillGrid.setRows(skillAreas);
  skillGrid.setColumns(improvementLevels);
  skillGrid.setRequired(true);

  form.addScaleItem()
    .setTitle('Q17. 最終課題（精密着陸競技会）で、自分の成果に満足していますか？')
    .setLabels('不満', '大変満足')
    .setBounds(1, 5)
    .setRequired(false);

  // =====================================================================
  // Section 6: Free-form / 自由記述
  // =====================================================================

  form.addPageBreakItem().setTitle('セクション 6: 自由記述');

  form.addParagraphTextItem()
    .setTitle('Q18. 勉強会で最も印象に残ったこと・学んだことを教えてください')
    .setRequired(false);

  form.addParagraphTextItem()
    .setTitle('Q19. 改善してほしい点、追加してほしい内容があれば教えてください')
    .setRequired(false);

  var futureItem = form.addCheckboxItem();
  futureItem.setTitle('Q20. 今後、以下のような発展コースがあれば参加したいですか？（複数選択可）');
  futureItem.setChoiceValues([
    '自律飛行（GPS/ビジョンベース）',
    '機械学習を用いた制御',
    '群制御（マルチドローン）',
    'シミュレーション（SILS/HILS）',
    'ドローン自作（機体設計・電子回路）',
    'ROS 2 連携'
  ]);
  futureItem.showOtherOption(true);
  futureItem.setRequired(false);

  form.addParagraphTextItem()
    .setTitle('Q21. その他、講師・運営へのメッセージがあればお願いします')
    .setRequired(false);

  // =====================================================================
  // Confirmation message / 送信後メッセージ
  // =====================================================================

  form.setConfirmationMessage(
    'アンケートにご回答いただきありがとうございました！\n' +
    'いただいたフィードバックは今後の講義改善に活用させていただきます。'
  );

  // Log the form URL
  Logger.log('Form created successfully!');
  Logger.log('Edit URL: ' + form.getEditUrl());
  Logger.log('Response URL: ' + form.getPublishedUrl());

  return form;
}
