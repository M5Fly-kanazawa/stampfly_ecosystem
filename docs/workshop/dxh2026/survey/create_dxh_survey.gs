/**
 * DXH High-School Teacher Workshop Survey - Google Forms Auto-Generator
 * DXH 高校教員向け StampFly 体験講座 アンケート - Google Forms 自動生成スクリプト
 *
 * Usage / 使い方:
 * 1. Open https://script.google.com
 *    https://script.google.com を開く
 * 2. Create a new project
 *    「新しいプロジェクト」を作成する
 * 3. Paste this code
 *    このコードを貼り付ける
 * 4. Run createDxhSurvey()
 *    createDxhSurvey() を実行する
 * 5. Authorize when prompted
 *    実行許可を求められたら承認する
 * 6. Check the log for the form URL
 *    ログに出力されるフォームURLを確認する
 */

function createDxhSurvey() {
  var form = FormApp.create('DXH 高校教員向け StampFly 体験講座 受講アンケート');

  form.setDescription(
    '本日はDXH（高等学校DX加速化推進事業）高校教員向けStampFly体験講座にご参加いただき\n' +
    'ありがとうございました。\n' +
    '今後の教材・講座改善のため、アンケートにご協力ください（所要時間: 約5分）。\n' +
    'このアンケートは無記名で集計されます。メールアドレスの記入は任意で、今後の情報提供や\n' +
    '個別相談をご希望の方のみご記入いただくものです。'
  );

  form.setIsQuiz(false);
  form.setCollectEmail(false);
  form.setAllowResponseEdits(false);
  form.setLimitOneResponsePerUser(false);

  // =====================================================================
  // Section 1: Attributes / 属性
  // =====================================================================

  form.addPageBreakItem().setTitle('セクション 1: 属性');

  var subjectItem = form.addCheckboxItem();
  subjectItem.setTitle('Q1. 担当教科（複数選択可）');
  subjectItem.setChoiceValues([
    '情報',
    '数学',
    '理科（物理）',
    '理科（化学・生物・地学）',
    '工業',
    '総合的な探究の時間'
  ]);
  subjectItem.showOtherOption(true);
  subjectItem.setRequired(true);

  form.addMultipleChoiceItem()
    .setTitle('Q2. プログラミングの経験')
    .setChoiceValues([
      'ほとんど経験がない',
      '授業で基本文法を学んだ程度（Scratch等のビジュアル言語を含む）',
      '自分でテキストベースの言語（Python, C, JavaScript等）を書いたことがある',
      '授業で生徒にプログラミングを指導した経験がある',
      '業務・研究で日常的にプログラミングをしている'
    ])
    .setRequired(true);

  var droneExpItem = form.addMultipleChoiceItem();
  droneExpItem.setTitle('Q3. ドローンの操縦・プログラミング経験');
  droneExpItem.setChoiceValues([
    '操縦もプログラミングも今回が初めて',
    '操縦（トイドローン等）をしたことがある',
    'ドローンのプログラミングや電子工作をしたことがある'
  ]);
  droneExpItem.showOtherOption(true);
  droneExpItem.setRequired(true);

  // =====================================================================
  // Section 2: Comprehension & Difficulty per Part / 各パートの理解度・難易度
  // =====================================================================

  form.addPageBreakItem().setTitle('セクション 2: 各パートの理解度・難易度');

  var parts = [
    '①操縦体験（シミュレータ＋実機の自動離陸・スティック操作・自動着陸）',
    '②開発環境・Webフラッシャの紹介（デモ）',
    '②書き込み実習（sf flash でのファームウェア書き込み）',
    '③プログラム書き換え・モータ制御実習'
  ];

  var comprehensionLevels = [
    '全く理解できなかった',
    'あまり理解できなかった',
    'だいたい理解できた',
    'よく理解できた',
    '完全に理解できた'
  ];

  var comprehensionGrid = form.addGridItem();
  comprehensionGrid.setTitle('Q4. 各パートの理解度を教えてください');
  comprehensionGrid.setRows(parts);
  comprehensionGrid.setColumns(comprehensionLevels);
  comprehensionGrid.setRequired(true);

  var difficultyLevels = [
    '簡単すぎた',
    'やや簡単だった',
    'ちょうどよかった',
    'やや難しかった',
    '難しすぎた'
  ];

  var difficultyGrid = form.addGridItem();
  difficultyGrid.setTitle('Q5. 各パートの難易度は適切でしたか？');
  difficultyGrid.setHelpText('3が「ちょうどよかった」です');
  difficultyGrid.setRows(parts);
  difficultyGrid.setColumns(difficultyLevels);
  difficultyGrid.setRequired(true);

  // =====================================================================
  // Section 3: Overall Satisfaction / 全体満足度
  // =====================================================================

  form.addPageBreakItem().setTitle('セクション 3: 全体満足度');

  form.addScaleItem()
    .setTitle('Q6. 講座全体の満足度')
    .setLabels('不満', '大変満足')
    .setBounds(1, 5)
    .setRequired(true);

  // =====================================================================
  // Section 4: Intent to Adopt in Class / 授業への導入意向
  // =====================================================================

  form.addPageBreakItem().setTitle('セクション 4: 授業への導入意向');

  form.addScaleItem()
    .setTitle('Q7. ご自身の授業や活動で StampFly を使ってみたいと思いますか？')
    .setHelpText(
      '「使う」とは、担当科目の授業や部活動・総合的な探究の時間等でStampFlyや' +
      'ドローン教材を取り入れることを指します'
    )
    .setLabels('全く思わない', 'ぜひ使ってみたい')
    .setBounds(1, 5)
    .setRequired(true);

  form.addParagraphTextItem()
    .setTitle('Q8. どのような使い方を想定していますか？')
    .setHelpText(
      '例: 情報科の授業で1コマ／総合的な探究の時間で／部活動・課外活動で／教員研修で　等'
    )
    .setRequired(false);

  // Q9 is the most important question in this survey: the barrier list feeds
  // the "data flywheel" that prioritizes future teaching-material and support work.
  // Q9はこのアンケートの中核設問: 障壁の回答が、今後の教材・支援策の優先順位付けの
  // 起点となる「データフライホイール」に直結する。
  var barrierItem = form.addCheckboxItem();
  barrierItem.setTitle('Q9. ★ 授業に導入する上での障壁を教えてください（複数選択可・最重要）');
  barrierItem.setHelpText(
    'いただいた回答は、今後の教材開発・支援策の検討に直接反映します' +
    '（教員の声を起点とした改善の第一歩です）'
  );
  barrierItem.setChoiceValues([
    '予算（機材購入・維持費）',
    '授業時間の確保',
    '安全面（飛行スペース・保護具・保険等）',
    '自身の指導スキル・自信',
    'カリキュラムとの適合性（学習指導要領等との整合）'
  ]);
  barrierItem.showOtherOption(true);
  barrierItem.setRequired(true);

  // =====================================================================
  // Section 5: Free-form / 自由記述
  // =====================================================================

  form.addPageBreakItem().setTitle('セクション 5: 自由記述');

  form.addParagraphTextItem()
    .setTitle('Q10. ご自由にお書きください')
    .setHelpText(
      '以下の観点でご自由にお書きください（すべて任意です）\n' +
      '・講座で良かった点\n' +
      '・改善してほしい点\n' +
      '・今後あると助かる教材や支援（指導案、追加教材、相談窓口 等）'
    )
    .setRequired(false);

  // =====================================================================
  // Section 6: Future Contact / 今後の連絡
  // =====================================================================

  form.addPageBreakItem().setTitle('セクション 6: 今後の連絡');

  form.addTextItem()
    .setTitle('Q11. メールアドレス（任意）')
    .setHelpText(
      '今後の情報提供（教材配布・イベント案内等）や個別相談をご希望の方のみご記入ください。' +
      'このアンケート自体は無記名で集計されるため、記入の有無は他の回答の匿名性に影響しません。'
    )
    .setValidation(FormApp.createTextValidation().requireTextIsEmail().build())
    .setRequired(false);

  // =====================================================================
  // Confirmation message / 送信後メッセージ
  // =====================================================================

  form.setConfirmationMessage(
    'アンケートにご回答いただきありがとうございました！\n' +
    'いただいたフィードバックは今後の講座・教材の改善に活用させていただきます。'
  );

  // Log the form URL
  Logger.log('Form created successfully!');
  Logger.log('Edit URL: ' + form.getEditUrl());
  Logger.log('Response URL: ' + form.getPublishedUrl());

  return form;
}
