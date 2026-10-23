/**
 * Flight Experience Session Survey - Google Forms Auto-Generator
 * StampFly 操縦体験会 アンケート - Google Forms 自動生成スクリプト
 *
 * Usage / 使い方:
 * 1. Open https://script.google.com
 *    https://script.google.com を開く
 * 2. Create a new project
 *    「新しいプロジェクト」を作成する
 * 3. Paste this code
 *    このコードを貼り付ける
 * 4. Run createFlightSurvey()
 *    createFlightSurvey() を実行する
 * 5. Authorize when prompted
 *    実行許可を求められたら承認する
 * 6. Check the log for the form URLs (edit URL and response URL)
 *    ログに出力されるフォームURL（編集用・回答用）を確認する
 */

function createFlightSurvey() {
  var form = FormApp.create('StampFly 操縦体験会 アンケート');

  form.setDescription(
    'StampFly 操縦体験会にご参加いただきありがとうございました。\n' +
    '今後の体験会と教材の改善のため、アンケートにご協力ください（所要時間: 約3分）。\n' +
    '回答は無記名で集計します。'
  );

  form.setIsQuiz(false);
  // Do not collect e-mail addresses automatically (Q14 is an optional normal question)
  // メールアドレスを自動収集しない（Q14 は任意記入の通常設問）
  form.setCollectEmail(false);
  form.setAllowResponseEdits(false);
  form.setLimitOneResponsePerUser(false);
  // Respondents need not sign in. This setting exists only for Google Workspace
  // accounts and may throw on a personal account, so failure is ignored.
  // 回答にログインを求めない。この設定は Google Workspace のアカウントにしかなく、
  // 個人アカウントでは例外になることがあるため、失敗しても続ける。
  try {
    form.setRequireLogin(false);
  } catch (e) {
    Logger.log('setRequireLogin skipped: ' + e);
  }
  form.setProgressBar(true);

  // =====================================================================
  // Section 1: About you / あなたについて
  // =====================================================================

  // A section header (not a page break) so Q1 is on the first page
  // 先頭は改ページでなく見出しにする（開いてすぐ Q1 が見えるように）
  form.addSectionHeaderItem().setTitle('セクション 1: あなたについて');

  var gradeItem = form.addMultipleChoiceItem();
  gradeItem.setTitle('Q1. 学年');
  gradeItem.setChoiceValues([
    '1年',
    '2年'
  ]);
  gradeItem.showOtherOption(true);
  gradeItem.setRequired(true);

  form.addMultipleChoiceItem()
    .setTitle('Q2. ドローンを操縦したことはありますか')
    .setChoiceValues([
      '今日が初めて',
      '何回か飛ばしたことがある（トイドローン等）',
      'よく飛ばしている'
    ])
    .setRequired(true);

  form.addMultipleChoiceItem()
    .setTitle('Q3. プログラミングの経験')
    .setChoiceValues([
      'ほとんどない',
      '授業で習った程度',
      '自分でプログラムを書いたことがある'
    ])
    .setRequired(true);

  // =====================================================================
  // Section 2: Flight experience / 操縦体験について
  // =====================================================================

  form.addPageBreakItem().setTitle('セクション 2: 操縦体験について');

  form.addScaleItem()
    .setTitle('Q4. 操縦は楽しかったですか')
    .setLabels('楽しくなかった', 'とても楽しかった')
    .setBounds(1, 5)
    .setRequired(true);

  form.addScaleItem()
    .setTitle('Q5. 操縦の難しさはどうでしたか')
    .setHelpText('3が「ちょうどよかった」です')
    .setLabels('簡単すぎた', '難しすぎた')
    .setBounds(1, 5)
    .setRequired(true);

  var difficultItem = form.addCheckboxItem();
  difficultItem.setTitle('Q6. 難しかった操作はどれですか（複数選択可）');
  difficultItem.setChoiceValues([
    '前後左右に動かす（ピッチ・ロール）',
    '機首の向きを変える（ヨー）',
    '高さを調整する',
    '横に流れるのを止める',
    '狙った場所に着陸させる',
    '特になし'
  ]);
  difficultItem.showOtherOption(true);
  difficultItem.setRequired(false);

  form.addCheckboxItem()
    .setTitle('Q7. 試した飛行モード（複数選択可）')
    .setChoiceValues([
      'ALT HOLD（高さを自動でキープ）',
      'POS HOLD（高さと位置を自動でキープ）',
      'STABILIZE（高さも自分で操る）',
      'ACRO（上級者向け）'
    ])
    .setRequired(false);

  var guideLevels = [
    '1（分かりにくかった）',
    '2',
    '3',
    '4',
    '5（よく分かった）'
  ];

  var guideGrid = form.addGridItem();
  guideGrid.setTitle('Q8. 配布した操縦ガイドの分かりやすさ');
  guideGrid.setRows([
    'ピッチ・ロール・ヨーの意味',
    'コントローラのスティックとボタンの割り当て'
  ]);
  guideGrid.setColumns(guideLevels);
  guideGrid.setRequired(true);

  // =====================================================================
  // Section 3: Next steps / これから
  // =====================================================================

  form.addPageBreakItem().setTitle('セクション 3: これから');

  var interestItem = form.addCheckboxItem();
  interestItem.setTitle('Q9. もっと知りたい・やってみたいこと（複数選択可）');
  interestItem.setChoiceValues([
    '操縦をもっと上手になりたい',
    'プログラムでドローンを飛ばしたい',
    'ドローンが安定して飛ぶしくみ（制御）を知りたい',
    'センサや電子回路を知りたい',
    'パソコンのシミュレータで飛ばしたい'
  ]);
  interestItem.showOtherOption(true);
  interestItem.setRequired(false);

  // Q10 measures interest in the intensive course (2027-03-01 to 03-05).
  // Q10 は集中講義（2027年3月1日〜5日）への関心を把握する設問。
  form.addMultipleChoiceItem()
    .setTitle(
      'Q10. 2027年3月1日（月）〜5日（金）の集中講義' +
      '「StampFly でドローンの制御を学ぶ5日間」に興味はありますか'
    )
    .setHelpText('詳細・申込方法は、後日 学生ポータルでお知らせします')
    .setChoiceValues([
      'ぜひ受講したい',
      '興味がある',
      '今はまだ分からない',
      '受講しない'
    ])
    .setRequired(true);

  // Q11 shows whether the session would spread by word of mouth.
  // Q11 は体験会が口コミで広がりそうかを見る設問。
  form.addMultipleChoiceItem()
    .setTitle('Q11. この体験会を友だちに勧めたいですか')
    .setChoiceValues([
      'ぜひ勧めたい',
      '勧めてもよい',
      'どちらともいえない',
      '勧めない'
    ])
    .setRequired(true);

  // Q12 collects what content would bring people to a later session.
  // Q12 は 2 回目以降の体験会の内容を決める材料を集める設問。
  var nextItem = form.addCheckboxItem();
  nextItem.setTitle('Q12. 次に体験会があるなら、どんな内容なら参加したいですか（複数選択可）');
  nextItem.setChoiceValues([
    '操縦の続き（POS HOLD・STABILIZE・ACRO を試す）',
    '狙った場所への着陸など、操縦の課題に挑戦する',
    'タイムを競う',
    'プログラムでドローンを飛ばす',
    'パソコンのシミュレータで練習する',
    '今のところ参加の予定はない'
  ]);
  nextItem.showOtherOption(true);
  nextItem.setRequired(false);

  form.addParagraphTextItem()
    .setTitle('Q13. 感想・要望など自由にお書きください')
    .setRequired(false);

  // =====================================================================
  // Section 4: Contact / 連絡先（任意）
  // =====================================================================

  form.addPageBreakItem().setTitle('セクション 4: 連絡先（任意）');

  form.addTextItem()
    .setTitle('Q14. メールアドレス（任意）')
    .setHelpText(
      '集中講義や次回の体験会の案内を希望する方のみご記入ください。' +
      'アンケート自体は無記名で集計します。'
    )
    .setValidation(FormApp.createTextValidation().requireTextIsEmail().build())
    .setRequired(false);

  // =====================================================================
  // Confirmation message / 送信後メッセージ
  // =====================================================================

  form.setConfirmationMessage('ご回答ありがとうございました。');

  // Log the form URLs
  Logger.log('Form created successfully!');
  Logger.log('Edit URL: ' + form.getEditUrl());
  Logger.log('Response URL: ' + form.getPublishedUrl());

  return form;
}
