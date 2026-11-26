// Shared layout helpers for the B and C decks (Anthropic pptx skill, structured-deck style).
const path = require("path");
const pptxgen = require("pptxgenjs");

// Graphite structure, orange for "fails", teal for "robust".
// Fonts (user's choice): Latin text in Times New Roman, Chinese in 华文行楷 (STXingkai).
// pptxgenjs only writes the Latin slot, so setThemeFonts() below fills the East-Asian slots.
const LATIN_FONT = "Times New Roman";
const EA_FONT = "华文行楷";
const THEME = {
  name: "ORB-SLAM3 degradation",
  headFontFace: LATIN_FONT,
  bodyFontFace: LATIN_FONT,
  colors: {
    dk1: "1F2328", lt1: "FFFFFF", dk2: "263238", lt2: "F1F3F5",
    accent1: "E4572E", accent2: "17A398", accent3: "2B6CB0", accent4: "F2A541",
    accent5: "6C757D", accent6: "8E44AD", hlink: "2B6CB0", folHlink: "6C757D",
  },
};

function makeDeck(title, footer) {
  const pres = new pptxgen();
  pres.layout = "LAYOUT_WIDE"; // 13.33 x 7.5 in
  pres.title = title;
  pres.theme = { headFontFace: THEME.headFontFace, bodyFontFace: THEME.bodyFontFace };
  const C = pres.SchemeColor;

  pres.defineSlideMaster({
    title: "DARK_TITLE",
    background: { color: C.text2 },
    objects: [
      { placeholder: { options: { name: "title", type: "title", x: 0.8, y: 2.2, w: 11.7, h: 1.4, fontSize: 40, bold: true, color: C.background1, valign: "bottom" }, text: "" } },
      { placeholder: { options: { name: "body", type: "body", x: 0.8, y: 3.75, w: 11.7, h: 1.6, fontSize: 18, color: C.background2, valign: "top" }, text: "" } },
    ],
  });
  pres.defineSlideMaster({
    title: "CONTENT",
    background: { color: C.background1 },
    margin: [0.5, 0.6, 0.5, 0.6],
    objects: [
      { placeholder: { options: { name: "title", type: "title", x: 0.6, y: 0.35, w: 12.1, h: 0.75, fontSize: 30, bold: true, color: C.text2, valign: "middle" }, text: "" } },
      { placeholder: { options: { name: "body", type: "body", x: 0.6, y: 1.1, w: 12.1, h: 0.55, fontSize: 16, color: C.accent1, bold: true, valign: "top" }, text: "" } },
      { text: { text: footer, options: { x: 0.6, y: 7.05, w: 8, h: 0.3, fontSize: 10, color: C.accent5 } } },
    ],
    slideNumber: { x: 12.3, y: 7.05, w: 0.5, h: 0.3, fontSize: 10, color: C.accent5 },
  });

  const d = { pres, C };
  d.cover = (section, titleText, sub, notes) => {
    const s = pres.addSlide({ masterName: "DARK_TITLE", sectionTitle: section });
    s.addText(titleText, { placeholder: "title" });
    s.addText(sub, { placeholder: "body" });
    if (notes) s.addNotes(notes);
    return s;
  };
  d.content = (section, titleText, takeaway) => {
    const s = pres.addSlide({ masterName: "CONTENT", sectionTitle: section });
    s.addText(titleText, { placeholder: "title" });
    if (takeaway) s.addText(takeaway, { placeholder: "body" });
    return s;
  };
  d.bullets = (s, items, x, y, w, h, size = 14) =>
    s.addText(
      items.map((t, i) => ({ text: t, options: { bullet: true, breakLine: i < items.length - 1, paraSpaceAfter: 6 } })),
      { x, y, w, h, fontSize: size, color: C.text1, valign: "top", isTextBox: true, objectName: "Bullets" }
    );
  d.caption = (s, text, x, y, w) =>
    s.addText(text, { x, y, w, h: 0.3, fontSize: 10, color: C.accent5, italic: true, isTextBox: true, margin: 0, objectName: "Caption" });
  // picture fitted into a box, aspect ratio kept
  d.image = (s, file, ratio, x, y, w, h, name) => {
    let iw = w, ih = w / ratio;
    if (ih > h) { ih = h; iw = h * ratio; }
    s.addImage({ path: file, x: x + (w - iw) / 2, y: y + (h - ih) / 2, w: iw, h: ih, objectName: name, altText: name });
  };
  d.card = (s, x, y, w, h, big, label, color) => {
    s.addShape(pres.shapes.ROUNDED_RECTANGLE, { x, y, w, h, rectRadius: 0.12, fill: { color: C.background2 }, line: { color: C.background2 }, objectName: "Card" });
    s.addText(big, { x: x + 0.2, y: y + 0.15, w: w - 0.4, h: h * 0.5, fontSize: 32, bold: true, color, valign: "bottom", isTextBox: true, margin: 0 });
    s.addText(label, { x: x + 0.2, y: y + 0.15 + h * 0.5, w: w - 0.4, h: h * 0.45 - 0.2, fontSize: 13, color: C.text1, valign: "top", isTextBox: true, margin: 0 });
  };
  d.table = (s, rows, x, y, w, colW, fontSize = 12) => {
    const head = rows[0].map(t => ({ text: t, options: { bold: true, color: C.background1, fill: { color: C.text2 } } }));
    const body = rows.slice(1).map((r, i) => r.map(t => ({ text: t, options: { fill: { color: i % 2 ? C.background1 : C.background2 } } })));
    s.addTable([head, ...body], { x, y, w, colW, fontSize, color: C.text1, border: { type: "solid", pt: 0.5, color: "D0D5DA" }, valign: "middle", objectName: "Table" });
  };
  d.save = async (out, skillDir) => {
    const { applyTheme } = require(path.join(skillDir, "scripts/apply_theme.js"));
    await pres.writeFile({ fileName: out });
    await applyTheme(out, THEME);
    setThemeFonts(out);
    console.log("written", out);
  };
  return d;
}

// Put the Chinese font into the theme's East-Asian slots (<a:ea> and script="Hans") of both
// major (headings) and minor (body) fonts, so mixed text renders Latin in Times New Roman and
// Chinese in 华文行楷 without per-run overrides.
function setThemeFonts(file) {
  const { execFileSync } = require("child_process");
  execFileSync("python3", ["-c", `
import sys, zipfile, shutil, os, re
src = sys.argv[1]; ea = sys.argv[2]; tmp = src + ".tmp"
with zipfile.ZipFile(src) as zin, zipfile.ZipFile(tmp, "w", zipfile.ZIP_DEFLATED) as zout:
    for item in zin.infolist():
        data = zin.read(item.filename)
        if item.filename.startswith("ppt/theme/theme") and item.filename.endswith(".xml"):
            t = data.decode("utf-8")
            t = re.sub(r'<a:ea typeface="[^"]*"/>', '<a:ea typeface="%s"/>' % ea, t)
            t = re.sub(r'(<a:font script="Hans" typeface=")[^"]*(")', r'\\g<1>%s\\g<2>' % ea, t)
            data = t.encode("utf-8")
        zout.writestr(item, data)
os.replace(tmp, src)
`, file, EA_FONT]);
}

module.exports = { makeDeck };
