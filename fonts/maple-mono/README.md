# Maple Mono

Unmodified variable WOFF2 fonts from Maple Mono v7.9:
https://github.com/subframe7536/maple-font/tree/v7.9/woff2/var

- MapleMono[wght]-VF.woff2 → MapleMono-Variable.woff2
- MapleMono-Italic[wght]-VF.woff2 → MapleMono-Italic-Variable.woff2

Licensed under SIL Open Font License 1.1; see OFL.txt.
These two variable fonts do not include Chinese glyphs. Other blog pages use the
system fallback for Chinese; the running page adds the Chinese fonts below.

## Running page Chinese glyphs

`MapleMonoCN-Regular.woff2` and `MapleMonoCN-SemiBold.woff2` are subsets of
`MapleMono-NF-CN-Regular.ttf` and `MapleMono-NF-CN-SemiBold.ttf` from the same
[v7.9 release](https://github.com/subframe7536/maple-font/releases/tag/v7.9).
They retain the original copyright and license metadata and use the OFL above.

These two fonts are used only by the running page. They include GB2312 level-one
Chinese characters, CJK punctuation, kana, and additional characters currently
used in the running components and activity summaries. Latin text continues to
use the existing variable font. The subsets total about 1.94 MB rather than
shipping the complete CJK fonts. Uncovered characters use the browser fallback.

To regenerate after adding rare characters to labels or location data, extract
the official `MapleMono-NF-CN.zip` and run from the project directory:

```bash
uv run --with fonttools==4.68.0 --with brotli==1.2.1 python scripts/running/subset-font.py --font-dir /path/to/extracted/fonts
```

Font generation is optional maintenance; normal pnpm builds and data syncs do not
need Python. Map tile labels are baked into images and cannot use web fonts.
